function mdl = build_sm_corner(H, mdl)
%BUILD_SM_CORNER  One double-wishbone corner as a Simscape Multibody linkage.
%
%   BUILD_SM_CORNER(H) builds the left corner described by the hardpoint set H
%   (see sm_hardpoints) as a real linkage: two wishbones on revolute chassis
%   pivots, a track rod, and an upright joined to all three by ball joints.
%
%   WHAT THIS IS FOR. The plant today has no suspension geometry -- it has
%   three chosen numbers standing in for the consequences of geometry:
%   Susp.MotionRatioFront, Susp.CamberGainFront and Susp.ArbArmRadiusFront.
%   Here the linkage IS the model, so camber, toe, scrub and roll-centre
%   migration come out of where the pickup points are rather than being
%   picked by a person. Those three assumptions collapse into one coordinate
%   set the suspension department can measure and replace from CAD.
%
%   THE LOOP IS THE POINT AND THE COST. chassis -> lower arm -> upright ->
%   upper arm -> chassis is a closed kinematic chain. Simscape Multibody
%   resolves it, which is the reason to use it instead of writing the
%   trigonometry -- but it makes the model a DAE that wants a stiff
%   variable-step solver. That is right for a design study and is NOT the
%   fixed-step plant. Whether any of this exports as an FMU is a separate
%   question and is not answered here.
%
%   WISHBONE AXES ARE NOT COORDINATE AXES. A Revolute Joint turns about its
%   own +Z, while a wishbone turns about the line joining its two chassis
%   pickups -- which points wherever the geometry says. So each arm gets a
%   rigid transform carrying a rotation matrix that maps +Z onto that line,
%   and the arm vector is then expressed in the rotated frame. Getting this
%   wrong does not fail to build; it silently swings the arm through the
%   wrong plane, which is why the axis is computed from the hardpoints rather
%   than assumed to be along x.

if nargin < 2 || isempty(mdl), mdl = 'IFSSIM_SM_Corner'; end
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl,'Model');
set_param(mdl,'SolverType','Variable-step','Solver','ode23t', ...
              'StartTime','0','StopTime','1','AbsTol','1e-8','RelTol','1e-6');

load_system('sm_lib'); load_system('nesl_utility');
L.solver = ['nesl_utility/Solver' newline 'Configuration'];
L.world  = ['sm_lib/Frames and' newline 'Transforms/World Frame'];
L.rt     = ['sm_lib/Frames and' newline 'Transforms/Rigid Transform'];
L.sensor = ['sm_lib/Frames and' newline 'Transforms/Transform Sensor'];
L.mech   = 'sm_lib/Utilities/Mechanism Configuration';
L.rev    = 'sm_lib/Joints/Revolute Joint';
L.sph    = 'sm_lib/Joints/Spherical Joint';
L.uni    = 'sm_lib/Joints/Universal Joint';
L.ine    = 'sm_lib/Body Elements/Inertia';

add_block(L.solver,[mdl '/Solver'],   'Position',[ 40  40 100  70]);
add_block(L.world, [mdl '/World'],    'Position',[ 40 140 100 170]);
add_block(L.mech,  [mdl '/Mechanism'],'Position',[ 40 240 100 270]);
add_line(mdl,'Solver/RConn1','World/RConn1','autorouting','on');
add_line(mdl,'Mechanism/RConn1','World/RConn1','autorouting','on');

% ---- the upright -------------------------------------------------------
% One rigid body carrying four frames. In Simscape Multibody a body is not a
% single block: it is an inertia plus rigid transforms branching off a common
% node, all rigidly connected. The node here is the WHEEL CENTRE, because
% that is the frame every measurement wants.
add_block(L.ine,[mdl '/Upright'],'Position',[620 300 690 350]);
% CUSTOM inertia, not PointMass. A point mass has NO moment of inertia, and
% every body in this linkage rotates -- the arms about their pivots, the
% upright about the kingpin. A rotating body with zero rotational inertia is
% a singular mass matrix, and Simscape reports that as "error evaluating
% equations at time 0.0 ... there may be a singularity", which reads like a
% tolerance problem and sends you to the solver settings instead of here.
set_param([mdl '/Upright'],'Mass','6','MassUnits','kg', ...   % ASSUMED upright+hub+wheel
    'InertiaType','Custom', ...
    'MomentsOfInertia','[0.12 0.12 0.10]','MomentsOfInertiaUnits','kg*m^2', ...
    'ProductsOfInertia','[0 0 0]','CenterOfMass','[0 0 0]');
arms = { 'up_to_lca', H.lca_outer - H.wheel_centre
         'up_to_uca', H.uca_outer - H.wheel_centre
         'up_to_tie', H.tie_outer - H.wheel_centre };
for k = 1:size(arms,1)
    b = [mdl '/' arms{k,1}];
    add_block(L.rt, b, 'Position',[500 200+90*k 560 250+90*k]);
    set_cart(b, arms{k,2});
    % Base = the upright node, follower = the attachment point, so the offset
    % reads wheel-centre -> ball exactly as written in the table above.
    % Wiring these the other way round leaves the transform's B port
    % unconnected -- Simscape says so and then carries on -- and silently
    % inverts every arm.
    add_line(mdl,'Upright/RConn1',[arms{k,1} '/LConn1'],'autorouting','on');
end

% ---- the two wishbones -------------------------------------------------
wb = { 'lca', H.lca_front, H.lca_rear, H.lca_outer
       'uca', H.uca_front, H.uca_rear, H.uca_outer };
for k = 1:size(wb,1)
    nm = wb{k,1};  pf = wb{k,2};  pr = wb{k,3};  po = wb{k,4};
    axis  = (pr - pf) / norm(pr - pf);          % the pivot line
    R     = frame_with_z(axis);                 % +Z of the joint onto it
    armLocal = (R' * (po - pf)')';              % arm vector in that frame

    bMount = [mdl '/mount_' nm];
    add_block(L.rt, bMount,'Position',[180 100+200*k 240 150+200*k]);
    set_param(bMount,'TranslationMethod','Cartesian', ...
        'TranslationCartesianOffset', vec2str(pf), 'TranslationCartesianOffsetUnits','m', ...
        'RotationMethod','RotationMatrix','RotationMatrix', mat2str(R,10));
    add_line(mdl,'World/RConn1',['mount_' nm '/LConn1'],'autorouting','on');

    bJoint = [mdl '/pivot_' nm];
    add_block(L.rev, bJoint,'Position',[300 100+200*k 360 150+200*k]);
    add_line(mdl,['mount_' nm '/RConn1'],['pivot_' nm '/LConn1'],'autorouting','on');

    bArm = [mdl '/arm_' nm];
    add_block(L.rt, bArm,'Position',[400 100+200*k 460 150+200*k]);
    set_cart(bArm, armLocal);
    add_line(mdl,['pivot_' nm '/RConn1'],['arm_' nm '/LConn1'],'autorouting','on');

    % THE ARM NEEDS MASS. A massless link inside a closed kinematic loop
    % gives a singular mass matrix, and the failure is not a warning about
    % geometry -- it is "error evaluating equations at time 0.0, there may be
    % a singularity", which reads like a solver tolerance problem and is not.
    % The value matters far less than its existence.
    bArmM = [mdl '/mass_' nm];
    add_block(L.ine, bArmM,'Position',[400 165+200*k 460 205+200*k]);
    set_param(bArmM,'Mass','1.2','MassUnits','kg','InertiaType','Custom', ...
        'MomentsOfInertia','[0.01 0.01 0.01]','MomentsOfInertiaUnits','kg*m^2', ...
        'ProductsOfInertia','[0 0 0]','CenterOfMass','[0 0 0]');
    add_line(mdl,['arm_' nm '/RConn1'],['mass_' nm '/RConn1'],'autorouting','on');

    bBall = [mdl '/ball_' nm];
    add_block(L.sph, bBall,'Position',[500 100+200*k 560 150+200*k]);
    add_line(mdl,['arm_' nm '/RConn1'],['ball_' nm '/LConn1'],'autorouting','on');
    add_line(mdl,['ball_' nm '/RConn1'],['up_to_' nm '/RConn1'],'autorouting','on');
end

% ---- the track rod -----------------------------------------------------
% Spherical inboard, UNIVERSAL outboard. Two spherical joints would leave the
% rod free to spin about its own axis -- a DOF that changes nothing and that
% the solver has to chase. The universal removes it.
add_block(L.rt,[mdl '/mount_tie'],'Position',[180 620 240 670]);
set_cart([mdl '/mount_tie'], H.tie_inner);
add_line(mdl,'World/RConn1','mount_tie/LConn1','autorouting','on');
add_block(L.sph,[mdl '/ball_tie_in'],'Position',[300 620 360 670]);
add_line(mdl,'mount_tie/RConn1','ball_tie_in/LConn1','autorouting','on');
add_block(L.rt,[mdl '/rod_tie'],'Position',[400 620 460 670]);
set_cart([mdl '/rod_tie'], H.tie_outer - H.tie_inner);
add_line(mdl,'ball_tie_in/RConn1','rod_tie/LConn1','autorouting','on');
add_block(L.ine,[mdl '/mass_tie'],'Position',[400 690 460 730]);
set_param([mdl '/mass_tie'],'Mass','0.4','MassUnits','kg','InertiaType','Custom', ...
    'MomentsOfInertia','[0.002 0.002 0.002]','MomentsOfInertiaUnits','kg*m^2', ...
    'ProductsOfInertia','[0 0 0]','CenterOfMass','[0 0 0]');
add_line(mdl,'rod_tie/RConn1','mass_tie/RConn1','autorouting','on');
add_block(L.uni,[mdl '/ball_tie_out'],'Position',[500 620 560 670]);
add_line(mdl,'rod_tie/RConn1','ball_tie_out/LConn1','autorouting','on');
add_line(mdl,'ball_tie_out/RConn1','up_to_tie/RConn1','autorouting','on');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));
fprintf('  linkage built: 2 wishbones, 3 ball joints, track rod, upright\n');
end

% -------------------------------------------------------------------------
function set_cart(blk, v)
set_param(blk,'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset', vec2str(v), 'TranslationCartesianOffsetUnits','m');
end

function s = vec2str(v)
s = sprintf('[%.8g %.8g %.8g]', v(1), v(2), v(3));
end

function R = frame_with_z(a)
%FRAME_WITH_Z  A rotation whose third column is A, built without a preferred
%   world axis that could be parallel to it. Picking "up" as the seed fails
%   for a vertical pivot; seeding from the smallest component of A cannot.
a = a(:) / norm(a);
[~, i] = min(abs(a));
seed = zeros(3,1); seed(i) = 1;
x = cross(seed, a);  x = x / norm(x);
y = cross(a, x);
R = [x, y, a];
end
