function build_chassis(outdir, useVDB)
%BUILD_CHASSIS  Fill in IFSSIM_Chassis: 6-DOF rigid-body dynamics.
%
%   6-DOF rigid body. Everything else feeds it forces; this integrates them.
%
%     forces --> [Rigid Body Update] --> next state --> [Unit Delay] --+
%                                    \--> Pose bus out                 |
%                        state ---------------------------------------+
%
%   State is in Unit Delays so it is visible on the canvas. The update reads
%   LAST step's state, so there is no algebraic loop by construction.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
if nargin < 2 || isempty(useVDB), useVDB = false; end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);

P = ifssim_load_workspace();

name = 'IFSSIM_Chassis';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

%% ---- inputs -----------------------------------------------------------
ins = {'tyre_force',3; 'tyre_torque',3; 'aero_force',3; 'aero_torque',3};
y = 40;
for i = 1:size(ins,1)
    b = [name '/' ins{i,1}];
    add_block('simulink/Sources/In1', b, 'Position',[30 y 60 y+20], ...
              'PortDimensions',num2str(ins{i,2}));
    y = y + 60;
end
add_block('simulink/Sources/In1',[name '/Env'],'Position',[30 y 60 y+20], ...
          'OutDataTypeStr','Bus: IFSSIM_EnvBus','BusOutputAsStruct','on');

% Split Env here rather than inside the function. The selector documents which
% fields the chassis actually uses, and it keeps struct typing out of a block
% that only wants numbers.
add_block('simulink/Signal Routing/Bus Selector',[name '/Env Select'], ...
          'Position',[140 y-20 150 y+80], ...
          'OutputSignals','gravity_z,ext_force,ext_torque');
add_line(name,'Env/1','Env Select/1','autorouting','on');

y = y + 60;
add_block('simulink/Sources/In1',[name '/Sync'],'Position',[30 y 60 y+20], ...
          'OutDataTypeStr','Bus: IFSSIM_SyncBus','BusOutputAsStruct','on');

% Same treatment for Sync. Splitting it here keeps the state-injection fields
% visible on the canvas: someone opening this model can see that the platform
% is able to write pose, which a struct passed whole into a function would hide.
add_block('simulink/Signal Routing/Bus Selector',[name '/Sync Select'], ...
          'Position',[140 y+120 150 y+230], ...
          'OutputSignals','enable,pos,quat,vel_body,omega_body');
add_line(name,'Sync/1','Sync Select/1','autorouting','on');

if useVDB
    build_chassis_vdb(name, f, P);
    return
end

%% ---- the update function ---------------------------------------------
fcn = [name '/Rigid Body Update'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[260 40 460 320]);

code = chassis_code();
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = code;

% EXPLICIT PORT SIZES. Without these the model does not compile, and the error
% is not obvious: the function cannot infer its input sizes because they come
% from the Unit Delays, whose sizes come from the function's own outputs. That
% circle has no fixed point until someone states a dimension.
%
% Declaring every port also makes the block self-documenting — the contract is
% visible in the Ports and Data Manager, not only in the signature.
sizes = struct( ...
    'tyre_f',3,      'tyre_t',3,     'aero_f',3,       'aero_t',3, ...
    'gravity_z',1,   'ext_force',3,  'ext_torque',3, ...
    'pos_i',3,       'quat_i',4,     'velb_i',3,       'omega_i',3, ...
    'sync_en',1,     'sync_pos',3,   'sync_quat',4,    'sync_velb',3, 'sync_omega',3, ...
    'pos_n',3,       'quat_n',4,     'velb_n',3,       'omega_n',3, ...
    'position',3,    'quat',4,       'vel_world',3,    'vel_body',3, ...
    'omega_body',3,  'alpha_body',3, 'accel_proper',3, 'attitude',3);

data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes, d.Name)
        d.Props.Array.Size = num2str(sizes.(d.Name));
    end
end

% PARAMETERS must be DECLARED, not merely present in the base workspace.
% A MATLAB Function block does not pick up workspace variables the way a plain
% script does — an undeclared IFSSIM_Mass is simply an unknown identifier, and
% the resulting compile error says only "errors in the block body", which sends
% you looking at the maths instead of at the data dictionary.
%
% Declared this way, Simulink resolves each one from the base workspace at
% compile time, where ifssim_load_workspace() has put it — from settings.json.
params = {'IFSSIM_Ts','IFSSIM_Mass','IFSSIM_Ixx','IFSSIM_Iyy','IFSSIM_Izz','IFSSIM_CoGH'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing, params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name  = params{k};
    d.Scope = 'Parameter';
    d.Props.Array.Size = '1';
end

%% ---- state memory -----------------------------------------------------
% Initial conditions: at rest, level, at the origin. The platform teleports the
% car to the start gate by writing pose, so a non-zero IC here would just be a
% number to forget to update.
% Start AT RIDE HEIGHT, not at the world origin. With pos = [0;0;0] the corner
% attachments sit level with the road, so the suspension reads 0.3 m of
% compression on the first step and fires the car into the air. The platform
% teleports the car to the start gate anyway, but a plant that cannot be started
% from its own defaults is a plant nobody can test in isolation.
states = {'pos',      '[0;0;IFSSIM_CoGH]'
          'quat',     '[1;0;0;0]'
          'vel_body', '[0;0;0]'
          'omega',    '[0;0;0]'};
y = 400;
for i = 1:size(states,1)
    b = [name '/' states{i,1} ' (state)'];
    add_block('simulink/Discrete/Unit Delay', b, ...
              'Position',[300 y 360 y+30], 'InitialCondition',states{i,2}, ...
              'SampleTime','-1');
    y = y + 60;
end

%% ---- outputs ----------------------------------------------------------
add_block('simulink/Signal Routing/Bus Creator',[name '/Pose Bus'], ...
          'Position',[560 40 570 320], 'Inputs','8', ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus', ...
          'NonVirtualBus','on');
add_block('simulink/Sinks/Out1',[name '/Pose'],'Position',[640 170 670 190], ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus');
add_line(name,'Pose Bus/1','Pose/1','autorouting','on');

%% ---- wiring -----------------------------------------------------------
% Function port order follows the script signature exactly. Getting this wrong
% is silent — every signal is a double vector, so a swapped pair type-checks
% perfectly and simply produces a different car.
FB = 'Rigid Body Update';

% inputs 1..5: tyre_f, tyre_t, aero_f, aero_t, env
srcIn = {'tyre_force','tyre_torque','aero_force','aero_torque'};
for i = 1:numel(srcIn)
    add_line(name, [srcIn{i} '/1'], sprintf('%s/%d', FB, i), 'autorouting','on');
end
% inputs 5..7 come off the Env selector, in its declared output order.
for i = 1:3
    add_line(name, sprintf('Env Select/%d', i), sprintf('%s/%d', FB, 4+i), 'autorouting','on');
end

% inputs 8..11 are LAST step's state, read back from the delays. This is what
% breaks the algebraic loop: the update never reads its own current output.
stateNames = {'pos','quat','vel_body','omega'};
for i = 1:numel(stateNames)
    add_line(name, [stateNames{i} ' (state)/1'], sprintf('%s/%d', FB, 7+i), 'autorouting','on');
end

% inputs 12..16 come off the Sync selector, in its declared output order.
for i = 1:5
    add_line(name, sprintf('Sync Select/%d', i), sprintf('%s/%d', FB, 11+i), 'autorouting','on');
end

% outputs 1..4 are the NEXT state, fed into the delays.
for i = 1:numel(stateNames)
    add_line(name, sprintf('%s/%d', FB, i), [stateNames{i} ' (state)/1'], 'autorouting','on');
end

% outputs 5..12 are the reported state, in IFSSIM_PoseBus element order:
% position, quat, vel_world, vel_body, omega_body, alpha_body, accel_proper, attitude
for i = 1:8
    add_line(name, sprintf('%s/%d', FB, 4+i), sprintf('Pose Bus/%d', i), 'autorouting','on');
end

save_system(name, f);
fprintf('wrote %s\n', f);
fprintf('  mass %.0f kg, inertia [%.0f %.0f %.0f] kg m^2 (ASSUMED)\n', ...
        P.Mass, P.Assumed.Ixx, P.Assumed.Iyy, P.Assumed.Izz);
close_system(name,0);
end

%% =======================================================================
function c = chassis_code()
c = sprintf('%s\n', ...
"function [pos_n, quat_n, velb_n, omega_n, position, quat, vel_world, vel_body, omega_body, alpha_body, accel_proper, attitude] = ...", ...
"         chassis(tyre_f, tyre_t, aero_f, aero_t, gravity_z, ext_force, ext_torque, pos_i, quat_i, velb_i, omega_i, ...", ...
"                 sync_en, sync_pos, sync_quat, sync_velb, sync_omega)", ...
"%#codegen", ...
"% 6-DOF rigid body, Newton-Euler in the body frame, forward Euler at Ts.", ...
"%", ...
"% Frames: body is ISO 8855 / REP-103 — x forward, y LEFT, z up. World is ENU.", ...
"% Wheel order elsewhere is FL, FR, RL, RR.", ...
"%", ...
"% tyre_f / aero_f arrive in the BODY frame (they are generated by body-attached", ...
"% things). ext_force is WORLD — it comes from the platform's collision solver,", ...
"% which has no opinion about the car's orientation.", ...
"%", ...
"% The Env bus is split by a Bus Selector OUTSIDE this block rather than passed", ...
"% in whole. Two reasons: a MATLAB Function block has to infer a struct type", ...
"% through the port, which is fragile; and the selector makes the diagram show", ...
"% exactly which Env fields the chassis consumes.", ...
"", ...
"% Flat workspace scalars, not IFSSIM_P.Assumed.Izz: a MATLAB Function block", ...
"% resolves workspace variables as parameters and that lookup does not reach", ...
"% into nested structs. See ifssim_load_workspace.m — still one source, still", ...
"% settings.json.", ...
"Ts = IFSSIM_Ts;", ...
"m  = IFSSIM_Mass;", ...
"I  = [IFSSIM_Ixx; IFSSIM_Iyy; IFSSIM_Izz];   % principal, diagonal", ...
"", ...
"% --- state injection -------------------------------------------------", ...
"% Applied to the INCOMING state, before any dynamics. Overriding the outgoing", ...
"% state instead would look equivalent and is not: this step would still", ...
"% compute its forces and its reported acceleration from the OLD pose, so the", ...
"% caller would read a response to a state the plant is no longer in. For a", ...
"% one-step comparison against a reference that is precisely the wrong answer,", ...
"% and it is wrong by one step, which is the hardest kind of wrong to see.", ...
"%", ...
"% Normal running leaves enable at zero and nothing here executes.", ...
"if sync_en > 0.5", ...
"    pos_i   = sync_pos;", ...
"    quat_i  = sync_quat / max(norm(sync_quat), eps);", ...
"    velb_i  = sync_velb;", ...
"    omega_i = sync_omega;", ...
"end", ...
"", ...
"% --- orientation ---------------------------------------------------", ...
"q = quat_i / max(norm(quat_i), eps);        % renormalise every step: forward Euler", ...
"                                        % on a quaternion drifts off the unit", ...
"                                        % sphere, and the drift is silent.", ...
"R = quat2rotm_local(q);                 % body -> world", ...
"", ...
"% --- forces ----------------------------------------------------------", ...
"% Everything to the body frame first, then one sum. Mixing frames in a", ...
"% force accumulator is the classic way to get a plausible-but-wrong plant.", ...
"F_body = tyre_f + aero_f + R' * ext_force;", ...
"M_body = tyre_t + aero_t + R' * ext_torque;", ...
"", ...
"% PROPER acceleration excludes gravity — it is what an accelerometer reads.", ...
"% Publishing this directly is the point: the simulator currently finite-", ...
"% differences a world velocity and bolts +g on, which is where the missing", ...
"% Coriolis terms in the EKF came from.", ...
"accel_proper = F_body / m;", ...
"", ...
"% Gravity acts on the body but is NOT felt by an accelerometer, so it enters", ...
"% the equation of motion after accel_o has been taken.", ...
"g_body = R' * [0; 0; gravity_z];", ...
"", ...
"% --- rigid body ------------------------------------------------------", ...
"% v_dot = F/m + g - omega x v      (Coriolis term: body-frame rates are not", ...
"%                                   inertial, and dropping this is exactly the", ...
"%                                   bug that made vy drift during cornering)", ...
"vdot  = accel_proper + g_body - cross(omega_i, velb_i);", ...
"", ...
"% I*omega_dot = M - omega x (I*omega)   (gyroscopic coupling)", ...
"Iw    = I .* omega_i;", ...
"alpha_body = (M_body - cross(omega_i, Iw)) ./ I;", ...
"", ...
"% --- integrate -------------------------------------------------------", ...
"velb_n  = velb_i + Ts * vdot;", ...
"omega_n = omega_i + Ts * alpha_body;", ...
"", ...
"% Position integrates the WORLD velocity.", ...
"vel_world = R * velb_i;", ...
"pos_n   = pos_i + Ts * vel_world;", ...
"", ...
"% q_dot = 0.5 * q (x) [0; omega]", ...
"qd      = 0.5 * quatmul_local(q, [0; omega_i]);", ...
"quat_n  = q + Ts * qd;", ...
"quat_n  = quat_n / max(norm(quat_n), eps);", ...
"", ...
"% --- reported state ---------------------------------------------------", ...
"position = pos_i;", ...
"quat    = q;", ...
"vel_body = velb_i;", ...
"omega_body = omega_i;", ...
"", ...
"% attitude = [roll; pitch; heave]. Heave is height above the declared static", ...
"% CoG height, so 0 means sitting at ride height rather than at the world origin.", ...
"roll_  = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));", ...
"sinp   = 2*(q(1)*q(3) - q(4)*q(2));", ...
"pitch_ = asin(max(-1, min(1, sinp)));", ...
"attitude = [roll_; pitch_; pos_i(3) - IFSSIM_CoGH];", ...
"end", ...
"", ...
"function R = quat2rotm_local(q)", ...
"%#codegen", ...
"w = q(1); x = q(2); y = q(3); z = q(4);", ...
"R = [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w); ...", ...
"       2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w); ...", ...
"       2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)];", ...
"end", ...
"", ...
"function r = quatmul_local(a, b)", ...
"%#codegen", ...
"r = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4); ...", ...
"     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3); ...", ...
"     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2); ...", ...
"     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];", ...
"end");
end

% =========================================================================
function build_chassis_vdb(name, f, P)
%BUILD_CHASSIS_VDB  Step 4b: MathWorks' Vehicle Body 6DOF as the rigid body.
%
%   Same six inputs and the same IFSSIM_PoseBus out, so nothing downstream
%   knows. What changes is who integrates: the forked Vehicle Body 6DOF
%   instead of our own MATLAB Function.
%
%   Four things had to be measured first, and each would have failed
%   quietly. See docs/vdb_step4b_body_semantics.md.
%
%   1. THE FRAME IS z-DOWN and gravity is applied INSIDE the block at a
%      fixed 9.81. Our world is ENU with a configurable Env.gravity_z. Both
%      are handled explicitly below; vdb_body_frame owns the conversion.
%
%   2. Acc.ax/ay/az IS NOT PROPER ACCELERATION and is not in m/s^2 -- it is
%      kinematic acceleration in g. accel_proper is computed, not read.
%
%   3. THE BLOCK HAS ITS OWN AERODYNAMICS, Cd = 0.3 and Af = 2 by default,
%      which would silently double our drag. Both are ZEROED here. Aero
%      stays in IFSSIM_Aero where it is parameterised from the car and
%      tested; the body block is a rigid body and nothing else.
%
%   4. ITS DEFAULTS ARE A PASSENGER CAR -- 2000 kg, Iveh diag(430,1900,2100),
%      1.4/1.6 m axles, 1.9 m track. Every one is overwritten from car_spec
%      below. This is the same failure the tyre paramset had, and it does
%      not announce itself: it just understeers.

set_param(name,'SolverType','Fixed-step','Solver','ode1','FixedStep',ifssim_step(true));
% ode1, NOT FixedStepDiscrete. The body block integrates continuously; a
% discrete solver leaves its five integrators with no rate to run at.
%
% The step comes from ifssim_step(true), which is the VARIANT's step and not
% the default one. They are different doubles -- 2 ulp apart -- and which one
% makes the build work depends on what the plant negotiates, which in turn
% depends on whether the chassis is continuous. Hand-writing either literal
% here is how four of the ten stages broke; ifssim_step owns the choice and
% explains it.

fork_vehicle_body(name, [name '/Body'], [420 60 580 260]);
set_param([name '/Body'], ...
    'm',    sprintf('%.10g', P.Mass), ...
    'Iveh', sprintf('[%.10g 0 0; 0 %.10g 0; 0 0 %.10g]', ...
                    P.Assumed.Ixx, P.Assumed.Iyy, P.Assumed.Izz), ...
    'a',    sprintf('%.10g', P.Derived.aFront), ...
    'b',    sprintf('%.10g', P.Derived.bRear), ...
    'h',    sprintf('%.10g', P.CoGHeight), ...
    'w',    sprintf('[%.10g %.10g]', P.TrackFront, P.TrackRear), ...
    'Cd',   '0', ...        % see note 3 -- aero lives in IFSSIM_Aero
    'Af',   '0');
% Xe_o / xbdot_o / eul_o / p_o are NOT set here, and writing them would be
% worse than useless. fork_vehicle_body has just put every integrator on
% InitialConditionSource='external', at which point Simulink ignores the
% dialog initial condition entirely -- so those four values would read as
% configuration that determines the starting state while determining nothing.
%
% The initial state comes from the external IC ports, i.e. from the Sync bus
% via body_inputs, on the first step. That means the platform's sync.pos is
% the car's starting pose whether or not sync.enable is set, so it must carry
% a sensible ride height rather than zeros.

% FSusp/MSusp stay zero: every force this chassis receives already arrives
% summed at the body origin from tiresusp_post, so it goes in through
% FExt/MExt as the migration plan specifies. Feeding the double-wishbone
% block's per-wheel VehF/VehM into FSusp instead would let the BODY do the
% moment arithmetic, which is a further step and not this one.
for z = {'FSusp','MSusp'}
    add_block('simulink/Sources/Constant',[name '/' z{1} '_zero'], ...
              'Value','zeros(3,4)','Position',[330 60+40*(z{1}(1)=='M') 390 76+40*(z{1}(1)=='M')]);
end
add_block('simulink/Sources/Constant',[name '/Wind_zero'],'Value','[0;0;0]', ...
          'Position',[330 150 390 166]);
add_line(name,'FSusp_zero/1','Body/1','autorouting','on');
add_line(name,'MSusp_zero/1','Body/2','autorouting','on');
add_line(name,'Wind_zero/1','Body/5','autorouting','on');

%% ---- inputs into the block's frame ------------------------------------
IN = [name '/Body Inputs'];
add_block('simulink/User-Defined Functions/MATLAB Function', IN, ...
          'Position',[240 200 380 420]);
S = sfroot;  ci = S.find('-isa','Stateflow.EMChart','Path',IN);
ci.Script = body_inputs_code();
set_chart_sizes(ci, struct('tyre_f',3,'tyre_t',3,'aero_f',3,'aero_t',3, ...
    'gravity_z',1,'ext_force',3,'ext_torque',3, ...
    'sync_en',1,'sync_pos',3,'sync_quat',4,'sync_velb',3,'sync_omega',3,'dcm',[3 3], ...
    'FExt',3,'MExt',3,'trig',1,'ic_euler',3,'ic_pqr',3,'ic_vb',3,'ic_xe',3,'ic_acc',1));
declare_params(ci, {'IFSSIM_Mass'});

order = {'tyre_force','tyre_torque','aero_force','aero_torque'};
for i = 1:4, add_line(name,[order{i} '/1'],sprintf('Body Inputs/%d',i),'autorouting','on'); end
for i = 1:3, add_line(name,sprintf('Env Select/%d',i),sprintf('Body Inputs/%d',4+i),'autorouting','on'); end
for i = 1:5, add_line(name,sprintf('Sync Select/%d',i),sprintf('Body Inputs/%d',7+i),'autorouting','on'); end
add_line(name,'Body Inputs/1','Body/3','autorouting','on');   % FExt
add_line(name,'Body Inputs/2','Body/4','autorouting','on');   % MExt
gts = {'trig','IFSSIM_SYNC_TRIG'; 'ic_euler','IFSSIM_SYNC_EULER'; 'ic_pqr','IFSSIM_SYNC_PQR'
       'ic_vb','IFSSIM_SYNC_VB'; 'ic_xe','IFSSIM_SYNC_XE'; 'ic_acc','IFSSIM_SYNC_ACC'};
for k = 1:size(gts,1)
    add_line(name,sprintf('Body Inputs/%d',2+k), ...
             [matlab.lang.makeValidName(gts{k,2}) '_goto/1'],'autorouting','on');
end

%% ---- outputs back into IFSSIM_PoseBus ---------------------------------
add_block('simulink/Signal Routing/Bus Selector',[name '/Body Info'], ...
    'OutputSignals',['BdyFrm.Cg.AngAcc.pdot,BdyFrm.Cg.AngAcc.qdot,BdyFrm.Cg.AngAcc.rdot,' ...
                     'BdyFrm.Cg.Acc.xddot,BdyFrm.Cg.Acc.yddot,BdyFrm.Cg.Acc.zddot'], ...
    'Position',[620 60 630 200]);
add_line(name,'Body/1','Body Info/1','autorouting','on');

OUT = [name '/Pose Repack'];
add_block('simulink/User-Defined Functions/MATLAB Function', OUT, ...
          'Position',[700 60 840 360]);
co = S.find('-isa','Stateflow.EMChart','Path',OUT);
co.Script = pose_repack_code();
declare_params(co, {'IFSSIM_CoGH'});   % heave is measured from ride height
set_chart_sizes(co, struct('Vb',3,'pqr',3,'eul',3,'Xe',3,'Ve',3, ...
    'pdot',1,'qdot',1,'rdot',1,'xddot',1,'yddot',1,'zddot',1, ...
    'position',3,'quat',4,'vel_world',3,'vel_body',3,'omega_body',3, ...
    'alpha_body',3,'accel_proper',3,'attitude',3));

bodyOut = {2,'Vb'; 3,'pqr'; 5,'eul'; 6,'Xe'; 7,'Ve'};
for k = 1:size(bodyOut,1)
    add_line(name,sprintf('Body/%d',bodyOut{k,1}),sprintf('Pose Repack/%d',k),'autorouting','on');
end
for k = 1:6
    add_line(name,sprintf('Body Info/%d',k),sprintf('Pose Repack/%d',5+k),'autorouting','on');
end
% The body's DCM, one step late, so the world-frame external wrench can be
% rotated into the body frame. MEMORY, not Unit Delay: a Unit Delay declares a
% discrete rate and would make this model a hybrid, which is the trap the
% tyre/suspension model already documents at length.
add_block('simulink/Discrete/Memory',[name '/DCM (delay)'], ...
          'Position',[620 300 690 330],'InitialCondition','eye(3)');
add_line(name,'Body/4','DCM (delay)/1','autorouting','on');
add_line(name,'DCM (delay)/1',sprintf('Body Inputs/%d',13),'autorouting','on');

add_block('simulink/Signal Routing/Bus Creator',[name '/Pose Bus'], ...
          'Position',[880 60 890 340],'Inputs','8', ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus','NonVirtualBus','on');
add_block('simulink/Sinks/Out1',[name '/Pose'],'Position',[940 190 970 210], ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus');
for i = 1:8
    add_line(name,sprintf('Pose Repack/%d',i),sprintf('Pose Bus/%d',i),'autorouting','on');
end
add_line(name,'Pose Bus/1','Pose/1','autorouting','on');

save_system(name, f);
fprintf('wrote %s  (VDB Vehicle Body 6DOF)\n', f);
fprintf('  mass %.0f kg, inertia [%.0f %.0f %.0f] kg m^2, block aero DISABLED\n', ...
        P.Mass, P.Assumed.Ixx, P.Assumed.Iyy, P.Assumed.Izz);
close_system(name,0);
end

% -------------------------------------------------------------------------
function set_chart_sizes(chart, sizes)
d = chart.find('-isa','Stateflow.Data');
for k = 1:numel(d)
    if isfield(sizes, d(k).Name)
        d(k).Props.Array.Size = num2str(sizes.(d(k).Name));
    end
end
end

function declare_params(chart, params)
d = chart.find('-isa','Stateflow.Data');
have = {d.Name};
for k = 1:numel(params)
    if any(strcmp(have, params{k})), continue; end
    n = Stateflow.Data(chart);
    n.Name = params{k};  n.Scope = 'Parameter';  n.Props.Array.Size = '1';
end
end

% -------------------------------------------------------------------------
function c = body_inputs_code()
c = char(strjoin(string({
"function [FExt, MExt, trig, ic_euler, ic_pqr, ic_vb, ic_xe, ic_acc] = ..."
"         body_inputs(tyre_f, tyre_t, aero_f, aero_t, gravity_z, ext_force, ..."
"                     ext_torque, sync_en, sync_pos, sync_quat, sync_velb, sync_omega, dcm)"
"%#codegen"
"% Everything this chassis is pushed with, gathered and put into the body"
"% block's frame; plus the state injection, likewise converted."
""
"% ---- the applied wrench ----------------------------------------------"
"% THESE TERMS ARE NOT ALL IN THE SAME FRAME, and treating them as if they"
"% were is the bug this replaced. tyre_f and aero_f are OUR BODY frame."
"% ext_force is OUR WORLD frame -- IFSSIM_EnvBus says so, and it is where"
"% the platform's collision solver works. The incumbent chassis rotates it,"
"% R' * ext_force; the first version of this function simply added it, which"
"% applies a world-frame contact force along whatever direction the car"
"% happens to be pointing. A car hit from the side while facing north gets"
"% shoved along its own axis instead."
"%"
"% So each term is converted from the frame it is actually in:"
"%   body terms      one y/z flip into the block's body frame;"
"%   world terms     the same flip into the block's WORLD frame, then dcm,"
"%                   which was MEASURED to be world->body (a pure 30 deg yaw"
"%                   reproduces Rz(psi)' to 1.7e-12, not Rz(psi))."
"%"
"% dcm arrives one step late, through a Memory block. That matches the"
"% incumbent, which rotates by R built from LAST step's quaternion, and it"
"% is what keeps this out of an algebraic loop."
"F_body = [tyre_f(1) + aero_f(1); -(tyre_f(2) + aero_f(2)); -(tyre_f(3) + aero_f(3))];"
"M_body = [tyre_t(1) + aero_t(1); -(tyre_t(2) + aero_t(2)); -(tyre_t(3) + aero_t(3))];"
"F_ext_blk = dcm * [ext_force(1);  -ext_force(2);  -ext_force(3)];"
"M_ext_blk = dcm * [ext_torque(1); -ext_torque(2); -ext_torque(3)];"
""
"% GRAVITY. The block applies 9.81 m/s^2 internally along its +z, which is"
"% DOWN, and exposes no parameter for it. Env.gravity_z is ours and is"
"% configurable, so what goes in here is only the DIFFERENCE. With the"
"% default -9.81 this term is exactly zero and costs nothing; set lunar"
"% gravity and the car gets lighter instead of being quietly ignored."
"% abs() would DISCARD THE SIGN of a signed input. gravity_z is negative in"
"% our ENU world (down), so the magnitude is -gravity_z; taking abs() happens"
"% to agree for the usual case and silently ignores the sign for any other,"
"% which is the same one-sided-comparison mistake as reading vx <= 0.05 on a"
"% signed speed."
"g_block = 9.81;"
"g_extra = IFSSIM_Mass * ((-gravity_z) - g_block);"
""
"FExt = F_body + F_ext_blk + [0; 0; g_extra];"
"MExt = M_body + M_ext_blk;"
""
"% ---- state injection --------------------------------------------------"
"% The forked integrators reset on a RISING edge, so this passes the"
"% platform's enable through unchanged and lets the edge do the work."
"trig = sync_en;"
""
"% Quaternion -> Euler, then into the block's frame. Our quat is [w x y z],"
"% body->world, ENU. The frames differ by 180 degrees about x, which for a"
"% quaternion means negating the y and z parts of the vector -- and for the"
"% Euler triple, negating pitch and yaw."
"q = sync_quat;"
"n = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4));"
"if n < 1e-9"
"    q = [1; 0; 0; 0];"
"else"
"    q = q / n;"
"end"
"roll  = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)*q(2) + q(3)*q(3)));"
"sp    = 2*(q(1)*q(3) - q(4)*q(2));"
"if sp >  1, sp =  1; end"
"if sp < -1, sp = -1; end"
"pitch = asin(sp);"
"yaw   = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)*q(3) + q(4)*q(4)));"
"ic_euler = [roll; -pitch; -yaw];"
""
"ic_pqr = [sync_omega(1); -sync_omega(2); -sync_omega(3)];"
"ic_vb  = [sync_velb(1);  -sync_velb(2);  -sync_velb(3)];"
"ic_xe  = [sync_pos(1);   -sync_pos(2);   -sync_pos(3)];"
""
"% The SignalCollection accumulator is not a pose state -- it is a running"
"% total the block keeps. Teleporting the car does not make its history"
"% meaningful, so it is zeroed with the rest rather than left to carry"
"% distance across a jump it never travelled."
"ic_acc = 0;"
"end"
}), newline));
end

% -------------------------------------------------------------------------
function c = pose_repack_code()
c = char(strjoin(string({
"function [position, quat, vel_world, vel_body, omega_body, alpha_body, ..."
"          accel_proper, attitude] = ..."
"         pose_repack(Vb, pqr, eul, Xe, Ve, pdot, qdot, rdot, xddot, yddot, zddot)"
"%#codegen"
"% The body block's outputs, back into IFSSIM_PoseBus. Frames converted by"
"% negating y and z -- the block is x-forward/y-right/z-DOWN, ours is ENU."
""
"position   = [Xe(1);  -Xe(2);  -Xe(3)];"
"vel_world  = [Ve(1);  -Ve(2);  -Ve(3)];"
"vel_body   = [Vb(1);  -Vb(2);  -Vb(3)];"
"omega_body = [pqr(1); -pqr(2); -pqr(3)];"
"alpha_body = [pdot;   -qdot;   -rdot];"
""
"% Euler -> quaternion. Pitch and yaw are negated on the way out, the"
"% mirror of what body_inputs does on the way in."
"r2 = eul(1)/2;  p2 = -eul(2)/2;  y2 = -eul(3)/2;"
"cr = cos(r2); sr = sin(r2); cp = cos(p2); sp = sin(p2); cy = cos(y2); sy = sin(y2);"
"quat = [cr*cp*cy + sr*sp*sy;"
"        sr*cp*cy - cr*sp*sy;"
"        cr*sp*cy + sr*cp*sy;"
"        cr*cp*sy - sr*sp*cy];"
""
"% PROPER ACCELERATION -- computed, never read off the block."
"%"
"% Acc.ax/ay/az looks like the field for this and is NOT: measured, it"
"% reads 1.000 in free fall where an accelerometer reads zero, and 1.020"
"% where xddot reads 10.000. It is KINEMATIC acceleration expressed in g."
"% Wiring it through would publish an IMU 9.81 times too small and still"
"% carrying gravity. This plant already has one IMU scaling bug of that"
"% family; it does not need a second."
"%"
"% What an accelerometer reads is the kinematic acceleration MINUS gravity,"
"% resolved in the body frame. In the block's z-down world gravity is"
"% +9.81 along z, so its body-frame component is the third column of the"
"% body->world rotation, which for the ZYX Euler set is:"
"sp2 = sin(eul(2)); cp2 = cos(eul(2)); sr2 = sin(eul(1)); cr2 = cos(eul(1));"
"g_body = 9.81 * [-sp2; sr2*cp2; cr2*cp2];"
"a_blk  = [xddot; yddot; zddot] - g_body;"
"accel_proper = [a_blk(1); -a_blk(2); -a_blk(3)];"
""
"% attitude is [roll, pitch, heave]. Heave takes the same z flip as position"
"% -- the block counts down, we count up -- AND the same DATUM: it is height"
"% above the static CoG height, so 0 means sitting at ride height rather than"
"% at the world origin. The incumbent subtracts IFSSIM_CoGH (see the note on"
"% attitude in chassis_code); dropping it here made attitude(3) bit-identical"
"% to position(3), which is what gave it away."
"attitude = [eul(1); -eul(2); -Xe(3) - IFSSIM_CoGH];"
"end"
}), newline));
end
