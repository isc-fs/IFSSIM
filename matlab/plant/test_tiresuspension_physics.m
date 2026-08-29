function ok = test_tiresuspension_physics()
%TEST_TIRESUSPENSION_PHYSICS  Check the tyre/suspension block against closed form.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

h = 'tiresusp_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.5', ...
            'SaveFormat','Dataset','SignalLogging','on');

add_block('simulink/Ports & Subsystems/Model',[h '/TS'], ...
          'ModelNameDialog','IFSSIM_TireSuspension.slx','Position',[260 60 420 260]);

assignin('base','ROAD_FLAT', roadStruct(zeros(4,1), ones(4,1), P.TireMu*ones(4,1)));
assignin('base','POSE_REST', poseStruct(P.CoGHeight, [0;0;0], [0;0;0]));
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','ROAD_FLAT', ...
    'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 40 130 70]);
add_block('simulink/Sources/Constant',[h '/POSE'],'Value','POSE_REST', ...
    'OutDataTypeStr','Bus: IFSSIM_PoseBus','Position',[60 100 130 130]);
add_block('simulink/Sources/Constant',[h '/STEER'],'Value','[0;0;0;0]','Position',[60 160 130 190]);
add_block('simulink/Sources/Constant',[h '/DRV'],'Value','[0;0;0;0]','Position',[60 220 130 250]);
add_block('simulink/Sources/Constant',[h '/BRK'],'Value','[0;0;0;0]','Position',[60 280 130 310]);
add_line(h,'ROAD/1','TS/1','autorouting','on');
add_line(h,'POSE/1','TS/2','autorouting','on');
add_line(h,'STEER/1','TS/3','autorouting','on');
add_line(h,'DRV/1','TS/4','autorouting','on');
add_line(h,'BRK/1','TS/5','autorouting','on');

add_block('simulink/Sinks/To Workspace',[h '/w_out'],'VariableName','w_log', ...
    'SaveFormat','Timeseries','Position',[500 80 570 110]);
add_block('simulink/Sinks/To Workspace',[h '/f_out'],'VariableName','f_log', ...
    'SaveFormat','Timeseries','Position',[500 140 570 170]);
add_block('simulink/Sinks/To Workspace',[h '/m_out'],'VariableName','m_log', ...
    'SaveFormat','Timeseries','Position',[500 200 570 230]);
add_line(h,'TS/1','w_out/1','autorouting','on');
add_line(h,'TS/2','f_out/1','autorouting','on');
add_line(h,'TS/3','m_out/1','autorouting','on');

ok = true;
fprintf('\n=== tyre / suspension physics ===\n');
mg = P.Mass * 9.81;

%% 1. Static equilibrium at ride height.
r = sim(h);  W = wheels(r); F = force(r);
ok = check(ok,'static: total Fz = m*g', sum(W.fz), mg, 1e-6);
ok = check(ok,'static: front corner load', W.fz(1), P.Derived.StaticLoadFront, 1e-6);
ok = check(ok,'static: rear corner load',  W.fz(3), P.Derived.StaticLoadRear,  1e-6);
ok = check(ok,'static: no lateral force',  F(2), 0, 1e-9);
ok = check(ok,'static: all four in contact', sum(W.in_contact), 4, 0);

%% 2. A wheel over a hole cannot pull down on the road.
assignin('base','ROAD_HOLE', roadStruct([-1;0;0;0], ones(4,1), P.TireMu*ones(4,1)));
set_param([h '/ROAD'],'Value','ROAD_HOLE');
r = sim(h);  W = wheels(r);
ok = check(ok,'dropped wheel: Fz clamped to zero', W.fz(1), 0, 1e-12);
ok = check(ok,'dropped wheel: reported out of contact', W.in_contact(1), 0, 0);
ok = check(ok,'dropped wheel: others still loaded', W.fz(2) > 0, true, 0);
set_param([h '/ROAD'],'Value','ROAD_FLAT');

%% 2b. LOAD TRANSFER. Four corners, not two axles.
%
% This went untested for a long time, and it is the assumption every grip
% number downstream leans on. The suspension is what produces it: each
% corner's load follows its OWN deflection and deflection rate, taken at its
% own position through the body rotation, so pitching or rolling the body
% moves load between corners. There is no explicit transfer term anywhere --
% it falls out of the geometry, which is why it covers longitudinal and
% lateral transfer at once and why a two-axle algebraic formula is not needed.
%
% Pitch rate about y: the front corners move INTO the road while the rears
% move away from it, so the fronts must gain load and the rears must lose it,
% left and right staying equal because a pitch is symmetric.
%
% 0.1 rad/s deliberately, not something violent. At 0.5 the rear corner
% unloads completely and clamps at zero, and the assertion then passes on the
% lift clamp rather than on the transfer it is supposed to be testing.
assignin('base','POSE_PITCH', poseStruct(P.CoGHeight, [0;0;0], [0;0.1;0]));
set_param([h '/POSE'],'Value','POSE_PITCH');
r = sim(h);  Wp = wheels(r);
ok = check(ok,'pitch: front corners gain load', Wp.fz(1) > P.Derived.StaticLoadFront, true, 0);
ok = check(ok,'pitch: rear corners shed load',  Wp.fz(3) < P.Derived.StaticLoadRear,  true, 0);
ok = check(ok,'pitch: left and right stay equal', Wp.fz(1) - Wp.fz(2), 0, 1e-9);

% Roll rate about x: now it is left against right, and front against rear
% must stay put. y is POSITIVE LEFT, so a positive roll rate drives the LEFT
% corners down.
assignin('base','POSE_ROLL', poseStruct(P.CoGHeight, [0;0;0], [0.1;0;0]));
set_param([h '/POSE'],'Value','POSE_ROLL');
r = sim(h);  Wr = wheels(r);
ok = check(ok,'roll: left and right loads diverge', abs(Wr.fz(1) - Wr.fz(2)) > 1, true, 0);
ok = check(ok,'roll: front pair mirrors rear pair', ...
           sign(Wr.fz(1)-Wr.fz(2)), sign(Wr.fz(3)-Wr.fz(4)), 0);
fprintf('        pitch @0.1 rad/s: front %.0f N (static %.0f), rear %.0f N (static %.0f)\n', ...
        Wp.fz(1), P.Derived.StaticLoadFront, Wp.fz(3), P.Derived.StaticLoadRear);
fprintf('        roll  @0.1 rad/s: left %.0f N, right %.0f N\n', Wr.fz(1), Wr.fz(2));
set_param([h '/POSE'],'Value','POSE_REST');

%% 2c. THE TEST THAT WAS MISSING: does a TYRE FORCE produce a body moment?
%
% The two checks above impose a body RATE and assert the dampers respond.
% That is the wrong direction of causality, and it is why they passed for
% months on a plant with no load transfer at all.
%
% The FIRST version of this check was no better, in a way worth recording. It
% computed cross(r_b, F) in the test itself, with its own copy of the plant's
% r_b commented "as the plant builds it", and never simulated anything. It
% could not have caught the bug it was written for: it would have stayed red
% however the plant was fixed, and gone green the moment somebody edited the
% test's own copy of the arm. A test that mirrors the code tests nothing.
%
% These RUN the model and compare the wrench it reports against its own
% reported per-wheel forces, placed at the contact patch by Newton. The arm
% is the only thing under test, so the size of the term it contributes is
% asserted separately -- with a zero z arm both checks fail on that clause
% alone, which is what makes this a real regression guard.
rx4 = [ P.Derived.aFront;  P.Derived.aFront; -P.Derived.bRear; -P.Derived.bRear];
ry4 = [ P.TrackFront/2;   -P.TrackFront/2;    P.TrackRear/2;   -P.TrackRear/2];
harm = P.CoGHeight;

% Longitudinal: wheels locked at 10 m/s, so the tyres drag hard rearward.
assignin('base','POSE_BRK', poseStruct(P.CoGHeight, [10;0;0], [0;0;0]));
set_param([h '/POSE'],'Value','POSE_BRK');
r = sim(h);  Wb = wheels(r);  Mb = torque(r);
My_vert = -sum(rx4 .* Wb.fz);            % pitch from the vertical loads alone
My_arm  = -harm * sum(Wb.fx);            % the term the ground arm contributes
ok = check(ok,'pitch moment matches the contact-patch wrench', ...
           Mb(2), My_vert + My_arm, 1e-6*max(1,abs(My_vert+My_arm)));
ok = check(ok,'and the ground arm is what carries it', ...
           abs(My_arm) > 0.05*abs(My_vert), true, 0);
fprintf('        Fx %+.0f N -> pitch: model %+.1f N.m = vertical %+.1f + arm %+.1f\n', ...
        sum(Wb.fx), Mb(2), My_vert, My_arm);

% Lateral: sliding left at 10 m/s, wheels free, so the tyres push right.
assignin('base','POSE_LAT', poseStruct(P.CoGHeight, [10;2;0], [0;0;0]));
set_param([h '/POSE'],'Value','POSE_LAT');
r = sim(h);  Wl = wheels(r);  Ml = torque(r);
Mx_vert = sum(ry4 .* Wl.fz);
Mx_arm  = harm * sum(Wl.fy);
ok = check(ok,'roll moment matches the contact-patch wrench', ...
           Ml(1), Mx_vert + Mx_arm, 1e-6*max(1,abs(Mx_vert+Mx_arm)));
ok = check(ok,'and the ground arm is what carries it', ...
           abs(Mx_arm) > 0.05*max(abs(Mx_vert),1), true, 0);
fprintf('        Fy %+.0f N -> roll : model %+.1f N.m = vertical %+.1f + arm %+.1f\n', ...
        sum(Wl.fy), Ml(1), Mx_vert, Mx_arm);
set_param([h '/POSE'],'Value','POSE_REST');

%% 3. Lateral force opposes lateral slip, and saturates at mu*Fz.
% Sliding LEFT (+vy in ISO 8855) must produce force to the RIGHT.
assignin('base','POSE_SLIP', poseStruct(P.CoGHeight, [10;1;0], [0;0;0]));
set_param([h '/POSE'],'Value','POSE_SLIP');
r = sim(h);  W = wheels(r); F = force(r);
ok = check(ok,'slip left -> force right (Fy < 0)', F(2) < 0, true, 0);
alpha = atan2(1, 10);
ok = check(ok,'slip angle matches atan2(vy,vx)', W.slip_angle(1), alpha, 1e-6);

% Sliding hard, wheels not driven. Two invariants worth asserting, and one
% naive expectation that is WRONG and worth recording as a comment so nobody
% re-adds it:
%
%   The obvious test — "at a big slip angle lateral force approaches mu*m*g" —
%   fails, correctly. With omega = 0 and the body at 10 m/s the wheels are
%   LOCKED, so slip ratio is -1 and the friction budget is spent
%   longitudinally, not laterally. And the Magic Formula falls off past its
%   peak, so a large slip angle gives LESS lateral force, not more. Both
%   behaviours are right; the expectation was not.
%
%   Do not re-express these checks in terms of a peak slip angle. That number
%   is a property of the coefficients in settings.json, and it has already
%   moved once — 4.9 deg to 10.6 deg — when the curve was found to be far too
%   peaky for a slick. Assert invariants, not operating points.
assignin('base','POSE_BIG', poseStruct(P.CoGHeight, [10;10;0], [0;0;0]));
set_param([h '/POSE'],'Value','POSE_BIG');
r = sim(h);  W = wheels(r); F = force(r);

% The invariant that actually matters: one tyre, one friction budget. The
% resultant horizontal force per wheel may never exceed mu*Fz.
res = hypot(W.fx, W.fy);
cap = P.TireMu * W.fz + 1e-6;
ok = check(ok,'friction ellipse holds per wheel', all(res <= cap), true, 0);

% The wheels SPIN UP. Started from rest with the ground moving at 10 m/s, the
% longitudinal tyre force accelerates each wheel until the slip ratio vanishes
% and it is simply rolling. That is exactly the behaviour Chaos cannot produce
% — it snaps wheel speed to ground speed, so the wheel never has a dynamic
% state to spin up. Here it is integrated from torque, so it does.
% ...but NOT to zero slip, and not at this pose. The body is at [10;10;0] --
% a 45 degree slip angle -- and a Magic Formula tyre generates a longitudinal
% force at a slip angle even when the longitudinal slip is zero. An undriven
% wheel therefore settles where that induced force is cancelled, which is at
% kappa ~ -0.2, not at zero.
%
% This is the combined-slip coupling the tyre was swapped in for. The friction
% ellipse it replaced could only ever SCALE Fx and Fy, so Fx was identically
% zero whenever kappa was, and the wheel settled at exactly vx/Rw. Asserting
% that here would be asserting the old model's simplification.
ok = check(ok,'undriven wheels spin up from rest', W.omega(1) > 0.5*10/P.WheelRadius, true, 0);
fprintf('        at 45 deg slip: kappa settles at %+.4f, omega %.1f rad/s\n', W.slip_ratio(1), W.omega(1));
% Threshold -0.01, not -0.05. The original -0.05 was picked when this settled
% at -0.208, and that figure was inflated by a friction-vs-slip-speed decay
% inherited from the passenger-car tyre set: at a 45 degree slip angle the
% contact patch slides at 10 m/s, where the old reference velocity cost 40% of
% the grip. With that disabled the coupling settles at about -0.028 -- still
% forty times the straight-ahead residual of -0.0007, so the effect is real
% and this still tests it, but the old bound was measuring the artifact.
ok = check(ok,'slip angle induces longitudinal slip', W.slip_ratio(1) < -0.01, true, 0);
% Once rolling, the friction budget is available laterally again.
ok = check(ok,'rolling at slip angle: |Fy| > |Fx|', abs(F(2)) > abs(F(1)), true, 0);

% The strict version of the spin-up check, at the pose where it is actually
% exact: straight ahead, no slip angle, nothing to couple into. Here an
% undriven wheel must come to true free rolling, and the speed it settles at
% is the one the REST OF THE STACK depends on -- the pipeline converts motor
% rpm to road speed with this radius, so a tyre quietly rolling on a different
% one puts a bias into /odom that no odometry test would attribute to a tyre.
assignin('base','POSE_STRAIGHT', poseStruct(P.CoGHeight, [10;0;0], [0;0;0]));
set_param([h '/POSE'],'Value','POSE_STRAIGHT');
r = sim(h);  W = wheels(r);
ok = check(ok,'straight: free wheels spin up until slip -> 0', W.slip_ratio(1), 0, 1e-3);
ok = check(ok,'straight: spun-up speed matches vx/Rw', W.omega(1), 10/P.WheelRadius, 0.5);
set_param([h '/POSE'],'Value','POSE_REST');

%% 4. Wheel spin: airborne, so no tyre force resists the torque.
T = 21;
assignin('base','ROAD_AIR', roadStruct(zeros(4,1), zeros(4,1), P.TireMu*ones(4,1)));
set_param([h '/ROAD'],'Value','ROAD_AIR');
set_param([h '/DRV'],'Value',sprintf('[0;0;%g;%g]',T,T));
r = sim(h);  W = wheels(r);
ok = check(ok,'airborne wheel spin-up = T/Iw*t', W.omega(3), T/P.Assumed.WheelInertia*0.5, 0.5);
ok = check(ok,'undriven wheel stays still', W.omega(1), 0, 1e-12);

%% 5. Contact is gated on the platform having CHARACTERISED the ground.
%
% A negative residual is the platform saying it could not fit a plane at all —
% fewer than three of its probe rays hit — so it does not know what is under
% this wheel. Standing on an answer nobody has is how a wheel ends up loaded
% against geometry that is not there.
%
% Both halves are checked. Without the second, a gate that rejected EVERY wheel
% would pass the first and quietly ground the car.
set_param([h '/DRV'],'Value','[0;0;0;0]');
set_param([h '/POSE'],'Value','POSE_REST');

% FL's fit failed; the other three are fine.
assignin('base','ROAD_NOFIT', ...
    roadStruct(zeros(4,1), ones(4,1), P.TireMu*ones(4,1), [-1;0;0;0]));
set_param([h '/ROAD'],'Value','ROAD_NOFIT');
r = sim(h);  W = wheels(r);
ok = check(ok,'no plane fitted: that wheel carries no load', W.fz(1), 0, 1e-9);
ok = check(ok,'no plane fitted: the others still do',        W.fz(2) > 0, true, 0);

% And a LARGE but MEASURED residual must NOT remove contact. The patch spans a
% kerb, the plane describes it badly, but the tyre is still touching it —
% dropping Fz here would make the car fall through every kerb.
assignin('base','ROAD_ROUGH', ...
    roadStruct(zeros(4,1), ones(4,1), P.TireMu*ones(4,1), [0.05;0.05;0.05;0.05]));
set_param([h '/ROAD'],'Value','ROAD_ROUGH');
r = sim(h);  W = wheels(r);
ok = check(ok,'rough but measured: contact is KEPT', W.fz(1) > 0, true, 0);

set_param([h '/ROAD'],'Value','ROAD_FLAT');

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'tyre/suspension checks PASS.','TYRE/SUSPENSION CHECKS FAILED.'));
end

function s = roadStruct(hgt, valid, mu, res)
s = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid = valid; s.height = hgt; s.mu = mu;
s.normal_x = zeros(4,1); s.normal_y = zeros(4,1); s.normal_z = ones(4,1);
if nargin < 4, res = zeros(4,1); end
s.residual = res;
end

function s = poseStruct(z, velb, omegab)
s = Simulink.Bus.createMATLABStruct('IFSSIM_PoseBus');
s.position=[0;0;z]; s.quat=[1;0;0;0];
s.vel_world=[0;0;0]; s.vel_body=velb; s.omega_body=omegab;
s.alpha_body=[0;0;0]; s.accel_proper=[0;0;0]; s.attitude=[0;0;0];
end

function W = wheels(r)
s = r.get('w_log'); W = struct();
for n = {'omega','steer','fz','fx','fy','slip_ratio','slip_angle','susp_travel','in_contact'}
    d = s.(n{1}).Data;  W.(n{1}) = double(reshape(d(end,:),[],1));
end
end

function f = force(r)
d = r.get('f_log').Data;  f = double(reshape(d(end,:),[],1));
end

function m = torque(r)
d = r.get('m_log').Data;  m = double(reshape(d(end,:),[],1));
end

function ok = check(ok,name,got,want,tol)
if islogical(want) || islogical(got)
    pass = isequal(logical(got),logical(want));
    fprintf('  [%s] %-42s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass = all(abs(got-want) <= tol);
    fprintf('  [%s] %-42s got %+.6g  want %+.6g\n', ternary(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end

function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end
