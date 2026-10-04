function ok = test_chassis_physics()
%TEST_CHASSIS_PHYSICS  Run IFSSIM_Chassis against closed-form answers.
%
%   "It compiles" is not "it is right". Each case below has an analytic answer,
%   so a wrong sign, a wrong frame or a missing term fails loudly instead of
%   producing a plausible trajectory nobody checks.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(ifssim_models_dir());
P = ifssim_load_workspace();

h = 'chassis_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
% ode1, not FixedStepDiscrete. A discrete solver cannot simulate a model
% containing continuous states, and whether IFSSIM_Chassis has any depends on
% which variant was built -- the VDB Vehicle Body 6DOF integrates
% continuously. ode1 is forward Euler at the same fixed step, so anything
% already discrete steps exactly as it did before; this widens what the
% harness can run rather than changing how it runs it.
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep',chassis_step(),'StartTime','0','StopTime','1.0', ...
            'SaveFormat','Dataset','SignalLogging','on');

add_block('simulink/Ports & Subsystems/Model',[h '/Chassis'], ...
          'ModelNameDialog','IFSSIM_Chassis.slx','Position',[240 60 400 220]);

% Four force/torque inputs plus the Env bus.
srcs = {'tyre_force','TF','[0;0;0]'; 'tyre_torque','TT','[0;0;0]'; ...
        'aero_force','AF','[0;0;0]'; 'aero_torque','AT','[0;0;0]'};
y = 40;
for i = 1:size(srcs,1)
    add_block('simulink/Sources/Constant',[h '/' srcs{i,2}], ...
        'Value',srcs{i,3},'Position',[60 y 130 y+30]);
    add_line(h,[srcs{i,2} '/1'],sprintf('Chassis/%d',i),'autorouting','on');
    y = y + 60;
end
assignin('base','ENV_ZERO', envStruct(-9.81));
add_block('simulink/Sources/Constant',[h '/ENV'], ...
    'Value','ENV_ZERO','OutDataTypeStr','Bus: IFSSIM_EnvBus', ...
    'Position',[60 y 130 y+30]);
add_line(h,'ENV/1','Chassis/5','autorouting','on');

assignin('base','SYNC_OFF', syncStruct(0));
add_block('simulink/Sources/Constant',[h '/SYNC'], ...
    'Value','SYNC_OFF','OutDataTypeStr','Bus: IFSSIM_SyncBus', ...
    'Position',[60 y+60 130 y+90]);
add_line(h,'SYNC/1','Chassis/6','autorouting','on');

add_block('simulink/Sinks/To Workspace',[h '/pose_out'], ...
    'VariableName','pose_log','SaveFormat','Timeseries', ...
    'Position',[470 130 540 160]);
add_line(h,'Chassis/1','pose_out/1','autorouting','on');

ok = true;
fprintf('\n=== chassis physics ===\n');

%% 1. Free fall.
set_param([h '/TF'],'Value','[0;0;0]');
r = sim(h);
pose = last_pose(r);
g = -9.81; T = 1.0;
ok = check(ok,'free-fall vertical velocity', pose.vel_world(3), g*T, 0.05);
% An accelerometer in free fall reads ZERO. This is the single most commonly
% wrong signal in a vehicle sim, and it is the one the EKF consumes.
ok = check(ok,'free-fall proper acceleration is ZERO', norm(pose.accel_proper), 0, 1e-9);

%% 2. Body-x force, held level by an equal and opposite z force.
Fx = 1000;
set_param([h '/TF'],'Value',sprintf('[%g;0;%g]', Fx, -9.81*P.Mass*-1));
r = sim(h);
pose = last_pose(r);
ok = check(ok,'longitudinal velocity from F=ma', pose.vel_body(1), Fx/P.Mass*T, 0.05);
ok = check(ok,'proper acceleration reads F/m in x', pose.accel_proper(1), Fx/P.Mass, 1e-6);

%% 3. Yaw torque -> omega_z = Mz/Izz * t
Mz = 50;
set_param([h '/TF'],'Value','[0;0;0]');
set_param([h '/TT'],'Value',sprintf('[0;0;%g]',Mz));
r = sim(h);
pose = last_pose(r);
ok = check(ok,'yaw rate from M=I*alpha', pose.omega_body(3), Mz/P.Assumed.Izz*T, 0.02);

%% 4. State injection.
%
% The platform can WRITE the plant's state. Two things depend on it and both
% fail silently if it quietly does nothing: resetting an FMU to an arbitrary
% start gate, and comparing this plant against a reference from an identical
% state. A no-op injection looks exactly like a plant that agrees.
set_param([h '/TT'],'Value','[0;0;0]');

% 4a. enable = 0 must change NOTHING. Tested first, because an injection that
% always fires would pass every test below and break every normal run.
assignin('base','SYNC_OFF', syncStruct(0));
r = sim(h);
base = last_pose(r);

% 4b. enable = 1 places the body somewhere it could not have fallen to.
target = [12.0; -3.0; 7.5];
tvel   = [4.0; 0.5; 0.0];
sy = syncStruct(1); sy.pos = target; sy.vel_body = tvel;
assignin('base','SYNC_OFF', sy);
r = sim(h);
pose = last_pose(r);

% Held for the whole run, so the body starts each step from the injected state
% and only integrates one step's worth of gravity away from it. The position
% check is loose in z for exactly that reason; x and y have nothing acting on
% them and must land on the target.
ok = check(ok,'sync: x is written',  pose.position(1), target(1), 1e-6);
ok = check(ok,'sync: y is written',  pose.position(2), target(2), 1e-6);
ok = check(ok,'sync: velocity is written', pose.vel_body(1), tvel(1), 1e-6);
% The LATERAL component too. Only x was asserted before, so the y negation in
% the VDB chassis' frame conversion (ENU body -> the block's y-right frame)
% could be dropped and this case would still pass.
ok = check(ok,'sync: lateral velocity is written', pose.vel_body(2), tvel(2), 1e-6);

% And the proof that 4a meant something: the un-synced run must NOT be sitting
% at the target, or the test above proves nothing.
ok = check(ok,'sync: disabled run is unaffected', ...
           double(abs(base.position(1) - target(1)) > 1.0), 1, 0);

assignin('base','SYNC_OFF', syncStruct(0));

%% 4. Quaternion stays unit after a second of rotation.
ok = check(ok,'quaternion remains unit', norm(pose.quat), 1.0, 1e-9);

%% 5. External wrench is WORLD frame, so it must be rotated into the body.
%
% IFSSIM_EnvBus declares ext_force and ext_torque world frame -- they come
% from the platform's collision solver. The VDB chassis once applied them
% unrotated and nothing caught it, because every test ran at identity
% orientation, where a rotated and an unrotated vector are the same vector.
%
% So the car is yawed 90 deg first. Body +x then points along world +y, and
% the two interpretations come apart completely:
%   rotated correctly   -> a world +x push moves the car along world +x
%   applied unrotated   -> it acts along body +x and moves the car along world +y
% There is no tolerance in which one passes for the other.
[okF, dx, dy, wx, wy] = yawed_push(P);
a  = 1.0;                         % m/s^2, the push is m*a
tf = 1.0 - 0.05;                  % free time after the hold releases
ok = check(ok,'ext_force: moves along world x when yawed 90deg', dx, 0.5*a*tf^2, 0.01);
ok = check(ok,'ext_force: does not move along world y',           dy, 0,          0.005);
% A world-x torque on a car yawed 90 deg is a BODY -y torque, i.e. pitch.
% Applied unrotated it would be body +x, i.e. roll, at roughly 3.7x the rate
% because Ixx is a quarter of Iyy -- again no overlap between the outcomes.
My = 11.0;
ok = check(ok,'ext_torque: pitches (body -y) when yawed 90deg', wy, -My/P.Assumed.Iyy*tf, 0.005);
ok = check(ok,'ext_torque: does not roll (body x)',              wx, 0,                    0.003);
if ~okF, ok = false; end

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'chassis physics checks PASS.','CHASSIS PHYSICS CHECKS FAILED.'));
end

function e = envStruct(gz)
e = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
e.gravity_z = gz;
e.ext_force = [0;0;0]; e.ext_torque = [0;0;0]; e.ext_point = [0;0;0];
e.chassis_grounded = 0;
end

function p = last_pose(r)
% A non-virtual bus logged as Timeseries comes back as a struct of timeseries,
% one field per bus element — the element NAMES, which is exactly why the bus
% object was worth defining.
s = r.get('pose_log');
names = {'position','quat','vel_world','vel_body','omega_body','alpha_body','accel_proper','attitude'};
p = struct();
for i = 1:numel(names)
    ts = s.(names{i});
    d  = ts.Data;
    p.(names{i}) = double(reshape(d(end,:), [], 1));
end
end

function s = syncStruct(en)
s = Simulink.Bus.createMATLABStruct('IFSSIM_SyncBus');
s.enable = en; s.pos = [0;0;0]; s.quat = [1;0;0;0];
s.vel_body = [0;0;0]; s.omega_body = [0;0;0];
end

function ok = check(ok, name, got, want, tol)
d = abs(got - want);
pass = all(d <= tol);
if ~pass, ok = false; end
fprintf('  [%s] %-40s got %+.6g  want %+.6g\n', ternary(pass,'ok  ','FAIL'), name, got, want);
end

function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end

function s = chassis_step()
%CHASSIS_STEP  Whatever step IFSSIM_Chassis was actually built with.
%
%   Read off the model rather than assumed, because the two variants use
%   different steps and this harness has no way of knowing which one was
%   built. Hardcoding either makes the test pass for one variant and fail
%   for the other with a message about sample times that points nowhere near
%   the harness.
load_system('IFSSIM_Chassis');
s = get_param('IFSSIM_Chassis','FixedStep');
end

% -------------------------------------------------------------------------
function [ok, dx, dy, wx, wy] = yawed_push(P)
%YAWED_PUSH  Hold the car yawed 90 deg, release it, push it with a world wrench.
%
%   Needs a sync bus that CHANGES during the run -- held for the first 50 ms to
%   place the car, then released so it can move -- which the main harness's
%   Constant cannot do. Hence its own small harness.
ok = true;
h = 'chassis_extforce_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep',chassis_step(),'StartTime','0','StopTime','1.0', ...
            'SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Chassis'], ...
          'ModelNameDialog','IFSSIM_Chassis.slx','Position',[420 60 580 260]);

for k = 1:4
    add_block('simulink/Sources/Constant',[h '/z' num2str(k)],'Value','[0;0;0]', ...
              'Position',[60 20+40*k 120 40+40*k]);
    add_line(h,['z' num2str(k) '/1'],sprintf('Chassis/%d',k),'autorouting','on');
end

env = envStruct(-9.81);
env.ext_force  = [P.Mass * 1.0; 0; 0];     % world +x, 1 m/s^2 worth
env.ext_torque = [11.0; 0; 0];             % world +x
assignin('base','ENV_PUSH', env);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','ENV_PUSH', ...
          'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 220 120 250]);
add_line(h,'ENV/1','Chassis/5','autorouting','on');

% Sync: enable high for 50 ms, then released.
psi = pi/2;
src = { 'enable',     'step'
        'pos',        '[0;0;0]'
        'quat',       sprintf('[%.17g;0;0;%.17g]', cos(psi/2), sin(psi/2))
        'vel_body',   '[0;0;0]'
        'omega_body', '[0;0;0]' };
add_block('simulink/Signal Routing/Bus Creator',[h '/SyncB'], ...
          'Inputs','5','OutDataTypeStr','Bus: IFSSIM_SyncBus','NonVirtualBus','on', ...
          'Position',[300 280 310 420]);
for k = 1:size(src,1)
    b = [h '/s_' src{k,1}];
    if strcmp(src{k,2},'step')
        add_block('simulink/Sources/Step', b, 'Time','0.05','Before','1','After','0', ...
                  'Position',[160 260+30*k 200 280+30*k]);
    else
        add_block('simulink/Sources/Constant', b, 'Value', src{k,2}, ...
                  'Position',[160 260+30*k 200 280+30*k]);
    end
    % Bus Creator matches by SIGNAL NAME, so the line carries the element name.
    lh = add_line(h,['s_' src{k,1} '/1'],sprintf('SyncB/%d',k),'autorouting','on');
    set_param(lh,'Name',src{k,1});
end
add_line(h,'SyncB/1','Chassis/6','autorouting','on');

add_block('simulink/Signal Routing/Bus Selector',[h '/sel'], ...
          'OutputSignals','position,omega_body','Position',[620 120 630 180]);
add_line(h,'Chassis/1','sel/1','autorouting','on');
names = {'P_pos','P_om'};
for k = 1:2
    add_block('simulink/Sinks/To Workspace',[h '/w' num2str(k)],'VariableName',names{k}, ...
              'SaveFormat','Timeseries','Position',[680 90+50*k 740 120+50*k]);
    add_line(h,sprintf('sel/%d',k),['w' num2str(k) '/1'],'autorouting','on');
end

try
    r = sim(h);
    pos = squeeze(r.get('P_pos').Data);  if size(pos,1) == 3, pos = pos'; end
    om  = squeeze(r.get('P_om').Data);   if size(om,1)  == 3, om  = om';  end
    dx = pos(end,1);  dy = pos(end,2);
    wx = om(end,1);   wy = om(end,2);
catch ME
    fprintf('  [FAIL] ext_force harness: %s\n', ME.message);
    ok = false; dx = NaN; dy = NaN; wx = NaN; wy = NaN;
end
close_system(h,0);
end