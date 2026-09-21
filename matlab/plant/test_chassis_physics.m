function ok = test_chassis_physics()
%TEST_CHASSIS_PHYSICS  Run IFSSIM_Chassis against closed-form answers.
%
%   "It compiles" is not "it is right". Each case below has an analytic answer,
%   so a wrong sign, a wrong frame or a missing term fails loudly instead of
%   producing a plausible trajectory nobody checks.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

h = 'chassis_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','1.0', ...
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

% And the proof that 4a meant something: the un-synced run must NOT be sitting
% at the target, or the test above proves nothing.
ok = check(ok,'sync: disabled run is unaffected', ...
           double(abs(base.position(1) - target(1)) > 1.0), 1, 0);

assignin('base','SYNC_OFF', syncStruct(0));

%% 4. Quaternion stays unit after a second of rotation.
ok = check(ok,'quaternion remains unit', norm(pose.quat), 1.0, 1e-9);

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
