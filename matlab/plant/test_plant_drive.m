function ok = test_plant_drive()
%TEST_PLANT_DRIVE  Drive the WHOLE car. The end-to-end check.
%
%   Every other test exercises one subsystem in isolation. This one runs the
%   assembled plant: commands in, trajectory out. It is the test that catches
%   wiring mistakes, frame disagreements between subsystems, and sign errors
%   that only show up when the loop is closed.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

h = 'plant_drive_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','2.0','SaveFormat','Dataset');

add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);

% Flat road under all four wheels. The real platform ray-casts terrain; here a
% constant plane is enough to see whether the car drives.
assignin('base','ROAD_D', roadFlat(P));
assignin('base','ENV_D',  envFlat());
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','CMD_D', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','ROAD_D', ...
          'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 120 130 150]);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','ENV_D', ...
          'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 180 130 210]);
add_line(h,'CMD/1','Plant/1','autorouting','on');
add_line(h,'ROAD/1','Plant/2','autorouting','on');
add_line(h,'ENV/1','Plant/3','autorouting','on');

add_block('simulink/Sinks/To Workspace',[h '/pose_out'],'VariableName','pose_d', ...
          'SaveFormat','Timeseries','Position',[500 70 570 100]);
add_block('simulink/Sinks/To Workspace',[h '/wheel_out'],'VariableName','wheel_d', ...
          'SaveFormat','Timeseries','Position',[500 130 570 160]);
add_line(h,'Plant/1','pose_out/1','autorouting','on');
add_line(h,'Plant/2','wheel_out/1','autorouting','on');

ok = true;
fprintf('\n=== whole-car drive ===\n');

%% 1. Sitting still: the car must hold ride height and not wander.
cmd(0,0,0); r = sim(h); A = trace_(r);
ok = check(ok,'at rest: stays on the ground', abs(A.z(end) - P.CoGHeight) < 0.05, true);
ok = check(ok,'at rest: does not creep',      abs(A.vx(end)) < 0.05, true);
ok = check(ok,'at rest: nothing is NaN',      ~any(isnan(A.vx)), true);
fprintf('        ride height %.4f m (static %.4f)\n', A.z(end), P.CoGHeight);

%% 2. Throttle: the car accelerates forward, and the driven wheels turn.
cmd(0.5,0,0); r = sim(h); A = trace_(r); W = wheelTrace(r);
ok = check(ok,'throttle: accelerates forward', A.vx(end) > 1.0, true);
ok = check(ok,'throttle: travels forward',     A.x(end) > 0.5, true);
ok = check(ok,'throttle: rear wheels spinning', W.omega_rl(end) > 1.0, true);
ok = check(ok,'throttle: still on the ground', abs(A.z(end)-P.CoGHeight) < 0.1, true);
% NO SPURIOUS WHEELSPIN. At half throttle the drive torque is well inside the
% grip limit, so the wheels must roll, not spin. This assertion exists because
% an earlier version passed every other check while the wheels turned 12x
% faster than the road — a numerical instability, not physics.
rolling = A.vx(end) / P.WheelRadius;
ok = check(ok,'throttle: wheels roll, not spin', W.omega_rl(end) < 1.5*rolling, true);
fprintf('        wheel %.1f rad/s vs rolling %.1f rad/s (slip %.3f)\n', ...
        W.omega_rl(end), rolling, (W.omega_rl(end)*P.WheelRadius - A.vx(end))/max(A.vx(end),1));
fprintf('        after 2 s: vx %.2f m/s, travelled %.2f m, rear wheel %.1f rad/s\n', ...
        A.vx(end), A.x(end), W.omega_rl(end));

%% 3. Throttle and steer: the car yaws, and it yaws the RIGHT WAY.
% ISO 8855: positive steering command is LEFT, and left is positive yaw rate.
cmd(0.5,0,0.5); r = sim(h); A = trace_(r);
ok = check(ok,'steer left: positive yaw rate', A.wz(end) > 0.05, true);
ok = check(ok,'steer left: moves left (+y)',   A.y(end) > 0, true);
fprintf('        yaw rate %.3f rad/s, lateral %.3f m\n', A.wz(end), A.y(end));

cmd(0.5,0,-0.5); r = sim(h); A = trace_(r);
ok = check(ok,'steer right: negative yaw rate', A.wz(end) < -0.05, true);
ok = check(ok,'steer right: moves right (-y)',  A.y(end) < 0, true);

%% 4. Regen slows the car down. It is the only service brake this car has.
cmd(0,1,0); r = sim(h); A = trace_(r);
ok = check(ok,'regen from rest does not drive the car', A.vx(end) <= 0.05, true);

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'whole-car drive PASS.','WHOLE-CAR DRIVE FAILED.'));

    function cmd(th,rg,st)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=th; c.regen=rg; c.steer_norm=st; c.ebs_latch=0; c.handbrake=0;
        assignin('base','CMD_D',c);
    end
end

function s = roadFlat(P)
s = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
end
function s = envFlat()
s = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
s.gravity_z=-9.81; s.ext_force=[0;0;0]; s.ext_torque=[0;0;0]; s.ext_point=[0;0;0];
s.chassis_grounded=0;
end

function A = trace_(r)
p = r.get('pose_d');
pos = p.position.Data;  vb = p.vel_body.Data;  ob = p.omega_body.Data;
A.x = pos(:,1); A.y = pos(:,2); A.z = pos(:,3);
A.vx = vb(:,1); A.wz = ob(:,3);
end
function W = wheelTrace(r)
w = r.get('wheel_d'); om = w.omega.Data;  W.omega_rl = om(:,3);
end

function ok = check(ok,name,got,want)
pass = isequal(logical(got),logical(want));
fprintf('  [%s] %s\n', ternary(pass,'ok  ','FAIL'), name);
if ~pass, ok = false; end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
