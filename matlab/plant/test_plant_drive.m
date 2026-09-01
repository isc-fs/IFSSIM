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

% No harness model. The plant is driven directly by plant_direct_sim, because
% a Model-block parent cannot be made to agree with the plant's fixed step
% once the chassis is continuous -- see the long note in plant_direct_sim.
STOP = 2.0;
cmdS = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');

ok = true;
fprintf('\n=== whole-car drive ===\n');

%% 1. Sitting still: the car must hold ride height and not wander.
r = drive(P, STOP, cmdS, 0,   0, 0); A = trace_(r);
ok = check(ok,'at rest: stays on the ground', abs(A.z(end) - P.CoGHeight) < 0.05, true);
ok = check(ok,'at rest: does not creep',      abs(A.vx(end)) < 0.05, true);
ok = check(ok,'at rest: nothing is NaN',      ~any(isnan(A.vx)), true);
fprintf('        ride height %.4f m (static %.4f)\n', A.z(end), P.CoGHeight);

%% 2. Throttle: the car accelerates forward, and the driven wheels turn.
r = drive(P, STOP, cmdS, 0.5, 0, 0); A = trace_(r); W = wheelTrace(r);
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
r = drive(P, STOP, cmdS, 0.5, 0, 0.5); A = trace_(r);
ok = check(ok,'steer left: positive yaw rate', A.wz(end) > 0.05, true);
ok = check(ok,'steer left: moves left (+y)',   A.y(end) > 0, true);
fprintf('        yaw rate %.3f rad/s, lateral %.3f m\n', A.wz(end), A.y(end));

r = drive(P, STOP, cmdS, 0.5, 0, -0.5); A = trace_(r);
ok = check(ok,'steer right: negative yaw rate', A.wz(end) < -0.05, true);
ok = check(ok,'steer right: moves right (-y)',  A.y(end) < 0, true);

%% 4. Regen slows the car down. It is the only service brake this car has.
r = drive(P, STOP, cmdS, 0,   1, 0); A = trace_(r);
% ONE-SIDED ASSERTIONS MISS HALF THE FAILURES. This read `vx <= 0.05`, which
% is true of -9.94 m/s, so it passed while the car reversed 17.8 m in 3 s
% under a regen command from a standstill. abs().
ok = check(ok,'regen from rest does not drive the car, either way', ...
           abs(A.vx(end)) <= 0.05, true);

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

function r = drive(P, stop, cmdS, thr, brk, str)
%DRIVE  One run at a fixed command. Field names come from IFSSIM_CmdBus.
cmdS.throttle = thr;  cmdS.regen = brk;  cmdS.steer_norm = str;
r = plant_direct_sim(P, stop, cmdS);
end

function A = trace_(r)
p = r.yout{1}.Values;                    % port 1 is Pose
pos = p.position.Data;  vb = p.vel_body.Data;  ob = p.omega_body.Data;
A.x = pos(:,1); A.y = pos(:,2); A.z = pos(:,3);
A.vx = vb(:,1); A.wz = ob(:,3);
end
function W = wheelTrace(r)
w = r.yout{2}.Values;                    % port 2 is Wheels
om = w.omega.Data;  W.omega_rl = om(:,3);
end

function ok = check(ok,name,got,want)
pass = isequal(logical(got),logical(want));
fprintf('  [%s] %s\n', ternary(pass,'ok  ','FAIL'), name);
if ~pass, ok = false; end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end

function s = plant_step()
%PLANT_STEP  The step IFSSIM_Plant actually NEGOTIATES, to full precision.
%
%   Not its declared FixedStep. Those are different numbers and that is the
%   whole difficulty: with every model declaring '1/960', the plant negotiates
%   0.0010416666666666667 while a harness declaring that same string
%   negotiates ...671. Simulink compares the NEGOTIATED rates, so a harness
%   that copies the declared string inherits the mismatch instead of avoiding
%   it -- which is what the first version of this function did.
%
%   Simulink.BlockDiagram.getSampleTimes is the API that reports them.
%   CompiledSampleTime is a BLOCK parameter and errors on a model.
load_system('IFSSIM_Plant');
ts = Simulink.BlockDiagram.getSampleTimes('IFSSIM_Plant');
p = [];
for k = 1:numel(ts)
    v = ts(k).Value;
    if numel(v) >= 1 && isfinite(v(1)) && v(1) > 0
        p(end+1) = v(1); %#ok<AGROW>
    end
end
if isempty(p)
    s = get_param('IFSSIM_Plant','FixedStep');   % nothing discrete to match
else
    s = sprintf('%.17g', min(p));
end
end
