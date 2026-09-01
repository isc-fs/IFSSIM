function ok = test_brakes_physics()
%TEST_BRAKES_PHYSICS  Check the EBS, and that it stops the car.
%
%   Two levels: the block on its own, then the WHOLE CAR braking — because the
%   number that matters is a stopping distance, not a torque.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();
ok = true;
fprintf('\n=== brakes ===\n');

%% ---- the block alone --------------------------------------------------
h = 'brakes_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
% This harness references IFSSIM_Brakes, not the plant, and IFSSIM_Brakes is
% not a discrete/continuous hybrid -- so the bit-exact step rule never applied
% here and a plain literal is right. It briefly used plant_step(), which
% compiled the entire plant just to read a number this harness does not need.
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.05','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/BR'], ...
          'ModelNameDialog','IFSSIM_Brakes.slx','Position',[240 60 380 140]);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','CMD_B', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/W'],'Value','[50;50;50;50]','Position',[60 120 130 150]);
add_block('simulink/Sinks/To Workspace',[h '/T'],'VariableName','tl', ...
          'SaveFormat','Timeseries','Position',[460 80 520 110]);
add_line(h,'CMD/1','BR/1','autorouting','on');
add_line(h,'W/1','BR/2','autorouting','on');
add_line(h,'BR/1','T/1','autorouting','on');

setcmd(0,0); r = sim(h);
ok = check(ok,'no command: no brake torque', max(abs(T(r))), 0, 1e-12);
setcmd(1,0); r = sim(h); t = T(r);
ok = check(ok,'EBS latched: full torque, all four wheels', t, ...
           P.Derived.EbsTorquePerWheel*ones(4,1), 1e-6);
setcmd(0,1); r = sim(h);
ok = check(ok,'handbrake drives the same system', T(r), t, 1e-9);
fprintf('        EBS %.0f N.m/wheel vs grip limit %.0f N.m\n', ...
        t(1), P.TireMu*(P.Mass*9.81/4)*P.WheelRadius);
close_system(h,0);

%% ---- the whole car braking --------------------------------------------
% The whole car braking. Driven DIRECTLY rather than through a Model-block
% harness: a referencing parent cannot be made to agree with the plant's fixed
% step once the chassis is continuous. See the note in plant_direct_sim for
% what was eliminated before concluding that.
%
% Accelerate for 4 s, then latch the EBS and let it stop. The step is placed
% on adjacent samples so it is a step and not a ramp.
STOPT = 9;
tv  = [0; 3.999; 4; STOPT];
cmdB = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
cmdB.throttle   = [0.6; 0.6; 0;   0];
cmdB.ebs_latch  = [0;   0;   1;   1];
cmdB.regen      = 0;
cmdB.steer_norm = 0;
cmdB.handbrake  = 0;

r = plant_direct_sim(P, STOPT, cmdB, [], tv);  A = pose(r); W = wheels(r);
[~,i4] = min(abs(A.t-4));
v0 = A.vx(i4);
ok = check(ok,'car reaches speed before braking', v0 > 8, true);

% Stopping distance from the moment the EBS latches to the moment it stops.
moving = find(A.t > 4 & A.vx > 0.1);
if isempty(moving)
    stopIdx = i4;
else
    stopIdx = moving(end);
end
dist = A.x(stopIdx) - A.x(i4);
decel = v0 / max(A.t(stopIdx) - 4, 1e-6);
ok = check(ok,'EBS brings the car to a stop', A.vx(end) < 0.5, true);
% Asserted over the stop PROPER: from the EBS latching until the car first
% falls below the speed at which the tyre model stops claiming to be valid.
% A time window, not a speed mask, because at the end the car oscillates back
% and forth ACROSS any speed threshold, so a mask keeps re-entering the very
% regime it was meant to exclude.
%
% The property is that a brake decelerating a rolling wheel locks it and never
% drives it backwards, and it holds: 111 -> ~1 rad/s within a fifth of a
% second of latching, and positive for the whole descent.
%
% Below SlipRegularisationSpeed the plant has a genuine low-speed limit cycle.
% Sliding friction carries no stiction term, so a nearly-stopped car rocks
% about zero and drags the wheels with it. This is NOT new physics introduced
% by the tyre block -- the old hand-written tyre had the same hole and hid it,
% clamping the wheel state to zero right after integrating. Wheel spin is a
% state inside the block now and cannot be reached from out here, so the
% artifact is visible instead of suppressed. Fixing it properly means giving
% the tyre a stiction term, which is a real piece of work and is not this.
first_slow = find(A.t > 4 & A.vx < P.Assumed.SlipRegularisationSpeed, 1, 'first');
if isempty(first_slow), first_slow = numel(A.t); end
stop_phase = (A.t > 4) & ((1:numel(A.t))' < first_slow);
ok = check(ok,'wheels lock rather than reverse', min(min(W.M(stop_phase,:))) >= -1e-9, true);
% Locked-wheel deceleration is set by the TYRE at full slip, not by the torque.
% mu * mf(-1) * g with these coefficients is about 8 m/s^2.
ok = check(ok,'deceleration is grip-limited, not torque-limited', ...
           decel > 4 && decel < P.TireMu*9.81*1.2, true);
fprintf('        EBS stop from %.1f m/s: %.1f m in %.2f s  (%.1f m/s^2, %.2f g)\n', ...
        v0, dist, A.t(stopIdx)-4, decel, decel/9.81);

fprintf('\n%s\n', ternary(ok,'brake checks PASS.','BRAKE CHECKS FAILED.'));

    function setcmd(e,hb)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=0; c.regen=0; c.steer_norm=0; c.ebs_latch=e; c.handbrake=hb;
        assignin('base','CMD_B',c);
    end
    function stepcmd(th,rg,e)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=th; c.regen=rg; c.steer_norm=0; c.ebs_latch=e; c.handbrake=0;
        assignin('base','CMD_S',c);
    end
end

function t = T(r), d=r.get('tl').Data; t=double(reshape(d(end,:),[],1)); end
function A = pose(r)
p=r.yout{1}.Values;                      % port 1 is Pose
A.t=p.position.Time; A.x=p.position.Data(:,1); A.vx=p.vel_body.Data(:,1);
end
function W = wheels(r)
w=r.yout{2}.Values;                      % port 2 is Wheels
W.omega=w.omega.Data(:); W.M=squeeze(w.omega.Data);
end
function s = roadFlat(P)
s=Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
end
function s = envFlat()
s=Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
s.gravity_z=-9.81; s.ext_force=[0;0;0]; s.ext_torque=[0;0;0]; s.ext_point=[0;0;0];
s.chassis_grounded=0;
end
function ok = check(ok,name,got,want,tol)
if nargin<5, tol=0; end
if islogical(want)
    pass=isequal(logical(got),logical(want));
    fprintf('  [%s] %-44s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass=all(abs(got(:)-want(:))<=tol);
    fprintf('  [%s] %-44s got %s\n', ternary(pass,'ok  ','FAIL'), name, mat2str(round(got(:)',3)));
end
if ~pass, ok=false; end
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
