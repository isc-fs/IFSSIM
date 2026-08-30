function R = validate_dualtrack(speeds)
%VALIDATE_DUALTRACK  Fit the design model against the reference plant.
%
%   The two-model structure the literature uses only means something if the
%   two are actually compared. Escofet validates a Dual-Track NTV against
%   VI-CarRealTime and against logged CAT15x data, and reports one number per
%   manoeuvre: the relative yaw-rate error of eq. (12). This does the first
%   half of that -- design model against reference plant -- which needs no
%   vehicle data and can therefore be run today.
%
%   The second half needs a bag from a car driven fast enough to matter. See
%   docs/vehicle_dynamics_alignment.md: every bag in this repo peaks below
%   4.6 m/s, where none of the physics being compared here does anything.
%
%   The manoeuvre is a ramp steer at roughly constant speed, which is where
%   the thesis starts too. Both models are given the SAME steering command and
%   the SAME speed trace -- the plant's own, measured from its output, so the
%   design model is never asked to predict a speed it was not given.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','plant'));
addpath(fullfile(here,'..','plant','models'));
if nargin < 1 || isempty(speeds), speeds = [6 9 12]; end

P = ifssim_load_workspace();
M = dualtrack_build();

h = build_harness(P);
R = struct('speed',{},'fit',{},'eps',{},'peak_ay',{},'n',{}, ...
           't',{},'vx',{},'delta',{},'r_plant',{},'r_model',{}, ...
           'ay_plant',{},'win',{});

fprintf('\n=== design model vs reference plant: ramp steer ===\n');
fprintf('  metric is Escofet eq. (12) on YAW RATE. The thesis calls 96.7%% on a\n');
fprintf('  ramp steer and 82.5%% on an autocross lap satisfactory.\n\n');
fprintf('  target v   plant peak ay   fit      eps_rel\n');

for v = speeds
    [t, delta, vx, r_plant, ay_plant, steering] = run_plant(h, P, v, M);

    % The design model is driven by what the PLANT did, not by what it was
    % asked to do: same steering, same speed trace, same clock.
    Y = dualtrack_sim(t, delta, vx, M);

    % Score the STEERING phase only. The run starts with three seconds of
    % straight-line acceleration to get up to speed, and a yaw-rate error
    % measured against a yaw rate of zero says nothing about either model.
    w = steering;
    [pct, eps_rel] = yaw_rate_fit(r_plant(w), Y.r(w));

    R(end+1) = struct('speed',v,'fit',pct,'eps',eps_rel, ...
                      'peak_ay',max(abs(ay_plant(w))),'n',sum(w), ...
                      't',t,'vx',vx,'delta',delta,'r_plant',r_plant, ...
                      'r_model',Y.r,'ay_plant',ay_plant,'win',w); %#ok<AGROW>
    fprintf('  %5.1f      %6.2f m/s^2    %6.2f%%   %6.2f\n', ...
            v, max(abs(ay_plant(w))), pct, eps_rel);
end

fprintf('\n  A LOW number here is not automatically the design model''s fault.\n');
fprintf('  The plant has suspension states, relaxation and a rate-limited\n');
fprintf('  steering box that this model deliberately does not; and the plant\n');
fprintf('  has NO roll centre, which this model does. They are supposed to\n');
fprintf('  differ -- the question is whether they differ by more than the\n');
fprintf('  simplifications can account for.\n');
end

% -----------------------------------------------------------------------
function h = build_harness(P)
h = 'dualtrack_validate_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','1/960', ...
            'StartTime','0','StopTime','8.0','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);
s = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
assignin('base','ROAD_D',s);
e = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
e.gravity_z=-9.81; e.ext_force=[0;0;0]; e.ext_torque=[0;0;0]; e.ext_point=[0;0;0];
e.chassis_grounded=0; assignin('base','ENV_D',e);
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','ROAD_D', ...
    'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 120 130 150]);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','ENV_D', ...
    'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 180 130 210]);
% Steering and throttle arrive as time series, so the ramp is a real ramp and
% not a staircase of constant-block reruns.
% A ramp steer is defined at CONSTANT SPEED, so the speed has to be HELD, not
% hoped for. An open-loop throttle does not hold it: at a throttle that gave 6
% m/s the car reached 18.5, the steering ramp sized for 6 m/s was then far
% past the limit at 18.5, and the plant spun -- which read as a catastrophic
% model disagreement when it was only a harness that could not drive.
add_block('simulink/Sources/From Workspace',[h '/STEERTS'],'VariableName','steer_ts', ...
    'Position',[40 40 110 70]);
add_block('simulink/Sources/From Workspace',[h '/VTGT'],'VariableName','vtgt_ts', ...
    'Position',[40 100 110 130]);
add_block('simulink/Math Operations/Sum',[h '/verr'],'Inputs','+-','Position',[140 100 160 130]);
add_block('simulink/Math Operations/Gain',[h '/Kp'],'Gain','0.7','Position',[175 100 205 130]);
add_block('simulink/Discontinuities/Saturation',[h '/thr'], ...
    'UpperLimit','1','LowerLimit','0','Position',[220 80 250 110]);
add_block('simulink/Math Operations/Gain',[h '/neg'],'Gain','-1','Position',[220 130 250 160]);
add_block('simulink/Discontinuities/Saturation',[h '/rgn'], ...
    'UpperLimit','1','LowerLimit','0','Position',[265 130 295 160]);
% Five elements, not three: ebs_latch and handbrake are part of the command
% bus and a Bus Creator has to supply every one of them.
add_block('simulink/Sources/Constant',[h '/zeroA'],'Value','0','Position',[265 175 295 195]);
add_block('simulink/Sources/Constant',[h '/zeroB'],'Value','0','Position',[265 205 295 225]);
add_block('simulink/Signal Routing/Bus Creator',[h '/CMD'],'Inputs','5', ...
    'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[320 40 330 230]);
add_block('simulink/Signal Routing/Bus Selector',[h '/psel'], ...
    'OutputSignals','vel_body','Position',[470 250 475 280]);
add_block('simulink/Signal Routing/Demux',[h '/vd'],'Outputs','3','Position',[520 250 525 280]);
% The plant has direct feedthrough from command to pose, so feeding speed
% back without a delay makes an algebraic loop that Simulink cannot solve
% through a model reference with states. One step of memory at 1/960 s breaks
% it and costs nothing a speed controller would notice.
add_block('simulink/Discrete/Memory',[h '/vmem'],'InitialCondition','0', ...
    'Position',[420 250 450 280]);
add_line(h,'VTGT/1','verr/1','autorouting','on');
add_line(h,'verr/1','Kp/1','autorouting','on');
add_line(h,'Kp/1','thr/1','autorouting','on');
add_line(h,'Kp/1','neg/1','autorouting','on');
add_line(h,'neg/1','rgn/1','autorouting','on');
add_line(h,'CMD/1','Plant/1','autorouting','on');
add_line(h,'Plant/1','psel/1','autorouting','on');
add_line(h,'psel/1','vd/1','autorouting','on');
add_line(h,'vd/1','vmem/1','autorouting','on');
add_line(h,'vmem/1','verr/2','autorouting','on');

% A Bus Creator checks element NAMES against the bus object, so every line
% into it has to be named for the element it fills.
nm = {'thr/1','throttle'; 'rgn/1','regen'; 'STEERTS/1','steer_norm'; ...
      'zeroA/1','ebs_latch'; 'zeroB/1','handbrake'};
for q = 1:size(nm,1)
    L = add_line(h, nm{q,1}, sprintf('CMD/%d',q), 'autorouting','on');
    set_param(L,'Name',nm{q,2});
end
add_line(h,'ROAD/1','Plant/2','autorouting','on');
add_line(h,'ENV/1','Plant/3','autorouting','on');
add_block('simulink/Sinks/To Workspace',[h '/pose_out'],'VariableName','pose_d', ...
    'SaveFormat','Timeseries','Position',[500 70 570 100]);
add_block('simulink/Sinks/To Workspace',[h '/wheel_out'],'VariableName','wheel_d', ...
    'SaveFormat','Timeseries','Position',[500 130 570 160]);
add_line(h,'Plant/1','pose_out/1','autorouting','on');
add_line(h,'Plant/2','wheel_out/1','autorouting','on');
end

% -----------------------------------------------------------------------
function [t, delta, vx, r, ay, steering] = run_plant(h, P, vtarget, M)
% Accelerate to roughly vtarget, then ramp the steering. Speed is held with a
% crude proportional throttle -- crude is fine, because whatever speed the
% plant ends up at is measured and handed to the design model verbatim.
%
% The ramp stops at the steer angle that puts the car AT its grip limit for
% this speed, not at full lock. A ramp steer exists to find the limit; driving
% far past it spins the car, and two spun trajectories diverge for reasons
% that have nothing to do with the modelling difference under test. At 12 m/s
% full lock asks for 6 g.
dt   = 1/240;
tt   = (0:dt:8)';
n    = numel(tt);
dlim = min(M.maxSteer, atan(M.L * M.mu * 9.81 / max(vtarget,1)^2));
frac = dlim / M.maxSteer;
ramp = frac * max(0, (tt - 3)/5);          % 0 for 3 s while speed settles
assignin('base','steer_ts', timeseries(ramp, tt));
assignin('base','vtgt_ts',  timeseries(repmat(vtarget,n,1), tt));
r0 = sim(h);

p = r0.get('pose_d');
tp = p.position.Time;
vb = p.vel_body.Data;  ob = p.omega_body.Data;
t  = tp;
vx = vb(:,1);
r  = ob(:,3);
% True lateral acceleration, not the r*vx proxy: they agree in steady state
% and part company completely once the car starts to spin, which is exactly
% when a bogus 6 g would otherwise get printed.
vy = vb(:,2);
dvy = [0; diff(vy)./max(diff(tp),eps)];
ay  = dvy + r .* vx;
delta = interp1(tt, ramp, tp, 'linear', 'extrap') * P.MaxSteerAngle * pi/180;
steering = tp > 3.0;
end
