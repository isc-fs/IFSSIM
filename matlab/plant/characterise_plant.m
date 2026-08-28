function T = characterise_plant()
%CHARACTERISE_PLANT  Measure the handling numbers a controls engineer needs.
%
%   Constant-steer sweeps at constant speed, run to steady state. Reports, per
%   speed and steering angle:
%
%     yaw gain ratio   achieved yaw rate / kinematic yaw rate.
%                      Stanley outputs a ROAD-WHEEL ANGLE and implicitly assumes
%                      the car achieves the kinematic yaw for it. Anything below
%                      1.0 is loop gain the controller does not know it has lost.
%     lateral g        the grip actually available
%     understeer       deg of extra steer needed per g, the classic figure
%
%   Needs no vehicle and no UE. This is the plant characterised on its own terms.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();
L = P.Wheelbase;

h = 'charac_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','1/960','StartTime','0','StopTime','12','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Car'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[300 60 460 220]);

% Driver: proportional speed hold, with the steer ramped in after the car has
% settled so the measurement is of a steady state and not of a transient.
drv=[h '/Driver'];
add_block('simulink/User-Defined Functions/MATLAB Function',drv,'Position',[80 60 220 160]);
ch = sfroot().find('-isa','Stateflow.EMChart','Path',drv);
ch.Script = char(strjoin(string({
 "function [throttle, steer] = drv(t, v)"
 "%#codegen"
 "% Hold TARGET_V, then ramp steer in from 4 s so the plant reaches steady state."
 "throttle = max(0, min(1, 0.6*(TARGET_V - v)));"
 "if t < 4"
 "    steer = 0;"
 "else"
 "    steer = TARGET_STEER * min((t-4)/2, 1);"
 "end"
 "end"}), newline));
for d = ch.find('-isa','Stateflow.Data')'
    if any(strcmp(d.Name,{'t','v','throttle','steer'})), d.Props.Array.Size='1'; end
end
% Parameters must be CREATED, not discovered. find() cannot return data for
% identifiers the parser could not resolve, so relying on it is circular — the
% chart fails to parse precisely because the parameter does not exist yet.
existing = {ch.find('-isa','Stateflow.Data').Name};
for nm = {'TARGET_V','TARGET_STEER'}
    if any(strcmp(existing, nm{1})), continue; end
    d = Stateflow.Data(ch);
    d.Name = nm{1}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end
add_block('simulink/Sources/Clock',[h '/Clock'],'Position',[30 75 50 95]);
add_line(h,'Clock/1','Driver/1','autorouting','on');

assignin('base','ROAD_C',roadFlat(P)); assignin('base','ENV_C',envFlat());
% Three separate constants, not one fanned out: a Bus Creator names elements by
% SIGNAL name, and one source feeding three ports gives all three the same name.
add_block('simulink/Sources/Constant',[h '/zr'],'Value','0','Position',[80 190 110 210]);
add_block('simulink/Sources/Constant',[h '/ze'],'Value','0','Position',[80 215 110 235]);
add_block('simulink/Sources/Constant',[h '/zh'],'Value','0','Position',[80 240 110 260]);
add_block('simulink/Signal Routing/Bus Creator',[h '/Cmd'],'Position',[250 55 260 175], ...
    'Inputs','5','OutDataTypeStr','Bus: IFSSIM_CmdBus','NonVirtualBus','on');
LN={'Driver/1','Cmd/1','throttle';'zr/1','Cmd/2','regen';'Driver/2','Cmd/3','steer_norm';
    'ze/1','Cmd/4','ebs_latch';'zh/1','Cmd/5','handbrake'};
for i=1:size(LN,1), set_param(add_line(h,LN{i,1},LN{i,2},'autorouting','on'),'Name',LN{i,3}); end
add_block('simulink/Sources/Constant',[h '/R'],'Value','ROAD_C', ...
    'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[80 280 150 310]);
add_block('simulink/Sources/Constant',[h '/E'],'Value','ENV_C', ...
    'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[80 330 150 360]);
add_line(h,'Cmd/1','Car/1','autorouting','on');
add_line(h,'R/1','Car/2','autorouting','on');
add_line(h,'E/1','Car/3','autorouting','on');

% Speed feedback for the hold, and logging.
add_block('simulink/Signal Routing/Bus Selector',[h '/sel'],'Position',[520 60 530 160], ...
    'OutputSignals','vel_body,omega_body,accel_proper,attitude');
add_line(h,'Car/1','sel/1','autorouting','on');
add_block('simulink/Signal Routing/Demux',[h '/vd'],'Outputs','3','Position',[570 60 575 100]);
add_line(h,'sel/1','vd/1','autorouting','on');
% Break the speed-feedback loop. The plant's Pose bus has direct-feedthrough
% components (accel_proper depends on this step's inputs), so feeding velocity
% straight back into the driver is an algebraic loop. One step of delay at
% 1/960 s is 1 ms and cannot affect a steady-state measurement.
add_block('simulink/Discrete/Unit Delay',[h '/vdel'], ...
    'Position',[590 60 640 90],'InitialCondition','0','SampleTime','-1');
add_line(h,'vd/1','vdel/1','autorouting','on');
add_line(h,'vdel/1','Driver/2','autorouting','on');
add_block('simulink/Sinks/To Workspace',[h '/log'],'VariableName','cl', ...
    'SaveFormat','Timeseries','Position',[620 140 690 170]);
add_line(h,'Car/1','log/1','autorouting','on');

assignin('base','TARGET_V', 4); assignin('base','TARGET_STEER', 0);

speeds = [8 12];
steers = [0.10 0.20 0.30 0.40 0.50 0.65 0.80 1.00];   % normalised command
rows = {};
fprintf('\n=== plant characterisation ===\n');
fprintf('max road-wheel angle %.1f deg, wheelbase %.3f m, mu %.2f\n\n', ...
        P.MaxSteerAngle, L, P.TireMu);
for v = speeds
    assignin('base','TARGET_V', v);
    fprintf('--- target %g m/s ---\n', v);
    fprintf('%8s %10s %10s %10s %10s %10s\n', ...
            'steer', 'delta deg', 'v m/s', 'yaw r/s', 'lat g', 'yaw/kin');
    for s = steers
        assignin('base','TARGET_STEER', s);
        r = sim(h);
        d = r.get('cl');
        n = numel(d.vel_body.Time);
        w = round(0.85*n):n;                         % last 15% = steady state
        vx = mean(d.vel_body.Data(w,1));
        yr = mean(d.omega_body.Data(w,3));
        ay = mean(d.accel_proper.Data(w,2));
        delta = s * P.Derived.MaxSteerAngleRad;      % single-track road-wheel angle
        kin = vx * tan(delta) / L;                   % kinematic yaw rate
        ratio = yr / max(kin, 1e-9);
        fprintf('%8.2f %10.2f %10.2f %10.4f %10.3f %10.3f\n', ...
                s, rad2deg(delta), vx, yr, ay/9.81, ratio);
        rows(end+1,:) = {v, s, rad2deg(delta), vx, yr, ay/9.81, ratio}; %#ok<AGROW>
    end
    fprintf('\n');
end
T = cell2table(rows, 'VariableNames', ...
    {'target_v','steer_norm','delta_deg','v','yaw_rate','lat_g','yaw_over_kinematic'});
close_system(h,0);
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
