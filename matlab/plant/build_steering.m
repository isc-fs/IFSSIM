function build_steering(outdir)
%BUILD_STEERING  Fill in IFSSIM_Steering: command -> road-wheel angles.
%
%   The command is the SINGLE-TRACK angle: steer_norm * MaxSteerAngle is an
%   equivalent bicycle's angle, and Ackermann splits it across the front wheels.
%   The autonomy plans against a kinematic bicycle, so this keeps both
%   geometries in agreement.
%
%   The actuator (rate limit + lag) is modelled but defaulted to effectively
%   instantaneous — better no lag than the wrong lag.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_Steering';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

add_block('simulink/Sources/In1',[name '/Cmd'],'Position',[30 60 60 80], ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Cmd Select'], ...
          'Position',[130 50 140 100],'OutputSignals','steer_norm');
add_line(name,'Cmd/1','Cmd Select/1','autorouting','on');

fcn = [name '/Rack and Ackermann'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[240 40 440 160]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = steering_code();

sizes = struct('cmd_norm',1,'rack_i',1,'rack_n',1,'steer',4);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end
params = {'IFSSIM_Ts','IFSSIM_dmax','IFSSIM_L','IFSSIM_tF','IFSSIM_ack', ...
          'IFSSIM_drate','IFSSIM_dtau'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

add_block('simulink/Discrete/Unit Delay',[name '/rack angle (state)'], ...
          'Position',[280 220 350 250],'InitialCondition','0','SampleTime','-1');

add_block('simulink/Sinks/Out1',[name '/steer'],'Position',[520 90 550 110], ...
          'PortDimensions','4');

add_line(name,'Cmd Select/1',[fcn(numel(name)+2:end) '/1'],'autorouting','on');
FB = 'Rack and Ackermann';
add_line(name,'rack angle (state)/1',sprintf('%s/2',FB),'autorouting','on');
add_line(name,sprintf('%s/1',FB),'rack angle (state)/1','autorouting','on');
add_line(name,sprintf('%s/2',FB),'steer/1','autorouting','on');

save_system(name,f);
fprintf('wrote %s\n',f);
fprintf('  max road-wheel angle %.1f deg, Ackermann %.0f%%, rate limit %.0f rad/s\n', ...
        rad2deg(P.Derived.MaxSteerAngleRad), P.Assumed.AckermannFraction*100, ...
        P.Assumed.SteerRateLimit);
close_system(name,0);
end

%% =======================================================================
function c = steering_code()
L = {
"function [rack_n, steer] = steering(cmd_norm, rack_i)"
"%#codegen"
"% Normalised steering command -> four road-wheel angles."
"%"
"% The rack state is the SINGLE-TRACK (bicycle) angle. Ackermann then splits it"
"% between the front wheels. The autonomy plans against a kinematic bicycle, so"
"% this is the definition that keeps the controller's geometry and the plant's"
"% in agreement."
""
"Ts = IFSSIM_Ts;"
""
"% Command -> demanded single-track angle."
"cmd = max(-1, min(1, cmd_norm));"
"target = cmd * IFSSIM_dmax;"
""
"% --- actuator ------------------------------------------------------"
"% First-order lag, then a rate limit. Both default to effectively"
"% instantaneous — see the note on why an UNMEASURED actuator model is worse"
"% than none."
"alpha = min(Ts / max(IFSSIM_dtau, 1e-9), 1);"
"lagged = rack_i + alpha * (target - rack_i);"
""
"dmax_step = IFSSIM_drate * Ts;"
"delta = lagged - rack_i;"
"if delta >  dmax_step, delta =  dmax_step; end"
"if delta < -dmax_step, delta = -dmax_step; end"
"rack_n = rack_i + delta;"
""
"% --- Ackermann -----------------------------------------------------"
"% Geometric Ackermann about a common turn centre:"
"%     R = L / tan(d),  tan(d_inner) = L / (R - t/2),  tan(d_outer) = L / (R + t/2)"
"% blended toward parallel steer by the Ackermann fraction."
"d = rack_i;"
"if abs(d) < 1e-6"
"    dl = d; dr = d;                     % straight ahead: no split, and the"
"                                        % formula below divides by tan(d)."
"else"
"    R = IFSSIM_L / tan(d);"
"    % Positive d steers LEFT in ISO 8855 (y is left, yaw is positive left)."
"    % The LEFT wheel is then the inner one and takes the larger angle."
"    d_in  = atan(IFSSIM_L / (abs(R) - IFSSIM_tF/2));"
"    d_out = atan(IFSSIM_L / (abs(R) + IFSSIM_tF/2));"
"    s = sign(d);"
"    % Blend: 1 = full Ackermann, 0 = both wheels parallel at the rack angle."
"    a = IFSSIM_ack;"
"    d_in_b  = a*d_in  + (1-a)*abs(d);"
"    d_out_b = a*d_out + (1-a)*abs(d);"
"    if s > 0"
"        dl = s*d_in_b;  dr = s*d_out_b;     % turning left: left wheel inner"
"    else"
"        dl = s*d_out_b; dr = s*d_in_b;      % turning right: right wheel inner"
"    end"
"end"
""
"% FL, FR, RL, RR. The rear axle does not steer on this car."
"steer = [dl; dr; 0; 0];"
"end"
};
c = char(strjoin(string(L), newline));
end

