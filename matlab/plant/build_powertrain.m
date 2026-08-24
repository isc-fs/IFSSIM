function build_powertrain(outdir)
%BUILD_POWERTRAIN  Fill in IFSSIM_Powertrain: commands -> wheel torque.
%
%   Motor envelope, single-speed drivetrain, battery with state of charge.
%
%   The torque split is a VECTOR, so trying in-wheel motors or torque vectoring
%   is a one-line edit rather than a rewrite.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_Powertrain';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

add_block('simulink/Sources/In1',[name '/Cmd'],'Position',[30 60 60 80], ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Cmd Select'], ...
          'Position',[130 50 140 110],'OutputSignals','throttle,regen');
add_line(name,'Cmd/1','Cmd Select/1','autorouting','on');
add_block('simulink/Sources/In1',[name '/wheel_omega'],'Position',[30 160 60 180], ...
          'PortDimensions','4');

fcn = [name '/Motor and Battery'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[250 40 470 260]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = powertrain_code();

sizes = struct('throttle',1,'regen',1,'w',4,'soc_i',1, ...
               'soc_n',1,'drive_torque',4, ...
               'motor_rpm',1,'motor_torque',1,'motor_power',1, ...
               'batt_soc',1,'batt_voltage',1);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end
params = {'IFSSIM_Ts','IFSSIM_gr','IFSSIM_eta','IFSSIM_Tmax','IFSSIM_Pmax', ...
          'IFSSIM_Treg','IFSSIM_Preg','IFSSIM_Vmax','IFSSIM_Vmin', ...
          'IFSSIM_BAs','IFSSIM_Rint'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

add_block('simulink/Discrete/Unit Delay',[name '/battery SoC (state)'], ...
          'Position',[290 320 380 350],'InitialCondition','IFSSIM_SoC0','SampleTime','-1');

add_block('simulink/Sinks/Out1',[name '/drive_torque'],'Position',[620 60 650 80], ...
          'PortDimensions','4');
add_block('simulink/Signal Routing/Bus Creator',[name '/Powertrain Bus'], ...
          'Position',[550 120 560 260],'Inputs','5', ...
          'OutDataTypeStr','Bus: IFSSIM_PowertrainBus','NonVirtualBus','on');
add_block('simulink/Sinks/Out1',[name '/Powertrain'],'Position',[620 180 650 200], ...
          'OutDataTypeStr','Bus: IFSSIM_PowertrainBus');
add_line(name,'Powertrain Bus/1','Powertrain/1','autorouting','on');

FB = 'Motor and Battery';
add_line(name,'Cmd Select/1',sprintf('%s/1',FB),'autorouting','on');
add_line(name,'Cmd Select/2',sprintf('%s/2',FB),'autorouting','on');
add_line(name,'wheel_omega/1',sprintf('%s/3',FB),'autorouting','on');
add_line(name,'battery SoC (state)/1',sprintf('%s/4',FB),'autorouting','on');
add_line(name,sprintf('%s/1',FB),'battery SoC (state)/1','autorouting','on');
add_line(name,sprintf('%s/2',FB),'drive_torque/1','autorouting','on');
for i = 1:5
    add_line(name,sprintf('%s/%d',FB,2+i),sprintf('Powertrain Bus/%d',i),'autorouting','on');
end

add_block('built-in/Note',[name '/Notes'],'Position',[40 420], ...
    'Text', powertrain_notes(P),'HorizontalAlignment','left');

save_system(name,f);
fprintf('wrote %s\n',f);
fprintf('  motor %.0f Nm / %.0f kW, gear %.3f, eta %.2f, pack %.0f V %.1f kWh\n', ...
        P.MotorMaxTorque, P.MotorMaxPower/1000, P.GearRatio, P.DrivetrainEfficiency, ...
        P.Derived.BatteryVMax, P.Derived.BatteryWh/1000);
close_system(name,0);
end

%% =======================================================================
function c = powertrain_code()
L = {
"function [soc_n, drive_torque, motor_rpm, motor_torque, motor_power, batt_soc, batt_voltage] = ..."
"         powertrain(throttle, regen, w, soc_i)"
"%#codegen"
"% Motor envelope, single-speed drivetrain, battery state of charge."
"%"
"% Wheel order FL, FR, RL, RR."
""
"Ts = IFSSIM_Ts;"
""
"% TORQUE SPLIT. RWD today. Change this vector to try in-wheel motors or torque"
"% vectoring — it is written out precisely so that experiment cannot silently"
"% remain an RWD experiment."
"split = [0; 0; 0.5; 0.5];"
""
"% Motor speed follows the DRIVEN wheels through the single reduction. Using the"
"% mean of the driven pair is the open-differential assumption; a spool or an LSD"
"% would change this and so would per-wheel motors."
"driven = split > 0;"
"if any(driven)"
"    w_drv = sum(w .* driven) / sum(driven);"
"else"
"    w_drv = 0;"
"end"
"w_motor = w_drv * IFSSIM_gr;                 % rad/s"
"motor_rpm = w_motor * 60 / (2*pi);"
""
"% --- torque demand -------------------------------------------------"
"% Drive and regen are SUMMED. Chaos discards one of them: its wheel solver"
"% picks braking OR driving by magnitude, so a simultaneous regen-and-drive"
"% command loses a channel silently. Summing is what the real inverter does."
"th = max(0, min(1, throttle));"
"rg = max(0, min(1, regen));"
"T_cmd = th*IFSSIM_Tmax - rg*IFSSIM_Treg;"
""
"% --- envelope --------------------------------------------------------"
"wa = max(abs(w_motor), 1e-3);"
"if T_cmd >= 0"
"    T = min(T_cmd,  min(IFSSIM_Tmax, IFSSIM_Pmax / wa));"
"else"
"    % REGEN IS POWER LIMITED LONG BEFORE IT IS TORQUE LIMITED. MaxRegenPower is"
"    % the cell input-current cap and at any real speed it binds first, by a"
"    % large factor. This is the whole reason the car cannot stop as hard as its"
"    % motor torque figure suggests."
"    T = max(T_cmd, -min(IFSSIM_Treg, IFSSIM_Preg / wa));"
"end"
"motor_torque = T;"
""
"% --- electrical ------------------------------------------------------"
"P_mech = T * w_motor;"
"if P_mech >= 0"
"    P_elec = P_mech / IFSSIM_eta;            % drawing: losses add to the draw"
"else"
"    P_elec = P_mech * IFSSIM_eta;            % recuperating: losses cut the return"
"end"
"motor_power = P_elec;"
""
"% Open-circuit voltage, linear in SoC. Crude but monotonic and bounded, which"
"% is what matters for not dividing by zero."
"soc = max(0, min(1, soc_i));"
"V_oc = IFSSIM_Vmin + (IFSSIM_Vmax - IFSSIM_Vmin) * soc;"
"I = P_elec / max(V_oc, 1);"
"batt_voltage = V_oc - I * IFSSIM_Rint;"
""
"soc_n = soc - (I / IFSSIM_BAs) * Ts;"
"soc_n = max(0, min(1, soc_n));"
"batt_soc = soc;"
""
"% --- to the wheels ---------------------------------------------------"
"% Efficiency applies to the mechanical path in both directions."
"% split SUMS TO 1, so this distributes the total wheel torque. An earlier"
"% version multiplied by 2 as well, which handed each driven wheel the whole"
"% motor torque and doubled the car's acceleration. The stall test caught it."
"drive_torque = split * (T * IFSSIM_gr * IFSSIM_eta);"
"end"
};
c = char(strjoin(string(L), newline));
end

%% =======================================================================
function t = powertrain_notes(P)
t = sprintf([ ...
 'IFSSIM_POWERTRAIN                        OWNER: powertrain\\n\\n' ...
 'Motor %.0f Nm / %.0f kW, reduction %.3f, efficiency %.2f.\\n' ...
 'Pack %.0f V full, %.0f V empty, %.1f kWh.\\n\\n' ...
 'REGEN IS POWER LIMITED, NOT TORQUE LIMITED: ~%.0f Nm at 10 m/s against a\\n' ...
 '%.0f Nm envelope, a factor of %.1f. Regen is the ONLY service braking this\\n' ...
 'car has, so this cap sets how hard it can stop.\\n\\n' ...
 'Drive and regen are SUMMED — Chaos discards one of them.\\n\\n' ...
 'The torque split is a VECTOR [0 0 0.5 0.5]. Change it for in-wheel motors or\\n' ...
 'torque vectoring.\\n\\n' ...
 'BATTERY PARAMETERS come from matlab/IFS_Sim, NOT settings.json — that model\\n' ...
 'may describe a different car. Confirm, then promote them.\\n\\n' ...
 'Edit build_powertrain.m, not this model — it is regenerated.'], ...
 P.MotorMaxTorque, P.MotorMaxPower/1000, P.GearRatio, P.DrivetrainEfficiency, ...
 P.Derived.BatteryVMax, P.Derived.BatteryVMin, P.Derived.BatteryWh/1000, ...
 P.Derived.RegenTorqueAt10ms, P.MaxRegenTorque, ...
 P.MaxRegenTorque/P.Derived.RegenTorqueAt10ms);
end
