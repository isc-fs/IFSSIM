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
% ode1 and the plant's exact step: this model now carries a Simscape
% accumulator, and Simscape cannot run under a discrete solver.
set_param(name,'SolverType','Fixed-step','Solver','ode1', ...
               'FixedStep','0.0010416666666666671','StartTime','0','StopTime','inf');

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

sizes = struct('throttle',1,'regen',1,'w',4,'v_pack',1,'soc_pack',1, ...
               'i_demand',1,'drive_torque',4, ...
               'motor_rpm',1,'motor_torque',1,'motor_power',1, ...
               'batt_soc',1,'batt_voltage',1);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end
% Vmax/Vmin/BAs/Rint are gone: open-circuit voltage, capacity and internal
% resistance are the Simscape pack's business now, and having them in two
% places is how they come to disagree. Ipk is what the pack can pass, derived
% from the cell and the arrangement.
params = {'IFSSIM_Ts','IFSSIM_gr','IFSSIM_eta','IFSSIM_Tmax','IFSSIM_Pmax', ...
          'IFSSIM_Treg','IFSSIM_Preg','IFSSIM_Ipk'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

% THE ACCUMULATOR. Five Simscape modules; see build_battery_pack.
packInfo = build_battery_pack(name,'Accumulator',[300 320 460 470]);

% Terminal voltage comes back to the torque envelope through a MEMORY, not a
% unit delay: the pack is continuous and a unit delay would impose a discrete
% rate on it. The loop has to be broken somewhere -- the envelope needs the
% voltage, and the voltage depends on the current the envelope asks for -- and
% one step at 960 Hz is a step the pack cannot move far in.
% INITIALISED TO THE PACK'S OWN FULL-CHARGE VOLTAGE, not zero. The current
% demand is P_elec / v_pack, so a zero initial voltage asks for the current
% that would make the requested power at one volt -- which on the first step
% is an enormous charge current, and the battery block quite rightly asserts
% that state of charge cannot exceed 1.
add_block('simulink/Discrete/Memory',[name '/pack voltage (delay)'], ...
          'Position',[230 400 300 430], ...
          'InitialCondition',num2str(packInfo.V0(end)*packInfo.NumModules,8));

% The state of charge needs breaking too, even though the torque envelope
% does not use it. Simulink takes every output of a MATLAB Function block to
% depend on every input, so routing SoC straight back closes an algebraic
% loop through the Simscape solver -- which cannot be in one, because it
% updates state while computing outputs. Reporting a value one step old
% costs nothing; not being able to compile costs everything.
add_block('simulink/Discrete/Memory',[name '/pack soc (delay)'], ...
          'Position',[230 450 300 480],'InitialCondition','1');

% Per-module state of charge. Module 1 feeds the reported SoC; while the
% modules are identical and in series they all read the same, so this is not
% a choice between them. The other four are brought out and terminated
% because they are the hook a BMS or a per-module thermal model needs, and
% they should not have to be re-added to be used.
for mm = 2:packInfo.NumModules
    add_block('simulink/Sinks/Terminator',sprintf('%s/soc%d (spare)',name,mm), ...
              'Position',[540 470+30*mm 560 490+30*mm]);
end

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
add_line(name,'pack voltage (delay)/1',sprintf('%s/4',FB),'autorouting','on');
add_line(name,'Accumulator/2','pack soc (delay)/1','autorouting','on');            % module 1 SoC
add_line(name,'pack soc (delay)/1',       sprintf('%s/5',FB),'autorouting','on');
add_line(name,sprintf('%s/1',FB),'Accumulator/1','autorouting','on');              % current demand
add_line(name,'Accumulator/1','pack voltage (delay)/1','autorouting','on');        % v_pack back
for mm = 2:packInfo.NumModules
    add_line(name,sprintf('Accumulator/%d',mm+1),sprintf('soc%d (spare)/1',mm),'autorouting','on');
end
add_line(name,sprintf('%s/2',FB),'drive_torque/1','autorouting','on');
for i = 1:5
    add_line(name,sprintf('%s/%d',FB,2+i),sprintf('Powertrain Bus/%d',i),'autorouting','on');
end

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
"function [i_demand, drive_torque, motor_rpm, motor_torque, motor_power, batt_soc, batt_voltage] = ..."
"         powertrain(throttle, regen, w, v_pack, soc_pack)"
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
""
"% WHAT THE ACCUMULATOR CAN ACTUALLY GIVE, at the voltage it is at right now"
"% rather than at its nameplate. This is the limit that did not exist before:"
"% the envelope was min(Tmax, Pmax/w), the battery was computed and ignored,"
"% and the plant handed out 80 kW from a pack that can supply 51."
"%"
"% v_pack is last step's terminal voltage, so it already has the sag from the"
"% current we are about to ask for. Multiplied by what the pack can pass, it"
"% is the electrical power available; times efficiency is the mechanical"
"% power that reaches the shaft."
"vb     = max(v_pack, 1);"
"P_pack = vb * IFSSIM_Ipk;"
""
"if T_cmd >= 0"
"    P_avail = min(IFSSIM_Pmax, P_pack * IFSSIM_eta);"
"    T = min(T_cmd,  min(IFSSIM_Tmax, P_avail / wa));"
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
"% The battery is no longer modelled here. Open-circuit voltage, capacity,"
"% internal resistance and the state of charge all live in the Simscape"
"% accumulator now, which is five modules of real cells rather than a line"
"% through two voltages. What crosses this boundary is a CURRENT DEMAND out"
"% and a terminal voltage and state of charge back."
"%"
"% Dividing by the terminal voltage rather than the open-circuit voltage is"
"% deliberate: the pack has already sagged under load, and the current needed"
"% to make a given power is set by the voltage actually present."
"i_demand    = P_elec / vb;"
"batt_voltage = v_pack;"
"batt_soc     = soc_pack;"
""
"% --- to the wheels ---------------------------------------------------"
"% EFFICIENCY IS DIRECTIONAL, and it was not. The comment here used to read"
"% efficiency applies to the mechanical path in both directions, and the"
"% code multiplied by eta unconditionally, which is wrong in each direction"
"% for a different reason."
"%"
"% Driving, the motor makes T and the wheel receives less: T*gr*eta. That is"
"% right on its own, but the electrical branch above ALREADY charges the loss"
"% once, as P_elec = P_mech/eta -- so the loss was being taken twice and the"
"% car was accelerating on eta^2 = 0.846 of its drivetrain."
"%"
"% Regenerating, the wheel drives the motor, so the wheel must supply the"
"% braking torque PLUS the losses: T*gr/eta, not T*gr*eta. Written the old"
"% way the wheel braked with less torque than the motor absorbed, which is"
"% energy appearing from nowhere, and it understated the only service braking"
"% this car has by a factor of eta^2."
"%"
"% split SUMS TO 1, so this distributes the total wheel torque. An earlier"
"% version multiplied by 2 as well, which handed each driven wheel the whole"
"% motor torque and doubled the car's acceleration. The stall test caught it."
"if T >= 0"
"    T_wheel = T * IFSSIM_gr * IFSSIM_eta;      % losses reduce what arrives"
"else"
"    T_wheel = T * IFSSIM_gr / IFSSIM_eta;      % losses must also be braked"
"end"
"drive_torque = split * T_wheel;"
"end"
};
c = char(strjoin(string(L), newline));
end

