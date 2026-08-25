function build_brakes(outdir)
%BUILD_BRAKES  Fill in IFSSIM_Brakes: EBS, and the service brake this car lacks.
%
%   THIS CAR HAS NO HYDRAULIC SERVICE BRAKE. Slowing down under normal driving
%   is motor regen, which lives in the powertrain and arrives at the tyre as a
%   NEGATIVE drive torque. What is left for this block is the EBS: a pneumatic
%   emergency system acting on all four corners.
%
%   So a zero output here is correct during normal driving, and that is worth
%   saying out loud — a reader who sees brake_torque flat at zero while the car
%   slows down has not found a bug.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_Brakes';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

add_block('simulink/Sources/In1',[name '/Cmd'],'Position',[30 60 60 80], ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Cmd Select'], ...
          'Position',[130 50 140 110],'OutputSignals','ebs_latch,handbrake');
add_line(name,'Cmd/1','Cmd Select/1','autorouting','on');
add_block('simulink/Sources/In1',[name '/wheel_omega'],'Position',[30 160 60 180], ...
          'PortDimensions','4');

fcn = [name '/EBS'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[250 40 430 180]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = brakes_code();

sizes = struct('ebs',1,'hbrake',1,'w',4,'p_i',1,'p_n',1,'brake_torque',4);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end
params = {'IFSSIM_Ts','IFSSIM_Tebs','IFSSIM_ebstau'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

add_block('simulink/Discrete/Unit Delay',[name '/line pressure (state)'], ...
          'Position',[280 240 380 270],'InitialCondition','0','SampleTime','-1');
add_block('simulink/Sinks/Out1',[name '/brake_torque'],'Position',[520 90 550 110], ...
          'PortDimensions','4');

FB = 'EBS';
add_line(name,'Cmd Select/1',sprintf('%s/1',FB),'autorouting','on');
add_line(name,'Cmd Select/2',sprintf('%s/2',FB),'autorouting','on');
add_line(name,'wheel_omega/1',sprintf('%s/3',FB),'autorouting','on');
add_line(name,'line pressure (state)/1',sprintf('%s/4',FB),'autorouting','on');
add_line(name,sprintf('%s/1',FB),'line pressure (state)/1','autorouting','on');
add_line(name,sprintf('%s/2',FB),'brake_torque/1','autorouting','on');

add_block('built-in/Note',[name '/Notes'],'Position',[40 320], ...
    'Text', sprintf([ ...
     'IFSSIM_BRAKES                            OWNER: braking\\n\\n' ...
     'THIS CAR HAS NO HYDRAULIC SERVICE BRAKE. Normal deceleration is motor\\n' ...
     'regen, which lives in the powertrain and reaches the tyre as NEGATIVE\\n' ...
     'drive torque. A flat zero here while the car slows down is correct.\\n\\n' ...
     'What is left is the EBS: pneumatic, all four corners.\\n\\n' ...
     'EBS torque %.0f N.m per wheel = %.1fx the grip limit at static load.\\n' ...
     'Sized as a MULTIPLE of grip on purpose. Above the grip limit the wheel\\n' ...
     'locks, and from there deceleration is set by the TYRE at full slip — not\\n' ...
     'by how much more torque the system could apply. So this number decides\\n' ...
     'WHETHER it locks, not how hard it stops.\\n\\n' ...
     'It replaces Chaos''s default 3000 N.m per wheel: about 15x grip, chosen by\\n' ...
     'nobody, and the number the whole emergency-braking case rested on.\\n\\n' ...
     'FILL TIME is what actually matters and is NOT measured (%.0e s here,\\n' ...
     'effectively instant). It directly flatters every stopping distance.\\n' ...
     'Bench the pneumatic system and set IFSSIM_ebstau.\\n\\n' ...
     'Edit build_brakes.m, not this model — it is regenerated.'], ...
     P.Derived.EbsTorquePerWheel, P.Assumed.EbsGripMultiple, P.Assumed.EbsFillTau), ...
     'HorizontalAlignment','left');

save_system(name,f);
fprintf('wrote %s\n',f);
fprintf('  EBS %.0f N.m/wheel (%.1fx grip limit), fill tau %.0e s\n', ...
        P.Derived.EbsTorquePerWheel, P.Assumed.EbsGripMultiple, P.Assumed.EbsFillTau);
close_system(name,0);
end

%% =======================================================================
function c = brakes_code()
L = {
"function [p_n, brake_torque] = brakes(ebs, hbrake, w, p_i)"
"%#codegen"
"% EBS only. There is no hydraulic service brake on this car; regen does that"
"% job and arrives at the tyre as negative drive torque from the powertrain."
""
"Ts = IFSSIM_Ts;"
""
"% Either channel commands the same pneumatic system. They are separate inputs"
"% because the LATCH logic differs upstream — EBS latches until the mission"
"% flow releases it, the handbrake does not — but the hardware is one system."
"demand = max(ebs, hbrake);"
"demand = max(0, min(1, demand));"
""
"% Line pressure as a first-order fill. Defaulted to effectively instant, so"
"% this is a hook rather than a model until someone measures the real system."
"alpha = min(Ts / max(IFSSIM_ebstau, 1e-9), 1);"
"p_n = p_i + alpha * (demand - p_i);"
""
"% Magnitude only. The tyre block owns the sign, because it owns the wheel"
"% state and is the only place that knows which way the wheel is turning."
"T = p_i * IFSSIM_Tebs;"
"brake_torque = [T; T; T; T];"
"end"
};
c = char(strjoin(string(L), newline));
end
