function info = build_battery_pack(mdl, sysName, pos)
%BUILD_BATTERY_PACK  A Simscape accumulator, one block per real module.
%
%   info = BUILD_BATTERY_PACK(mdl, sysName, pos) adds a subsystem to mdl
%   holding the accumulator as Simscape Battery blocks, and returns the names
%   of its Simulink ports.
%
%   ONE BLOCK PER MODULE, NOT ONE PER CELL. The car has 570 cells and
%   modelling each one would put 570 states inside a plant that steps at 960
%   Hz, which is not a trade anyone would take. Each of the five real modules
%   is one Table-Based battery block, lumped as its own 19s6p: capacity
%   multiplied by the parallel count, open-circuit voltage by the series
%   count, resistance by series-over-parallel.
%
%   Five is not an arbitrary compromise -- it is how the accumulator is
%   actually built and how it is actually instrumented. A per-module SoC
%   output and a per-module thermal port are exactly the hooks a BMS or a
%   cooling model needs, and neither exists at cell level on the real car
%   either.
%
%   The subsystem takes a current demand in amps and returns pack terminal
%   voltage and per-module state of charge.
%
%   See also PACK_FROM_CELLS, PROBE_SIMSCAPE_BATTERY.

if nargin < 3, pos = [300 400 460 520]; end
P = ifssim_params();
addpath(fullfile(fileparts(mfilename('fullpath')),'..','car'));
K = pack_from_cells(car_spec());
C = car_spec();
cv = @(n) C.Fields.(strrep(n,'.','_')).value;

nMod = cv('Pack.ModulesInSeries') * cv('Pack.ModulesInParallel');
ns   = cv('Pack.CellsSeriesPerModule');
np   = cv('Pack.CellsParallelPerModule');

% Per-module lumped equivalent.
AH   = np * cv('Cell.CapacityAh');
Rmod = cv('Cell.Rint') * ns / np;

% Open-circuit voltage against state of charge. A straight line between the
% cell's own limits, times the series count. It is crude and it is honest:
% nobody has discharged one of these cells and recorded the curve, so a
% fitted-looking curve would be a fiction with more decimal places. The shape
% is the first thing to replace when somebody runs a cell.
socv = [0 .1 .25 .5 .75 .9 1];
vcell = cv('Cell.VMin') + (cv('Cell.VMax') - cv('Cell.VMin')) * socv;
V0   = ns * vcell;
R0   = Rmod * ones(size(socv));

load_system('batt_lib');
sys = [mdl '/' sysName];
add_block('simulink/Ports & Subsystems/Subsystem', sys, 'Position', pos);
delete_line(sys,'In1/1','Out1/1'); delete_block([sys '/In1']); delete_block([sys '/Out1']);

SOLVER = sprintf('nesl_utility/Solver\nConfiguration');
PS2S   = sprintf('nesl_utility/PS-Simulink\nConverter');
S2PS   = sprintf('nesl_utility/Simulink-PS\nConverter');

add_block('simulink/Sources/In1',[sys '/I_demand'],'Position',[30 60 60 80]);
add_block(S2PS,[sys '/S2PS'],'Position',[110 55 150 85]);
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ...
          [sys '/ISRC'],'Position',[210 40 270 120]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
          [sys '/GND'],'Position',[220 420 260 460]);
add_block(SOLVER,[sys '/SC'],'Position',[60 420 120 460]);

for m = 1:nMod
    b = sprintf('%s/MOD%d', sys, m);
    add_block('batt_lib/Cells/Battery (Table-Based)', b, ...
              'Position',[360 40+90*(m-1) 440 110+90*(m-1)]);
    % T_dependence OFF FIRST. It defaults to 'yes', and while it is on the
    % block reads the temperature-indexed V0_mat and R0_mat and IGNORES
    % V0_vec and R0_vec entirely -- so the cell data below is written,
    % accepted, and silently unused. The pack then reports a voltage that
    % has nothing to do with the numbers you set.
    set_param(b, 'T_dependence','simscape.enum.tablebattery.temperature_dependence.no');
    set_param(b, 'SOC_vec', mat2str(socv), 'V0_vec', mat2str(V0,8), ...
                 'R0_vec', mat2str(R0,8), 'AH', num2str(AH,8), ...
                 'SOC_port','simscape.enum.tablebattery.enable.yes');
    add_block(PS2S, sprintf('%s/P2S%d',sys,m), 'Position',[500 45+90*(m-1) 540 75+90*(m-1)]);
    % Port numbers are PINNED, not left to creation order. Left implicit the
    % SoC outputs take ports 1..n and v_pack lands last, so anything reading
    % 'Pack/1' expecting a voltage silently gets a state of charge -- which
    % reads 1.0 at full charge and looks exactly like a plausible bad number.
    add_block('simulink/Sinks/Out1', sprintf('%s/soc%d',sys,m), ...
              'Position',[580 50+90*(m-1) 610 70+90*(m-1)], 'Port', num2str(m+1));
end
add_block('fl_lib/Electrical/Electrical Sensors/Voltage Sensor',[sys '/VS'],'Position',[210 250 270 310]);
add_block(PS2S,[sys '/P2Sv'],'Position',[120 260 160 300]);
add_block('simulink/Sinks/Out1',[sys '/v_pack'],'Position',[40 270 70 290],'Port','1');

Pp = @(b) get_param([sys '/' b],'PortHandles');
add_line(sys,'I_demand/1','S2PS/1');
% PORT ORDER, established by testing each port rather than guessing. Both the
% controlled current source and the voltage sensor use the same layout, and
% it is NOT the one the diagram suggests:
%
%   LConn(1) = electrical terminal
%   RConn(1) = PHYSICAL SIGNAL          <- the one that is easy to get wrong
%   RConn(2) = electrical terminal
%
% Driving a signal into an electrical port fails with a domain-rules message
% that never names the offending port, so every wrong guess costs a full
% build cycle to find.
add_line(sys, Pp('S2PS').RConn(1), Pp('ISRC').RConn(1));

% Modules in series: each one's + to the next one's -.
prevPlus = Pp('MOD1').RConn(1);
add_line(sys, Pp('MOD1').LConn(1), Pp('GND').LConn(1));      % pack minus at ground
for m = 2:nMod
    add_line(sys, prevPlus, Pp(sprintf('MOD%d',m)).LConn(1));
    prevPlus = Pp(sprintf('MOD%d',m)).RConn(1);
end
% Current source across the stack: pack plus -> source -> ground.
add_line(sys, prevPlus,            Pp('ISRC').LConn(1));   % pack + to source
add_line(sys, Pp('ISRC').RConn(2), Pp('GND').LConn(1));   % source to ground
% Sense the whole stack.
% POLARITY: RConn(2) is the sensor's positive side, not LConn(1). Wired the
% other way the pack reads a perfectly plausible -367 V, which is the right
% magnitude with the wrong sign -- the kind of error that survives a glance
% at the number and fails later as a negative power draw.
add_line(sys, Pp('VS').RConn(2), prevPlus);             % across the stack, +
add_line(sys, Pp('VS').LConn(1), Pp('GND').LConn(1));   % across the stack, -
add_line(sys, Pp('VS').RConn(1), Pp('P2Sv').LConn(1));  % the reading itself
add_line(sys,'P2Sv/1','v_pack/1');
% The solver hangs off the network. Branching, which add_line does for us.
add_line(sys, Pp('SC').RConn(1), Pp('GND').LConn(1));
for m = 1:nMod
    add_line(sys, Pp(sprintf('MOD%d',m)).LConn(2), Pp(sprintf('P2S%d',m)).LConn(1));
    add_line(sys, sprintf('P2S%d/1',m), sprintf('soc%d/1',m));
end

% LOCAL SOLVER, fixed step, same as the plant. Without it Simscape wants a
% variable-step DAE solver and the FMU cannot carry one.
set_param([sys '/SC'],'UseLocalSolver','on', ...
          'LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER', ...
          'LocalSolverSampleTime',get_param(mdl,'FixedStep'),'DoFixedCost','on');

info = struct('Subsystem',sysName,'NumModules',nMod,'AH',AH,'Rmod',Rmod, ...
              'V0',V0,'SOCvec',socv,'Pack',K, ...
              'InPorts',{{'I_demand'}}, ...
              'OutPorts',{[{'v_pack'} arrayfun(@(m) sprintf('soc%d',m),1:nMod,'uni',0)]});
fprintf('  accumulator: %d modules of %ds%dp, %.1f A*h each, %.3f ohm each\n', ...
        nMod, ns, np, AH, Rmod);
end
