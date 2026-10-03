function info = build_battery_pack(mdl, sysName, pos, P)
%BUILD_BATTERY_PACK  The accumulator, as a Simscape Battery pack built from the spec.
%
%   info = BUILD_BATTERY_PACK(mdl, sysName, pos)      the car_spec pack
%   info = BUILD_BATTERY_PACK(mdl, sysName, pos, P)   P = ifssim_params(overrides)
%
%   Adds a subsystem to mdl holding the accumulator, and returns its
%   arrangement. The subsystem takes a current demand in amps and returns
%   pack terminal voltage (port 1) and each module's state of charge (ports
%   2..NModules+1).
%
%   GENERATED, NOT HAND-WIRED. The pack is built by Simscape Battery's
%   builder (battery_from_spec -> battery_library -> buildBattery) from
%   car_spec's cell and four topology integers. The hand-wired version this
%   replaced placed one Table-Based block per module and chained them in
%   series in a loop; it could not represent strings in parallel (it wired
%   Pack.ModulesInParallel > 1 in series regardless), and any other
%   arrangement meant editing this file. A new accumulator is now a new set
%   of numbers in car_spec. The cell model is the same Table-Based battery,
%   lumped per module, so for the IFS-08 the physics is unchanged.
%
%   THE CELL'S DATA ARE WORKSPACE VARIABLES, not numbers written into blocks:
%   IFSSIM_cell_SOC/V0/R0/AH and IFSSIM_SoC0, assigned by
%   ifssim_load_workspace from P. So a different cell, or a study override
%   of one, reaches the plant at the next run with no rebuild. Only a change
%   of ARRANGEMENT needs one (it is a different generated library).
%
%   Three things the generated pack does that are not obvious, all found by
%   driving it against the analytic answer V = Ns*OCV(soc) - I*R:
%
%   1. ITS PLACEHOLDER CELL IS NOT OURS. Generated modules carry a default
%      27 Ah, 8.5 mOhm cell. Every module's cell data are overwritten here.
%   2. IT STARTS FULL, then asserts every step that charge cannot exceed 1.
%      Setting socCell alone does nothing: socCell_specify must be on too.
%   3. ITS TERMINALS ARE NAMED BACKWARDS FOR US. Wired by name -- '+' to the
%      load -- a discharge demand CHARGES the pack and it reads -416 V. The
%      terminal named '-' is the one that goes to the current source.
%      test_battery_pack holds the polarity, so a regenerated library that
%      fixes the names breaks a test, not the car.
%
%   See also BATTERY_FROM_SPEC, BATTERY_LIBRARY, TEST_BATTERY_PACK, PACK_FROM_CELLS.

if nargin < 3 || isempty(pos), pos = [300 400 460 520]; end
if nargin < 4 || isempty(P), P = ifssim_params(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','spec'));
[~, libBlock, B] = battery_library(P);

sys = [mdl '/' sysName];
add_block('simulink/Ports & Subsystems/Subsystem', sys, 'Position', pos);
delete_line(sys,'In1/1','Out1/1'); delete_block([sys '/In1']); delete_block([sys '/Out1']);

SOLVER = sprintf('nesl_utility/Solver\nConfiguration');
PS2S   = sprintf('nesl_utility/PS-Simulink\nConverter');
S2PS   = sprintf('nesl_utility/Simulink-PS\nConverter');

% The pack, with its library link BROKEN: the plant then carries the pack's
% structure itself and needs only the generated Simscape package on the path
% (ifssim_load_workspace adds it), not the library model as well.
add_block(libBlock, [sys '/PACK'], 'Position', [320 40 460 300]);
set_param([sys '/PACK'], 'LinkStatus', 'none');

% 1 and 2: our cell, and a start below full.
mods = find_system([sys '/PACK'], 'LookUnderMasks','all', 'BlockType','SimscapeBlock');
for k = 1:numel(mods)
    set_param(mods{k}, 'SOC_vecCell','IFSSIM_cell_SOC', 'V0_vecCell','IFSSIM_cell_V0', ...
              'R0_vecCell','IFSSIM_cell_R0', 'AHCell','IFSSIM_cell_AH', ...
              'socCell_specify','on', 'socCell','IFSSIM_SoC0', 'socCell_priority','High');
end
if numel(mods) ~= B.NModules
    error('build_battery_pack:modules', 'generated pack has %d module blocks, expected %d', ...
          numel(mods), B.NModules);
end

add_block('simulink/Sources/In1',[sys '/I_demand'],'Position',[30 360 60 380]);
add_block(S2PS,[sys '/S2PS'],'Position',[110 355 150 385]);
add_block('fl_lib/Electrical/Electrical Sources/Controlled Current Source', ...
          [sys '/ISRC'],'Position',[210 340 270 420]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', ...
          [sys '/GND'],'Position',[220 520 260 560]);
add_block(SOLVER,[sys '/SC'],'Position',[60 520 120 560]);
add_block('fl_lib/Electrical/Electrical Sensors/Voltage Sensor',[sys '/VS'],'Position',[520 340 580 420]);
add_block(PS2S,[sys '/P2Sv'],'Position',[620 360 660 390]);
add_block('simulink/Sinks/Out1',[sys '/v_pack'],'Position',[700 365 730 385],'Port','1');

Pp = @(b) get_param([sys '/' b],'PortHandles');
% 3: the terminals, by name, reversed.
[posT, negT] = pack_terminals([sys '/PACK']);

% Current source and voltage sensor, port layout as established in the
% hand-wired version: LConn(1) terminal, RConn(1) physical signal, RConn(2)
% terminal; the sensor's positive side is RConn(2).
add_line(sys,'I_demand/1','S2PS/1');
add_line(sys, Pp('S2PS').RConn(1), Pp('ISRC').RConn(1));
add_line(sys, negT, Pp('GND').LConn(1));
add_line(sys, posT, Pp('ISRC').LConn(1));
add_line(sys, Pp('ISRC').RConn(2), Pp('GND').LConn(1));
add_line(sys, Pp('VS').RConn(2), posT);
add_line(sys, Pp('VS').LConn(1), Pp('GND').LConn(1));
add_line(sys, Pp('VS').RConn(1), Pp('P2Sv').LConn(1));
add_line(sys, 'P2Sv/1', 'v_pack/1');
add_line(sys, Pp('SC').RConn(1), Pp('GND').LConn(1));

% Outputs. Per-module state of charge out, one port each, pinned to ports
% 2..N+1 (left implicit, a reader of 'Accumulator/1' gets a state of charge
% where it expects a voltage). The rest -- cell current, cell voltage,
% per-assembly values, cycle counts -- are what a BMS or a thermal model
% will want; they are terminated here, not deleted, so they are one line away.
outs = find_system([sys '/PACK'],'SearchDepth',1,'BlockType','Outport');
onames = get_param(outs,'Name');
for k = 1:numel(outs)
    pnum = str2double(get_param(outs{k},'Port'));
    if strcmp(onames{k}, 'socCell')
        add_block('simulink/Signal Routing/Demux',[sys '/socDemux'],'Outputs',num2str(B.NModules), ...
                  'Position',[520 60 525 60+30*B.NModules]);
        add_line(sys, sprintf('PACK/%d', pnum), 'socDemux/1');
        for m = 1:B.NModules
            add_block('simulink/Sinks/Out1', sprintf('%s/soc%d',sys,m), ...
                      'Position',[600 60+30*(m-1) 630 80+30*(m-1)], 'Port', num2str(m+1));
            add_line(sys, sprintf('socDemux/%d',m), sprintf('soc%d/1',m));
        end
    else
        t = sprintf('%s_unused', onames{k});
        add_block('simulink/Sinks/Terminator',[sys '/' t],'Position',[520 320+30*k 540 340+30*k]);
        add_line(sys, sprintf('PACK/%d', pnum), [t '/1']);
    end
end
if ~any(strcmp(onames,'socCell'))
    error('build_battery_pack:soc','generated pack has no socCell output');
end

% LOCAL SOLVER, fixed step, same as the plant: the FMU cannot carry
% Simscape's default variable-step DAE solver.
set_param([sys '/SC'],'UseLocalSolver','on', ...
          'LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER', ...
          'LocalSolverSampleTime',get_param(mdl,'FixedStep'),'DoFixedCost','on');

info = struct('Subsystem',sysName,'NumModules',B.NModules,'Library',B.LibraryName, ...
              'Np',B.Np,'Ns',B.Ns,'NModSeries',B.NModSeries,'NModParallel',B.NModParallel, ...
              'VFull', B.SeriesCells*P.Cell.OCV_V(end), ...
              'InPorts',{{'I_demand'}}, ...
              'OutPorts',{[{'v_pack'} arrayfun(@(m) sprintf('soc%d',m),1:B.NModules,'uni',0)]});
fprintf('  accumulator: %s -- %d string(s) of %d modules, %ds%dp each (Simscape Battery, lumped)\n', ...
        B.LibraryName, B.NModParallel, B.NModSeries, B.Ns, B.Np);
end

% -------------------------------------------------------------------------
function [posT, negT] = pack_terminals(blk)
%PACK_TERMINALS  The pack's electrically positive and negative terminals.
%   By the PMIO port NAMES, then swapped -- see note 3 in the header.
ph = get_param(blk, 'PortHandles');
io = find_system(blk, 'SearchDepth', 1, 'BlockType', 'PMIOPort');
h = struct();
for k = 1:numel(io)
    side = get_param(io{k}, 'Side');
    left  = find_system(blk, 'SearchDepth',1, 'BlockType','PMIOPort', 'Side', side);
    nums  = cellfun(@(x) str2double(get_param(x,'Port')), left);
    idx   = find(sort(nums) == str2double(get_param(io{k},'Port')));
    if strcmp(side, 'Left'), handle = ph.LConn(idx); else, handle = ph.RConn(idx); end
    h.(matlab.lang.makeValidName(get_param(io{k},'Name'), 'ReplacementStyle','hex')) = handle;
end
names = fieldnames(h);
plusName  = names{contains(names, '0x2B')};    % '+'
minusName = names{contains(names, '0x2D')};    % '-'
posT = h.(minusName);       % named '-', electrically positive for our wiring
negT = h.(plusName);
end
