function T = acc_parameters()
%ACC_PARAMETERS  The accumulator department's parameters, and what each one moves.
%
%       cd matlab/accumulator
%       acc_parameters
%
%   Same two computed columns as the other departments. Counts step by ONE
%   (a cell, a string, a module) rather than 10%, because 1.9 cells is not a
%   design. The car's mass follows the cells (acc_kpis), so a bigger pack is
%   charged for its weight.
%
%   Two kinds of parameter here bind nothing BY DESIGN, and the table should
%   say so: the cell's RATINGS (ICharacterised, ISustained) are what the
%   report compares the currents against, not physics; and VNom is a label.
%   VMax is the interesting one -- see the note it prints.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
C = car_spec();
v = @(n) C.Fields.(strrep(n,'.','_')).value;

SPEC = {
'Cell.CapacityAh'            'cell'     'charge per cell: range'
'Cell.Rint'                  'cell'     'sag and heat, both as I^2 -- the loss that grows fastest'
'Cell.Mass'                  'cell'     'pack mass, so car mass; and thermal mass'
'Cell.SpecificHeat'          'cell'     'how fast a cell heats for the same loss'
'Cell.VMin'                  'cell'     'the floor: when the event ends'
'Cell.VMax'                  'cell'     'top of charge -- but see the note'
'Cell.VNom'                  'label'    'a nominal voltage; computes nothing'
'Cell.ICharacterised'        'rating'   'compared against, not simulated'
'Cell.ISustained'            'rating'   'compared against, not simulated'
'Pack.CellsSeriesPerModule'  'topology' 'voltage, so power at the current limit (+1 cell)'
'Pack.ModulesInSeries'       'topology' 'voltage, in steps of a whole module (+1 module)'
'Pack.CellsParallelPerModule' 'topology' 'capacity and current sharing (+1 string)'
'Pack.ModulesInParallel'     'topology' 'doubles everything parallel (+1)'
'Pack.CurrentLimit'          'limit'    'THE power limit of this car'
};
n = size(SPEC,1);
steps = struct('Pack_CellsSeriesPerModule',  v('Pack.CellsSeriesPerModule') + 1, ...
               'Pack_ModulesInSeries',       v('Pack.ModulesInSeries') + 1, ...
               'Pack_CellsParallelPerModule', v('Pack.CellsParallelPerModule') + 1, ...
               'Pack_ModulesInParallel',     v('Pack.ModulesInParallel') + 1);

KPI = {'laps','laps'; 'E_used','E'; 'V_cell_min','Vmin'; 'I_cell_rms','Irms'; ...
       'dT_cell','dT'; 'kW_fresh','kW'; 't75','75 m'; 'lap_first','lap'; 'm_car','mass'};
K0 = acc_kpis();
R  = spec_reach(SPEC(:,1));
moves = dept_moves(SPEC(:,1), @acc_kpis, KPI, K0, steps);

fprintf('\n========================= ACCUMULATOR ========================\n');
fprintf('  %d s %d p, %.0f V max, %.2f kWh, %.1f kg of cells, %.3f ohm\n', ...
        K0.Ns, K0.Np, K0.V_max, K0.E_kWh, K0.m_cells, K0.R_pack);
fprintf('\n  %-27s %8s %-5s %-10s %-8s %s\n','PARAMETER','VALUE','UNIT','PROVENANCE','ROLE','STEP MOVES (+10%, or +1 for counts)');
for i = 1:n
    f = C.Fields.(strrep(SPEC{i,1},'.','_'));
    fprintf('  %-27s %8.4g %-5s %-10s %-8s %s\n', SPEC{i,1}, f.value, f.unit, ...
            provclass(f.source), SPEC{i,2}, moves{i});
end
fprintf(['\n  laps = endurance laps completed (of %.1f); E = energy drawn; Vmin = lowest\n' ...
         '  cell voltage; Irms = rms cell current; dT = cell heating with NO cooling\n'], K0.R.laps_needed);

fprintf('\n---- where each one reaches (spec_reach) ----------------------\n');
for i = 1:n
    fprintf('  %-27s plant: %s\n', SPEC{i,1}, R.Plant{i});
end
fprintf('  The plant''s pack is Simscape Battery, generated from these: the cell and\n');
fprintf('  the current limit are read at run time, the arrangement at build time.\n');

fprintf('\n---- READ THIS BEFORE TRUSTING A RESULT -----------------------\n');
iV = strcmp(SPEC(:,1), 'Cell.VMax');
if strcmp(moves{iV}, 'DOES NOT BIND')
    fprintf('  Cell.VMax DOES NOT BIND. The voltage the pack actually has comes from\n');
    fprintf('  the OCV table (Cell.OCV_V), whose top entry is %.2f V -- the same\n', C.Fields.Cell_OCV_V.value(end));
    fprintf('  number, typed twice. Change one and the other does not follow.\n\n');
end
fprintf('  The endurance is run FLAT OUT, every lap, no cooling: an upper bound on\n');
fprintf('  energy, current and heat. On it this pack %s (%.1f of %.1f laps, %.0f%%\n', ...
        tern(K0.finishes, 'FINISHES', 'DOES NOT FINISH'), K0.laps, K0.R.laps_needed, 100*K0.soc_end);
fprintf('  charge left). A driven endurance is slower and cheaper.\n\n');
fprintf('  The car''s mass follows the cells (%.1f kg here); housings, busbars and\n', K0.m_cells);
fprintf('  cooling do not, so a bigger pack is still under-charged for its weight.\n');
fprintf('==============================================================\n');
fprintf('  acc_report      the pack, the endurance, and the ratings it meets\n');
fprintf('  acc_plots       the same, as figures\n');
fprintf('  acc_study(...)  try a change\n\n');

T = table(SPEC(:,1), SPEC(:,2), moves, R.Reaches, ...
          'VariableNames', {'Parameter','Role','Moves','Reaches'});
end

function c = provclass(src)
w = upper(strtok(src));
known = {'MEASURED','MEASURED-ISH','GEOMETRY','DERIVED','DATASHEET', ...
         'SECONDARY','ASSUMED','DISPUTED','ZEROED','UNKNOWN'};
if any(strcmp(w, known)), c = w; else, c = 'UNKNOWN'; end
if strcmp(c,'MEASURED-ISH'), c = 'MEASURED~'; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
