function S = acc_study(varargin)
%ACC_STUDY  Change the pack, see what it does. Without changing the car.
%
%   acc_study('Pack.CellsParallelPerModule', 7)     a string more per module
%   acc_study('Cell.Rint', 0.013)                   if the AC figure were the DC one
%   acc_study('Pack.CurrentLimit', 250)
%
%   Runs acc_kpis twice, side by side, with the lap current and the endurance
%   overlaid in accumulator/figures/study/. The car's mass follows the
%   cells. settings.json is NOT touched.
%
%   An override that moves none of these numbers is refused, the same as the
%   other departments' studies.
%
%   THIS IS THE DESIGN-LEVEL STUDY. The plant's Simscape Battery pack takes
%   the same overrides: the cell's data and the current limit at run time,
%   the arrangement through plant_study, which rebuilds for it.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if isempty(varargin) || mod(numel(varargin), 2)
    error('acc_study:usage', ['give parameter/value pairs, e.g.\n' ...
          '   acc_study(''Pack.CellsParallelPerModule'', 7)']);
end

K0 = acc_kpis(ifssim_params());
K1 = acc_kpis(ifssim_params(varargin));

dead = {};
for i = 1:2:numel(varargin)
    Ki = acc_kpis(ifssim_params(varargin(i:i+1)));
    if isequal(vec(K0), vec(Ki)), dead{end+1} = varargin{i}; end %#ok<AGROW>
end
if ~isempty(dead)
    L = {sprintf('%d of your overrides would not change anything this study measures:', numel(dead))};
    for k = 1:numel(dead), L{end+1} = sprintf('  %s', dead{k}); end %#ok<AGROW>
    L{end+1} = 'acc_parameters says which bind, and which are ratings or labels.';
    error('acc_study:noEffect', '%s', strjoin(L, newline));
end

P0 = ifssim_params();
fprintf('\n======================= ACCUMULATOR STUDY =======================\n');
fprintf('  car: %s\n', ifssim_car());
for i = 1:2:numel(varargin)
    fprintf('  %-28s %g  ->  %g\n', varargin{i}, getdot(P0, varargin{i}), varargin{i+1});
end
if abs(K1.dMass) > 1e-9
    fprintf('  car mass follows the cells: %+.1f kg\n', K1.dMass);
end
fprintf('\n  %-36s %12s %12s %9s\n', '', 'as built', 'study', 'change');
row('pack energy [kWh]',             K0.E_kWh,      K1.E_kWh);
row('pack resistance [ohm]',         K0.R_pack,     K1.R_pack);
row('car mass [kg]',                 K0.m_car,      K1.m_car);
row('shaft power fresh [kW]',        K0.kW_fresh,   K1.kW_fresh);
row('shaft power at 25% SoC [kW]',   K0.kW_tired,   K1.kW_tired);
row('75 m [s]',                      K0.t75,        K1.t75);
row('first lap [s]',                 K0.lap_first,  K1.lap_first);
row('endurance laps completed',      K0.laps,       K1.laps);
row('charge left at the end [%]',    100*K0.soc_end, 100*K1.soc_end);
row('energy drawn [kWh]',            K0.E_used,     K1.E_used);
row('lowest cell voltage [V]',       K0.V_cell_min, K1.V_cell_min);
row('rms cell current [A]',          K0.I_cell_rms, K1.I_cell_rms);
row('cell heating, no cooling [K]',  K0.dT_cell,    K1.dT_cell);
fprintf('\n  finishes the endurance: %s -> %s\n', yn(K0.finishes), yn(K1.finishes));
fprintf('================================================================\n');
acc_plots(K0, K1, {'as built','study'}, fullfile(here,'figures','study'));
S = struct('baseline', rmfield(K0,{'P','PK'}), 'study', rmfield(K1,{'P','PK'}), 'overrides', {varargin});
end

% -------------------------------------------------------------------------
function x = vec(K)
x = round([K.laps K.E_used K.V_cell_min K.I_cell_rms K.dT_cell K.kW_fresh ...
           K.t75 K.lap_first K.m_car K.soc_end], 9);
end
function s = yn(b), if b, s = 'yes'; else, s = 'NO'; end, end
function row(name, a, b)
if abs(a) > 1e-12, c = sprintf('%+8.2f%%', 100*(b-a)/abs(a)); else, c = '       -'; end
fprintf('  %-36s %12.3f %12.3f %9s\n', name, a, b, c);
end
function x = getdot(S, name)
parts = strsplit(name, '.');
x = S;
for k = 1:numel(parts), x = x.(parts{k}); end
end
