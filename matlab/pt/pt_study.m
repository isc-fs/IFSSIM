function S = pt_study(varargin)
%PT_STUDY  Change a powertrain parameter, see what it does. Without changing the car.
%
%   pt_study('Pack.CurrentLimit', 250)
%   pt_study('DrivetrainEfficiency', 0.95, 'MaxRegenPower', 12000)
%   pt_study('plant', 'Pack.CurrentLimit', 250)     % also run the full plant
%
%   Runs pt_model twice -- the car as built and the car with your change --
%   and prints both side by side, with the figures overlaid in
%   pt/figures/study/. settings.json is NOT touched.
%
%   AN OVERRIDE THAT DOES NOT BIND IS REFUSED, with what is binding instead.
%   On this car most of the motor's own numbers do not: pt_study('GearRatio',
%   3.5) would print two identical cars. That is a true answer, but a
%   side-by-side table of zeroes reads as "it doesn't matter", when the real
%   answer is "it doesn't matter BECAUSE something else limits first" -- and
%   which something is the useful part.
%
%   'plant' also runs the acceleration event on the full Simulink plant, with
%   and without the change (~2 min). It refuses what the plant cannot take
%   without a rebuild -- the tyre, and the accumulator's ARRANGEMENT, which
%   selects the generated pack; plant_study rebuilds for those. The cell's
%   data and Pack.CurrentLimit are runtime variables and run here directly.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'), fullfile(here,'..','vd'));

withPlant = ~isempty(varargin) && ischar(varargin{1}) && strcmpi(varargin{1}, 'plant');
if withPlant, varargin(1) = []; end
if isempty(varargin) || mod(numel(varargin), 2)
    error('pt_study:usage', ['give parameter/value pairs, e.g.\n' ...
          '   pt_study(''Pack.CurrentLimit'', 250)\n' ...
          'pt_parameters lists what binds on this car.']);
end

P0 = ifssim_params();
P1 = ifssim_params(varargin);
E0 = pt_model(P0);
E1 = pt_model(P1);

% ---- refuse what does not bind -------------------------------------------
dead = {};
for i = 1:2:numel(varargin)
    Ei = pt_model(ifssim_params(varargin(i:i+1)));
    if isequal(kpis(E0), kpis(Ei)), dead{end+1} = varargin{i}; end %#ok<AGROW>
end
if ~isempty(dead)
    error('pt_study:noEffect', '%s', explain(dead, E0));
end

% ---- refuse a plant run the plant cannot honour ------------------------
if withPlant
    % baked into blocks at BUILD time; plant_study knows how to rebuild for them
    BAKED = {'TireMu','Pacejka','Tyre','WheelRadius','WheelWidth','Mass','WeightDistFront', ...
             'Pack.CellsSeriesPerModule','Pack.CellsParallelPerModule', ...
             'Pack.ModulesInSeries','Pack.ModulesInParallel'};
    R = spec_reach(varargin(1:2:end));
    bad = {};
    for k = 1:height(R)
        nm = R.Parameter{k};
        if startsWith(R.Reaches{k}, 'NOT STUDYABLE')
            bad{end+1} = sprintf('  %-26s read from car_spec directly; no override reaches the plant', nm); %#ok<AGROW>
        elseif any(startsWith(nm, BAKED))
            bad{end+1} = sprintf('  %-26s fixed at build time (tyre or pack arrangement); use plant_study', nm); %#ok<AGROW>
        elseif strcmp(R.Plant{k}, '-')
            bad{end+1} = sprintf('  %-26s %s', nm, R.Reaches{k}); %#ok<AGROW>
        end
    end
    if ~isempty(bad)
        error('pt_study:plantCannot', '%s', strjoin([{'the plant cannot run this study:'}, bad, ...
              {'drop ''plant'' to run it on pt_model.'}], newline));
    end
end

% ---- side by side --------------------------------------------------------
fprintf('\n======================= POWERTRAIN STUDY =======================\n');
for i = 1:2:numel(varargin)
    fprintf('  %-28s %g  ->  %g\n', varargin{i}, getdot(P0, varargin{i}), varargin{i+1});
end
fprintf('\n  %-36s %12s %12s %9s\n', '', 'as built', 'study', 'change');
row('power at the shaft [kW]',       E0.shaft_kW,        E1.shaft_kW);
row('launch [g]',                    E0.launch_g,        E1.launch_g);
row('traction-limited up to [m/s]',  E0.v_traction_end,  E1.v_traction_end);
row(sprintf('%.0f m [s]', E0.s_accel), E0.t_accel,       E1.t_accel);
row('  speed there [km/h]',          3.6*E0.v_accel_end, 3.6*E1.v_accel_end);
row('top speed [m/s]',               E0.v_top,           E1.v_top);
row('  motor speed there [rpm]',     E0.top_rpm,         E1.top_rpm);
row('regen + drag at 10 m/s [g]',    E0.regen_g_at(10),  E1.regen_g_at(10));
row('regen + drag at 20 m/s [g]',    E0.regen_g_at(20),  E1.regen_g_at(20));
b0 = unique(E0.limit, 'stable');  b1 = unique(E1.limit, 'stable');
fprintf('\n  limits, low speed to high:  %s  ->  %s\n', strjoin(b0, ' / '), strjoin(b1, ' / '));
if ~isequal(b0, b1)
    fprintf('  THE BINDING LIMIT CHANGED. Parameters that did not bind before may now.\n');
end

S = struct('baseline', E0, 'study', E1, 'overrides', {varargin});

if withPlant
    A0 = accel_run(E0.s_accel);
    A1 = accel_run(E0.s_accel, 1, varargin);
    fprintf('\n  on the plant, no traction control:\n');
    row(sprintf('%.0f m [s]', E0.s_accel), A0.t_target, A1.t_target);
    row('  speed there [km/h]', A0.v_end_kmh, A1.v_end_kmh);
    S.plant = struct('baseline', A0, 'study', A1);
end
fprintf('================================================================\n');

pt_plots(E0, E1, {'as built', 'study'}, fullfile(here, 'figures', 'study'));
end

% -------------------------------------------------------------------------
function k = kpis(E)
k = round([E.t_accel, E.v_top, E.launch_g, E.regen_g_at(10), E.regen_g_at(20)], 9);
end

function msg = explain(dead, E0)
L = {sprintf(['%d of your overrides would not change anything this study ' ...
              'measures, so it would have compared two IDENTICAL cars:'], numel(dead)), ''};
R = spec_reach(dead);
for k = 1:numel(dead)
    L{end+1} = sprintf('  %s', dead{k}); %#ok<AGROW>
    if startsWith(R.Reaches{k}, 'NOTHING')
        L{end+1} = sprintf('    reaches: %s', R.Reaches{k}); %#ok<AGROW>
    else
        L{end+1} = '    it is wired in, but on this car it does not BIND:'; %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = sprintf('  what binds instead, low speed to high: %s', ...
                   strjoin(unique(E0.limit, 'stable'), ' -> '));
L{end+1} = sprintf('    traction up to %.1f m/s, then the pack at %.1f kW (the motor is rated higher)', ...
                   E0.v_traction_end, E0.shaft_kW);
L{end+1} = '    regen is power-limited (MaxRegenPower) from a few m/s up';
L{end+1} = '    there is no motor speed limit in the plant, so GearRatio has no top end to trade against';
L{end+1} = '  pt_parameters shows which parameters move what.';
msg = strjoin(L, newline);
end

function row(name, a, b)
if abs(a) > 1e-12, c = sprintf('%+8.2f%%', 100*(b-a)/abs(a)); else, c = '       -'; end
fprintf('  %-36s %12.3f %12.3f %9s\n', name, a, b, c);
end

function x = getdot(S, name)
parts = strsplit(name, '.');
if numel(parts) == 2 && strcmp(parts{1}, 'Tyre'), parts = {'Assumed', ['Tyre' parts{2}]}; end
x = S;
for k = 1:numel(parts), x = x.(parts{k}); end
end
