function S = tyre_study(varargin)
%TYRE_STUDY  Change a tyre parameter, see what it does. Without changing the car.
%
%   tyre_study('TireMu', 1.65)                  the tyres department's number
%   tyre_study('Tyre.LoadSensitivity', -0.4)    what a slick probably has
%   tyre_study('Pacejka.LatC', 1.6, 'Pacejka.LatE', 0)
%
%   Runs tyre_kpis twice, side by side, with the curves overlaid in
%   tyres/figures/study/. settings.json is NOT touched.
%
%   An override that moves none of these numbers is refused, the same as the
%   other departments' studies. For the tyre that includes the three inert
%   parameters, which is the point: Tyre.Pressure is declared, exported and
%   read, and does nothing while the pressure terms are zeroed.
%
%   On the full plant the tyre is baked in at BUILD time, so use plant_study,
%   which rebuilds the tyre set for you.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if isempty(varargin) || mod(numel(varargin), 2)
    error('tyre_study:usage', ['give parameter/value pairs, e.g.\n' ...
          '   tyre_study(''TireMu'', 1.65)']);
end

K0 = tyre_kpis(ifssim_params());
K1 = tyre_kpis(ifssim_params(varargin));

dead = {};
for i = 1:2:numel(varargin)
    Ki = tyre_kpis(ifssim_params(varargin(i:i+1)));
    if isequal(vec(K0), vec(Ki)), dead{end+1} = varargin{i}; end %#ok<AGROW>
end
if ~isempty(dead)
    R = spec_reach(dead);
    L = {sprintf('%d of your overrides would not change anything this study measures:', numel(dead))};
    for k = 1:numel(dead)
        L{end+1} = sprintf('  %-28s %s', dead{k}, R.Reaches{k}); %#ok<AGROW>
    end
    L{end+1} = 'tyre_parameters lists what moves what, and which parameters are inert on purpose.';
    error('tyre_study:noEffect', '%s', strjoin(L, newline));
end

P0 = ifssim_params();
fprintf('\n========================== TYRE STUDY ==========================\n');
for i = 1:2:numel(varargin)
    fprintf('  %-28s %g  ->  %g\n', varargin{i}, getdot(P0, varargin{i}), varargin{i+1});
end
fprintf('\n  %-36s %12s %12s %9s\n', '', 'as built', 'study', 'change');
row('peak mu at 1.5 Fz0',            K0.mu_1p5,     K1.mu_1p5);
row('cornering stiffness [N/deg]',   K0.Ca_nom,     K1.Ca_nom);
row('peak slip angle [deg]',         K0.alpha_peak, K1.alpha_peak);
row('lateral tail [fraction]',       K0.tail_lat,   K1.tail_lat);
row('lap [s]',                       K0.lap,        K1.lap);
row('skid pad [s]',                  K0.skidpad,    K1.skidpad);
row('limit at 22 m/s [g]',           K0.aylim_22,   K1.aylim_22);
row('understeer K [deg/g]',          K0.understeer, K1.understeer);
row('launch [g]',                    K0.launch_g,   K1.launch_g);
row('75 m [s]',                      K0.t75,        K1.t75);
if sign(K0.understeer) ~= sign(K1.understeer)
    fprintf('\n  THE CAR CHANGED SIDES: %s -> %s at low lateral.\n', side(K0.understeer), side(K1.understeer));
end
fprintf('================================================================\n');
tyre_plots(K0, K1, {'as built','study'}, fullfile(here,'figures','study'));
S = struct('baseline', rmfield(K0,{'G','M'}), 'study', rmfield(K1,{'G','M'}), 'overrides', {varargin});
end

% -------------------------------------------------------------------------
function x = vec(K)
x = round([K.mu_1p5 K.Ca_nom K.alpha_peak K.tail_lat K.lap K.skidpad K.aylim_22 ...
           K.understeer K.launch_g K.t75], 9);
end
function s = side(k), if k > 0, s = 'understeer'; else, s = 'oversteer'; end, end
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
