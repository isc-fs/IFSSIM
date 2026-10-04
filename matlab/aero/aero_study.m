function S = aero_study(varargin)
%AERO_STUDY  Change an aero parameter, see what it does. Without changing the car.
%
%   aero_study('ClA', 2.4, 'CdA', 1.1)          a bigger package
%   aero_study('AeroBalanceFront', 0.40)        move the centre of pressure back
%
%   Runs aero_kpis twice -- the car as built and with your change -- side by
%   side, with the lap and the balance overlaid in aero/figures/study/.
%   settings.json is NOT touched.
%
%   An override that moves none of these numbers is refused, the same as
%   vd_study and pt_study: two identical cars printed with confidence read
%   as an answer.
%
%   The plant reads every aero number at RUN time (IFSSIM_ClA, _CdA, _abal),
%   so plant_study takes these without a rebuild when the full plant is wanted.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if isempty(varargin) || mod(numel(varargin), 2)
    error('aero_study:usage', ['give parameter/value pairs, e.g.\n' ...
          '   aero_study(''ClA'', 2.4, ''CdA'', 1.1)']);
end

K0 = aero_kpis(ifssim_params());
K1 = aero_kpis(ifssim_params(varargin));

dead = {};
for i = 1:2:numel(varargin)
    Ki = aero_kpis(ifssim_params(varargin(i:i+1)));
    if isequal(vec(K0), vec(Ki)), dead{end+1} = varargin{i}; end %#ok<AGROW>
end
if ~isempty(dead)
    R = spec_reach(dead);
    L = {sprintf('%d of your overrides would not change anything this study measures:', numel(dead))};
    for k = 1:numel(dead)
        L{end+1} = sprintf('  %-22s %s', dead{k}, R.Reaches{k}); %#ok<AGROW>
    end
    L{end+1} = 'aero_parameters lists what moves what.';
    error('aero_study:noEffect', '%s', strjoin(L, newline));
end

P0 = ifssim_params();
fprintf('\n========================== AERO STUDY ==========================\n');
fprintf('  car: %s\n', ifssim_car());
for i = 1:2:numel(varargin)
    fprintf('  %-28s %g  ->  %g\n', varargin{i}, getdot(P0, varargin{i}), varargin{i+1});
end
fprintf('\n  %-36s %12s %12s %9s\n', '', 'as built', 'study', 'change');
row('lap [s]',                         K0.lap,        K1.lap);
row('  mean speed [km/h]',             K0.lap_mean_kmh, K1.lap_mean_kmh);
row('  pack energy [kJ/lap]',          K0.lap_E_kJ,   K1.lap_E_kJ);
row('  of which drag [kJ/lap]',        K0.lap_drag_kJ, K1.lap_drag_kJ);
row('cornering limit at 8 m/s [g]',    K0.aylim_8,    K1.aylim_8);
row('cornering limit at 22 m/s [g]',   K0.aylim_22,   K1.aylim_22);
row('F/R capacity at 8 m/s',           K0.ratio_8,    K1.ratio_8);
row('F/R capacity at 22 m/s',          K0.ratio_22,   K1.ratio_22);
row('75 m [s]',                        K0.t75,        K1.t75);
row('top speed [m/s]',                 K0.v_top,      K1.v_top);
fprintf('\n  first to saturate at 22 m/s: %s -> %s\n', K0.limits_22, K1.limits_22);
if K0.limits_22 ~= K1.limits_22
    fprintf('  THE LIMITING AXLE CHANGED. The car goes from limit %s to limit %s.\n', ...
            ust(K0.limits_22), ust(K1.limits_22));
end
fprintf('================================================================\n');
aero_plots(K0, K1, {'as built','study'}, fullfile(here,'figures','study'), false);
S = struct('baseline', rmfield(K0,{'G'}), 'study', rmfield(K1,{'G'}), 'overrides', {varargin});
end

% -------------------------------------------------------------------------
function x = vec(K)
x = round([K.lap K.aylim_8 K.aylim_22 K.ratio_8 K.ratio_22 K.t75 K.v_top K.lap_E_kJ], 9);
end
function s = ust(a), if a == "front", s = 'understeer'; else, s = 'oversteer'; end, end
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
