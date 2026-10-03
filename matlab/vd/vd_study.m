function S = vd_study(varargin)
%VD_STUDY  Change a parameter, see what it does. Without changing the car.
%
%   vd_study('TireMu', 1.65)
%   vd_study('CoGHeight', 0.3441, 'Assumed.Izz', 200)
%   vd_study('Susp.ArbRateFront', 17467, 'Susp.ArbRateRear', 2467)
%
%   An override that would not change the design model is REFUSED, with the
%   parameters that would.
%
%   Runs the characterisation twice -- the car as built, and the car with your
%   change -- and prints both side by side. settings.json is NOT touched, so
%   nothing you do here can affect the simulator or anybody else's run.
%
%   Overrides go in before anything is derived, so the whole chain follows.
%   Raise HeaveStiffness and the ride frequency, the damper coefficient and
%   the anti-roll bar rates all move with it; you cannot accidentally produce
%   a car whose numbers disagree with each other.
%
%   TO MAKE A CHANGE REAL, once you are happy with it:
%     1. edit the number in matlab/spec/car_spec.m, with a source for it
%     2. run build_car   (writes settings.json, so the simulator agrees)
%     3. run vd_report
%
%   Names are any parameter in ifssim_params: top-level ones like TireMu or
%   CoGHeight, Pacejka.LatC, or Assumed.Izz. Run  ifssim_params_report  to see
%   the full list with where each number came from.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','plant'));

if isempty(varargin)
    error('vd_study:noChange', ...
          ['nothing to study. Give a parameter and a value, e.g.\n' ...
           '   vd_study(''TireMu'', 1.65)']);
end

P0 = ifssim_params();
P1 = ifssim_params(varargin);
M0 = dualtrack_build(P0);
M1 = dualtrack_build(P1);

% REFUSE AN OVERRIDE THAT REACHES NOTHING THIS STUDY RUNS.
%
% This study runs the DESIGN model. An override that does not change that
% model produces a side-by-side of two identical cars, printed with full
% confidence -- worse than an error, because it reads as an answer. That is
% not hypothetical: this README's own headline example,
%   vd_study('RollStiffnessFront', 32000, 'RollStiffnessRear', 17000)
% did exactly that for as long as roll stiffness has been DERIVED from the
% springs and the bar, because the declared total it overrides feeds nothing.
%
% Each override is tested ALONE, so a dead one cannot hide behind a live one
% in the same call: vd_study('TireMu',1.65,'RollStiffnessFront',32000) changes
% the model, and the second half of it still does nothing.
addpath(fullfile(here,'..','spec'));
dead = {};
for i = 1:2:numel(varargin)
    Mi = dualtrack_build(ifssim_params(varargin(i:i+1)));
    if models_equal(M0, Mi), dead{end+1} = varargin{i}; end %#ok<AGROW>
end
if ~isempty(dead)
    error('vd_study:noEffect', '%s', explain_dead(dead));
end

fprintf('\n================== STUDY ==================\n');
for i = 1:2:numel(varargin)
    key = varargin{i};
    was = getdot(P0, key);
    now = getdot(P1, key);
    if isempty(was)
        fprintf('  %-28s   (new)  ->  %g\n', key, now);
    else
        fprintf('  %-28s  %g  ->  %g\n', key, was, now);
    end
end

A = characterise(M0);
B = characterise(M1);
S = struct('baseline',A,'study',B,'overrides',{varargin});

fprintf('\n  %-34s %12s %12s %10s\n', '', 'as built', 'study', 'change');
row('understeer gradient K [deg/g]', A.K, B.K, '%+12.3f');
row('skid pad lap [s]',              A.lap, B.lap, '%12.2f');
row('skid pad speed [m/s]',          A.vmax, B.vmax, '%12.2f');
row('max lateral [g]',               A.aymax/9.81, B.aymax/9.81, '%12.2f');
row('step response t90 [s]',         A.t90, B.t90, '%12.3f');
row('ride frequency [Hz]',           P0.Derived.RideFreqHz, P1.Derived.RideFreqHz, '%12.2f');
row('roll stiffness front [%]',      100*M0.KrF/(M0.KrF+M0.KrR), ...
                                     100*M1.KrF/(M1.KrF+M1.KrR), '%12.1f');

if A.K < 0 && B.K >= 0
    fprintf('\n  This change turns the car from OVERSTEERING to understeering.\n');
elseif A.K >= 0 && B.K < 0
    fprintf('\n  This change turns the car from understeering to OVERSTEERING.\n');
end
fprintf('===========================================\n');

vd_plots(M1, describe(varargin), fullfile(here,'figures','study'));
fprintf('  baseline figures are in figures/, this study''s in figures/study/\n\n');
end

% -----------------------------------------------------------------------
function A = characterise(M)
C = vd_constant_radius(9.125, M, 4:0.5:18);
A = struct('K',NaN,'lap',NaN,'vmax',NaN,'aymax',NaN,'t90',NaN);
if ~isempty(C.v)
    A.K = C.K;  A.vmax = C.v(end);  A.aymax = C.ay(end);
    A.lap = 2*pi*9.125/C.v(end);
end
d = steer_for(12, 0.4*9.81, M);
if ~isnan(d)
    S = vd_step_steer(12, d, M);  A.t90 = S.t90;
end
end

function row(name, a, b, fmt)
fprintf(['  %-34s ' fmt ' ' fmt ' %+9.1f%%\n'], name, a, b, 100*(b-a)/max(abs(a),eps));
end

function v = getdot(P, key)
parts = split(key,'.');  v = [];
try
    v = P.(parts{1});
    for i = 2:numel(parts), v = v.(parts{i}); end
catch
end
end

function s = describe(args)
p = cell(1,numel(args)/2);
for i = 1:2:numel(args), p{(i+1)/2} = sprintf('%s = %g', args{i}, args{i+1}); end
s = strjoin(p, ', ');
end

function d = steer_for(v, ayWant, M)
d = NaN;  lo = 1e-4;  alo = ay_at(lo, v, M);
for hi = linspace(0.01, M.maxSteer, 60)
    ahi = ay_at(hi, v, M);
    if ahi >= ayWant
        for it = 1:40
            mid = 0.5*(lo+hi);
            if ay_at(mid, v, M) < ayWant, lo = mid; else, hi = mid; end
        end
        d = 0.5*(lo+hi); return;
    end
    if ahi < alo, return; end
    lo = hi;  alo = ahi;
end
end

function a = ay_at(d, v, M)
S = dualtrack_trim(v, d, M);  a = S.ay;
end

% -------------------------------------------------------------------------
function tf = models_equal(A, B)
tf = true;
for f = fieldnames(A)'
    a = A.(f{1});  b = B.(f{1});
    if isnumeric(a) && isnumeric(b) && isequal(size(a), size(b))
        if any(abs(a(:) - b(:)) > 1e-12 * max(1, max(abs(a(:))))), tf = false; return; end
    end
end
end

function msg = explain_dead(dead)
L = {sprintf(['%d of your overrides would not change the design model this ' ...
              'study runs, so it would have compared two IDENTICAL cars:'], numel(dead)), ''};
R = spec_reach(dead);
for k = 1:numel(dead)
    nm = dead{k};
    L{end+1} = sprintf('  %s', nm); %#ok<AGROW>
    L{end+1} = sprintf('    reaches: %s', R.Reaches{k}); %#ok<AGROW>
    if ~strcmp(R.Plant{k}, '-')
        L{end+1} = sprintf(['    it does reach the PLANT (%s) -- use plant_study, ' ...
                            'or the department view, not vd_study'], R.Plant{k}); %#ok<AGROW>
    end
    % A declared value with a DERIVED twin is the common case: the derivation
    % moved and the declared input was left behind. Name the inputs that do
    % drive it, computed rather than looked up in a table that would go stale.
    leaf = regexprep(nm, '^.*\.', '');
    try
        drv = spec_drivers(['Derived.' leaf]);
        drv = setdiff(drv, {nm}, 'stable');
        if ~isempty(drv)
            L{end+1} = sprintf('    P.Derived.%s is what the models read, and it is driven by:', leaf); %#ok<AGROW>
            L{end+1} = sprintf('      %s', strjoin(drv, ', ')); %#ok<AGROW>
        end
    catch
    end
end
msg = strjoin(L, newline);
end