function [moves, K0] = dept_moves(names, kpiFn, KPI, K0)
%DEPT_MOVES  For each parameter: move it 10%, report which KPIs moved and how much.
%
%   moves = DEPT_MOVES(names, kpiFn, KPI)
%     names   car_spec parameter names
%     kpiFn   @(P) -> struct of scalar KPIs, P from ifssim_params(overrides)
%     KPI     {field, short label; ...} -- which fields to report
%
%   Returns one string per parameter: 'lap -0.28%, ay@22 +1.35%' or
%   'DOES NOT BIND'. The shared engine behind every department's
%   *_parameters "+10% MOVES" column, so the departments cannot come to
%   disagree about what "moves" means.
%
%   A zero-valued parameter is moved to 0.1 instead (10% of zero is zero,
%   and would wrongly read as "does not bind"). A change under 0.05% of the
%   KPI is treated as no change.

C = car_spec();
if nargin < 4 || isempty(K0), K0 = kpiFn(ifssim_params()); end
moves = cell(numel(names), 1);
for i = 1:numel(names)
    nm = names{i};
    v0 = C.Fields.(strrep(nm,'.','_')).value;
    if v0 == 0, v1 = 0.1; else, v1 = 1.1*v0; end
    K1 = kpiFn(ifssim_params({nm, v1}));
    c = {};
    for j = 1:size(KPI,1)
        a = K0.(KPI{j,1});  b = K1.(KPI{j,1});
        if abs(b - a) > 5e-4*max(abs(a), 1e-9)
            c{end+1} = sprintf('%s %+.2f%%', KPI{j,2}, 100*(b-a)/abs(a)); %#ok<AGROW>
        end
    end
    if isempty(c), moves{i} = 'DOES NOT BIND'; else, moves{i} = strjoin(c, ', '); end
end
end
