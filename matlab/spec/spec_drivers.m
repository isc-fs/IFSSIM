function names = spec_drivers(path)
%SPEC_DRIVERS  Which car_spec parameters actually move a given field.
%
%   NAMES = SPEC_DRIVERS('Derived.RollStiffnessFront') returns every car_spec
%   parameter that, when changed, changes P.Derived.RollStiffnessFront.
%
%   It answers the question a refused study leaves you with: "fine, that knob
%   is dead -- so which one isn't?" RollStiffnessFront is declared in
%   car_spec but derived from the springs and the anti-roll bar, so
%   overriding the declared total does nothing. The knobs that DO move it are
%   found here by perturbation, rather than listed by hand in a hint table
%   that would go stale the next time the derivation changes -- which is
%   exactly how RollStiffnessFront became dead in the first place.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'));

C  = car_spec();
P0 = ifssim_params();
target0 = getpath(P0, path);
if isempty(target0)
    error('spec_drivers:nofield', 'P.%s does not exist.', path);
end

names = {};
for i = 1:numel(C.Order)
    f = C.Fields.(C.Order{i});
    if ~(isnumeric(f.value) && isscalar(f.value)), continue; end
    if f.value == 0, v1 = 0.1; else, v1 = f.value * 1.1; end
    try
        P1 = ifssim_params({f.name, v1});
    catch
        continue
    end
    t1 = getpath(P1, path);
    if ~isempty(t1) && any(abs(t1(:) - target0(:)) > 1e-12 * max(1, max(abs(target0(:)))))
        names{end+1} = f.name; %#ok<AGROW>
    end
end
end

function v = getpath(s, path)
v = s;
for k = strsplit(path, '.')
    if isstruct(v) && isfield(v, k{1}), v = v.(k{1}); else, v = []; return; end
end
if ~isnumeric(v), v = []; end
end
