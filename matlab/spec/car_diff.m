function T = car_diff(a, b)
%CAR_DIFF  What is different between two prototypes, and how sure each is.
%
%   car_diff('IFS-08','IFS-09')
%
%   Lists every parameter whose VALUE differs, with both sources -- the
%   question a design review asks first. Then counts what the second car still
%   inherits, because "the same as last year" is only an answer if somebody
%   decided it, and INHERITED says nobody has yet.

addpath(fileparts(mfilename('fullpath')));
A = car_spec(a);  B = car_spec(b);
keys = unique([A.Order, B.Order], 'stable');
rows = cell(0, 5);
for i = 1:numel(keys)
    k = keys{i};
    inA = isfield(A.Fields, k);  inB = isfield(B.Fields, k);
    if inA && inB
        fa = A.Fields.(k);  fb = B.Fields.(k);
        if ~isequal(fa.value, fb.value)
            rows(end+1,:) = {fa.name, num2str(fa.value), num2str(fb.value), fa.unit, prov_class(fb.source)}; %#ok<AGROW>
        end
    elseif inA
        rows(end+1,:) = {A.Fields.(k).name, num2str(A.Fields.(k).value), '(absent)', A.Fields.(k).unit, '-'}; %#ok<AGROW>
    else
        rows(end+1,:) = {B.Fields.(k).name, '(absent)', num2str(B.Fields.(k).value), B.Fields.(k).unit, prov_class(B.Fields.(k).source)}; %#ok<AGROW>
    end
end

fprintf('\n=== %s -> %s ===\n', a, b);
if isempty(rows)
    fprintf('  no parameter differs.\n');
else
    fprintf('  %-28s %14s %14s %-8s %s\n', 'PARAMETER', a, b, 'UNIT', [b ' SOURCE']);
    for r = 1:size(rows,1)
        fprintf('  %-28s %14s %14s %-8s %s\n', rows{r,:});
    end
end
inh = sum(cellfun(@(k) strcmp(prov_class(B.Fields.(k).source), 'INHERITED'), B.Order));
fprintf('\n  %s: %d of %d parameters still INHERITED', b, inh, numel(B.Order));
if ~isempty(B.Parent), fprintf(' from %s', B.Parent); end
fprintf('.\n');
T = cell2table(rows, 'VariableNames', {'Parameter', matlab.lang.makeValidName(a), ...
               matlab.lang.makeValidName(b), 'Unit', 'Source'});
end
