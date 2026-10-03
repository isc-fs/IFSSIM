function T = spec_reach(names)
%SPEC_REACH  Where each car_spec parameter ACTUALLY reaches. Computed, not claimed.
%
%   T = SPEC_REACH()        every numeric parameter in car_spec
%   T = SPEC_REACH(names)   just these, e.g. {'GearRatio','CdA','Cell.Capacity'}
%
%   A parameter can be declared in car_spec, documented, exported -- and read
%   by nothing at all. Changing it then does exactly nothing, and a study run
%   on it returns a confident comparison of two identical cars. The department
%   views have to say which parameters are live, and saying it by hand is how
%   it goes stale: vd_parameters printed "PitchStiffness reaches NOTHING" as a
%   sentence, true when written and checked by nothing since.
%
%   METHOD. For each parameter:
%     1. perturb it, through the same override path vd_study uses, so the whole
%        derivation chain in ifssim_params follows;
%     2. diff the entire parameter tree before and after -- every Derived and
%        Assumed field that moved is something this parameter feeds;
%     3. trace those fields to their readers.
%
%   The plant reads parameters TWO ways, and both are traced:
%     build time  build_*.m writes P.<field> into a block via set_param;
%     run time    ifssim_load_workspace exports P.<field> as an IFSSIM_* base
%                 variable, and a MATLAB Function block inside the plant reads
%                 that name. Missing this hop would report most of the chassis
%                 and suspension as unreachable when they are not.
%
%   WIRED IS NOT THE SAME AS STUDYABLE, and the difference is the one that
%   matters to a department. build_battery_pack reads car_spec() directly,
%   through a string-keyed accessor, rather than the override-able
%   P = ifssim_params(overrides). The accumulator is genuinely wired into the
%   plant -- and no study override can ever reach it. So there are three
%   verdicts, not two:
%     studyable          reached through P; a study override moves it
%     NOT STUDYABLE      read from car_spec directly; real, but overrides
%                        never arrive, so a study compares two identical cars
%     NOTHING            nothing reads it at all
%
%   LIMITS, so the table is not over-trusted. It is a static trace, with
%   comments stripped, that follows ONE level of calls into project helpers. A field read by dynamic name -- P.(name) -- is invisible
%   to it, and would show as "not reached" when it is. A field that is read
%   but then multiplied by zero shows as reached when its effect is nil. It
%   answers "is anything wired to this?", not "how much does it matter?" --
%   for that, run the study.

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
addpath(here, fullfile(root,'plant'));

C  = car_spec();
P0 = ifssim_params();

if nargin < 1 || isempty(names)
    names = {};
    for i = 1:numel(C.Order)
        f = C.Fields.(C.Order{i});
        if isnumeric(f.value) && isscalar(f.value), names{end+1} = f.name; end %#ok<AGROW>
    end
end

% ---- consumers ---------------------------------------------------------
design = read_code(fullfile(root,'vd','dualtrack_build.m'));
bf = dir(fullfile(root,'plant','build_*.m'));
% Project functions a builder might call. A builder that hands car_spec to a
% helper -- build_battery_pack does pack_from_cells(car_spec()) -- reads
% every field that helper reads, and tracing the builder alone misses all of
% it: the first version of this reported 8 accumulator fields as reaching
% NOTHING when pack_from_cells consumed every one of them.
helpers = [dir(fullfile(root,'spec','*.m')); dir(fullfile(root,'plant','*.m'))];
hname = erase({helpers.name}, '.m');
builders = struct('name',{},'code',{});
for k = 1:numel(bf)
    code = read_code(fullfile(bf(k).folder, bf(k).name));
    self = erase(bf(k).name,'.m');
    % one level: inline the source of any project function this builder calls
    for h = 1:numel(hname)
        if strcmp(hname{h}, self) || startsWith(hname{h}, 'build_') || ...
           any(strcmp(hname{h}, {'ifssim_params','ifssim_load_workspace','car_spec','spec_reach'}))
            continue
        end
        if ~isempty(regexp(code, ['(?<![\w.])' hname{h} '\s*\('], 'once'))
            code = [code newline read_code(fullfile(helpers(h).folder, helpers(h).name))]; %#ok<AGROW>
        end
    end
    builders(end+1) = struct('name', self, 'code', code); %#ok<AGROW>
end
wsmap = workspace_map(fullfile(root,'plant','ifssim_load_workspace.m'));
% Consumers that read car_spec() themselves, bypassing the override path.
specReaders = struct('name',{},'code',{});
for b = builders
    if contains(b.code, 'car_spec(')
        specReaders(end+1) = b; %#ok<AGROW>
    end
end

n = numel(names);
out = cell(n, 5);
for i = 1:n
    nm = names{i};
    v0 = spec_value(C, nm);
    v1 = perturb(v0);
    try
        P1 = ifssim_params({nm, v1});
    catch ME
        out(i,:) = {nm, 0, '-', '-', ['override refused: ' ME.message]};
        continue
    end
    moved = diff_tree(P0, P1, '');

    inDesign = any(cellfun(@(p) reads(design, p), moved));

    hits = {};
    for b = builders
        direct  = any(cellfun(@(p) reads(b.code, p), moved));
        runtime = false;
        for w = 1:size(wsmap,1)
            if any(cellfun(@(p) reads_expr(wsmap{w,2}, p), moved)) && contains(b.code, wsmap{w,1})
                runtime = true; break
            end
        end
        if direct || runtime
            hits{end+1} = strrep(b.name,'build_',''); %#ok<AGROW>
        end
    end

    % Read by string key from car_spec directly? Then it is wired, but an
    % override cannot reach it.
    bypass = {};
    for b = specReaders
        if contains(b.code, ['''' nm ''''])
            bypass{end+1} = strrep(b.name,'build_',''); %#ok<AGROW>
        end
    end

    if ~inDesign && isempty(hits) && ~isempty(bypass)
        verdict = 'NOT STUDYABLE -- wired, but read from car_spec directly';
        hits = cellfun(@(x) [x '*'], bypass, 'UniformOutput', false);
    elseif isempty(moved)
        verdict = 'NOTHING -- not even a derived value moves';
    elseif ~inDesign && isempty(hits)
        verdict = 'NOTHING -- values move, but no model reads them';
    elseif inDesign && isempty(hits)
        verdict = 'design model only';
    elseif ~inDesign
        verdict = 'plant only';
    else
        verdict = 'design model + plant';
    end
    out(i,:) = {nm, numel(moved), tern(inDesign,'yes','no'), ...
                strjoin_or(hits,'-'), verdict};
end

T = cell2table(out, 'VariableNames', {'Parameter','FieldsMoved','DesignModel','Plant','Reaches'});
end

% =========================================================================
function code = read_code(f)
%READ_CODE  Source with comments removed, so a comment mentioning P.Mass is
%   not mistaken for a read of it.
txt  = fileread(f);
L    = strsplit(txt, newline);
keep = strings(numel(L),1);
for k = 1:numel(L)
    s = L{k};
    t = strtrim(s);
    if startsWith(t,'%'), continue; end
    % drop a trailing comment, but not a % inside a quoted string
    q = false;
    for j = 1:numel(s)
        if s(j) == '''', q = ~q; end
        if s(j) == '%' && ~q, s = s(1:j-1); break; end
    end
    keep(k) = s;
end
% char, not string: callers concatenate with [a newline b], which on string
% objects builds a 1x3 ARRAY instead of joining the text.
code = char(strjoin(keep, newline));
end

function M = workspace_map(f)
%WORKSPACE_MAP  {'IFSSIM_name', 'P.expression'} pairs from ifssim_load_workspace.
code = read_code(f);
tok  = regexp(code, '''(IFSSIM_\w+)''\s*,\s*([^,\n]+)', 'tokens');
M = cell(numel(tok), 2);
for k = 1:numel(tok), M(k,:) = {tok{k}{1}, strtrim(tok{k}{2})}; end
end

function tf = reads(code, path)
tf = ~isempty(regexp(code, ['P\.' regexptranslate('escape', path) '(?![\w])'], 'once'));
end

function tf = reads_expr(expr, path)
tf = ~isempty(regexp(expr, ['P\.' regexptranslate('escape', path) '(?![\w])'], 'once'));
end

function moved = diff_tree(a, b, pre)
%DIFF_TREE  Paths of every numeric leaf that differs between two parameter trees.
moved = {};
if isstruct(a) && isstruct(b)
    f = fieldnames(a);
    for k = 1:numel(f)
        if ~isfield(b, f{k}), continue; end
        if isempty(pre), p = f{k}; else, p = [pre '.' f{k}]; end
        moved = [moved, diff_tree(a.(f{k}), b.(f{k}), p)]; %#ok<AGROW>
    end
elseif isnumeric(a) && isnumeric(b) && isequal(size(a), size(b))
    if any(abs(a(:) - b(:)) > 1e-12 * max(1, max(abs(a(:)))))
        moved = {pre};
    end
end
end

function v = spec_value(C, nm)
for i = 1:numel(C.Order)
    f = C.Fields.(C.Order{i});
    if strcmp(f.name, nm), v = f.value; return; end
end
error('spec_reach:unknown', '"%s" is not a car_spec parameter.', nm);
end

function v1 = perturb(v0)
% 10%, which moves anything continuous; a zero gets a small absolute nudge
% instead, since 10% of zero is zero and would wrongly read as "reaches nothing".
if v0 == 0, v1 = 0.1; else, v1 = v0 * 1.1; end
end

function s = strjoin_or(c, d), if isempty(c), s = d; else, s = strjoin(c, ', '); end, end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
