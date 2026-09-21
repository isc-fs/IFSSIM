function ok = verify_plant_skeleton(outdir)
%VERIFY_PLANT_SKELETON  Compile every generated model and report.
%
%   A skeleton that generates but does not COMPILE is worse than none — the
%   engineer who opens it inherits someone else's broken model and cannot tell
%   which errors are theirs. This compiles each referenced model on its own,
%   then the top level.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
ifssim_load_workspace();

models = {'IFSSIM_Steering','IFSSIM_Powertrain','IFSSIM_Brakes', ...
          'IFSSIM_TireSuspension','IFSSIM_Aero','IFSSIM_Chassis','IFSSIM_Plant'};
ok = true;
fprintf('\n=== compiling plant skeleton ===\n');

% UNCONNECTED PORTS FIRST. A model with dangling ports still COMPILES — which
% is how the top-level plant sat as six disconnected blocks reporting green.
% "It compiles" and "it is wired" are different claims and this checks both.
ok = check_connectivity(models) && ok;
for i = 1:numel(models)
    m = models{i};
    try
        load_system(m);
        eval([m '([],[],[],''compile'');']);
        eval([m '([],[],[],''term'');']);
        fprintf('  [ok  ] %s\n', m);
    catch ME
        ok = false;
        fprintf('  [FAIL] %s\n         %s\n', m, ME.message);
        % Simulink wraps compile failures as "Error due to multiple causes",
        % which says nothing. Walk the causes — the real message is in there.
        print_causes(ME, 2);
        try, eval([m '([],[],[],''term'');']); catch, end
    end
    try, close_system(m,0); catch, end
end
fprintf('\n%s\n', ternary(ok,'all models compile.','SOME MODELS FAILED TO COMPILE.'));
end
function ok = check_connectivity(models)
ok = true;
for i = 1:numel(models)
    m = models{i};
    try, load_system(m); catch, continue; end
    blocks = find_system(m,'SearchDepth',1,'Type','Block');
    dangling = {};
    for b = 1:numel(blocks)
        pc = get_param(blocks{b},'PortConnectivity');
        for k = 1:numel(pc)
            if isempty(pc(k).SrcBlock) && isempty(pc(k).DstBlock)
                dangling{end+1} = get_param(blocks{b},'Name'); %#ok<AGROW>
                break
            end
        end
    end
    if isempty(dangling)
        fprintf('  [ok  ] %s wired\n', m);
    else
        ok = false;
        fprintf('  [FAIL] %s has unconnected port(s) on: %s\n', m, strjoin(unique(dangling),', '));
    end
end
end

function print_causes(ME, depth)
if depth > 4 || isempty(ME.cause), return; end
for i = 1:numel(ME.cause)
    c = ME.cause{i};
    fprintf('%s- %s\n', repmat('  ',1,depth+2), strtrim(c.message));
    print_causes(c, depth+1);
end
end

function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end
