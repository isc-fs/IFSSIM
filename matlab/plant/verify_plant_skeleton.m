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
