function ok = verify_plant_skeleton(outdir)
%VERIFY_PLANT_SKELETON  Compile every generated model and report.
%
%   A skeleton that generates but does not COMPILE is worse than none — the
%   engineer who opens it inherits someone else's broken model and cannot tell
%   which errors are theirs. This compiles each referenced model on its own,
%   then the top level.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'generated');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
assignin('base','IFSSIM_P', ifssim_params());
ifssim_plant_buses();
% Placeholder Constant blocks reference these zeroed bus structs by name.
for b = {'IFSSIM_PoseBus','IFSSIM_WheelsBus','IFSSIM_PowertrainBus','IFSSIM_StatusBus'}
    assignin('base',[b{1} '_zero'], Simulink.Bus.createMATLABStruct(b{1}));
end

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
        try, eval([m '([],[],[],''term'');']); catch, end
    end
    try, close_system(m,0); catch, end
end
fprintf('\n%s\n', ternary(ok,'all models compile.','SOME MODELS FAILED TO COMPILE.'));
end
function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end
