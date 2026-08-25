function fmuPath = export_plant_fmu(outdir)
%EXPORT_PLANT_FMU  Export IFSSIM_Plant as an FMI 3.0 Co-Simulation FMU.
%
%   The output is what the simulator loads. One file, no MATLAB required to
%   run it — which is the point: a teammate without a Simulink licence still
%   gets the car.
%
%   Check the result with either inspector; they share no code and must agree:
%       python3 tools/fmu/inspect_fmu.py <the .fmu>
%       inspectFmu <abs path>            over the sim RPC

here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(outdir)
    outdir = fullfile(here, 'fmu');
end
outdir = char(outdir);
if ~isfolder(outdir), mkdir(outdir); end
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

% Simulink.FMUExporter RESOLVES THE MODEL RELATIVE TO THE CURRENT DIRECTORY,
% not the MATLAB path. With the model on the path but pwd elsewhere it fails
% with "Model does not exist", which points nowhere near the cause. cd into the
% model directory and restore afterwards.
modelDir = fullfile(here,'models');
oldPwd = pwd;                       %#ok<NASGU>
restore = onCleanup(@() cd(oldPwd));
cd(modelDir);

mdl = 'IFSSIM_Plant';
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system(mdl);

% A finite stop time is required for export; the platform ignores it and steps
% the FMU itself, but 'inf' is rejected at export.
set_param(mdl,'StopTime','1e5');
save_system(mdl);

e = Simulink.FMUExporter(mdl);
% SET THE OPTIONS, NOT THE OBJECT PROPERTIES. e.SaveDirectory and e.FMUName
% exist and are silently IGNORED — export() then returns without error and
% writes nothing, which is the least helpful failure mode available.
opts = { 'SaveDirectory',outdir
         'FMUName',mdl
         'FMIVersion','3.0'
         'FMUType','CS'
         'canGetAndSetFMUStateOverride','on'   % the deterministic-reset gate
         'SaveSourceCodeToFMU','on'            % lets other platforms compile it
         'CreateModelAfterGeneratingFMU','off'
         'AddIcon','off' };
for i = 1:size(opts,1)
    try
        e.Options.(opts{i,1}) = opts{i,2};
    catch
        try, e.Options.(opts{i,1}) = char(matlab.lang.OnOffSwitchState(opts{i,2})); catch, end
    end
end

fprintf('exporting %s ...\n', mdl);
e.export();

d = dir(fullfile(outdir,'*.fmu'));
if isempty(d), error('export produced no .fmu'); end
fmuPath = fullfile(outdir, d(1).name);
fprintf('\nPRODUCED %s (%.0f kB)\n', fmuPath, d(1).bytes/1024);
fprintf('car: %.0f kg, %.0f Nm, wheelbase %.3f m, mu %.2f\n', ...
        P.Mass, P.MotorMaxTorque, P.Wheelbase, P.TireMu);
close_system(mdl,0);
end
