function ifssim_setup()
%IFSSIM_SETUP  Put the plant on the MATLAB path. Run this first.
%
%   Adds matlab/plant and matlab/plant/models, then loads parameters and buses
%   into the base workspace.
%
%   You need this before opening any model. A model opened without its bus
%   objects shows red ports and unhelpful errors, and the natural conclusion —
%   "someone checked in a broken model" — is wrong.

here = fileparts(mfilename('fullpath'));
addpath(here);
ifssim_workdir();
addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

fprintf('IFSSIM plant ready.\n');
fprintf('  parameters : %s\n', P.SpecPath);
fprintf('  models     : %s\n', fullfile(here,'models'));
fprintf('\n  ifssim_plant_check     build everything and run every test\n');
fprintf('  ifssim_params_report   show every parameter and where it came from\n');
fprintf('  open_system(''IFSSIM_Plant'')\n\n');
end
