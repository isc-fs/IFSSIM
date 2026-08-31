function open_models()
%OPEN_MODELS  Open the plant's Simulink diagrams, with buses loaded first.
%
%   A model opened without its bus objects shows red ports and errors that
%   look like a broken model and are not. ifssim_setup loads them.
here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','spec'));
cd(here);
ifssim_setup;
open_system('IFSSIM_Plant');
open_system('IFSSIM_TireSuspension');
load_system('vehdynlibsuspension'); open_system('vehdynlibsuspension');
disp('IFSSIM_Plant, IFSSIM_TireSuspension and the VDB suspension library are open.');
end
