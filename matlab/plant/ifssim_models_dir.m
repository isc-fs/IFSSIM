function d = ifssim_models_dir(car, what)
%IFSSIM_MODELS_DIR  Where a car's plant is built. The one place that decides.
%
%   d = IFSSIM_MODELS_DIR()              the active car's models folder
%   d = IFSSIM_MODELS_DIR('IFS-09')      a named car's
%   d = IFSSIM_MODELS_DIR(car, 'fmu')    its FMU folder instead
%
%   THE SIMULATOR'S CAR (the IFS-08) builds where it always has:
%   plant/models and plant/fmu, committed, loaded by UE5. EVERY OTHER CAR
%   builds into matlab/build/cars/<name>/, which is gitignored -- so building
%   the IFS-09 can never overwrite the IFS-08's models, and a prototype's
%   plant is not committed until somebody decides it should be.
%
%   It matters more than it looks: the cars' models share their names
%   (IFSSIM_Plant, IFSSIM_Powertrain, ...) and Simulink finds them on the
%   path. ifssim_load_workspace keeps exactly one car's folder on the path.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'spec'));
if nargin < 1 || isempty(car), car = ifssim_car(); end
if nargin < 2 || isempty(what), what = 'models'; end
C = car_spec(car);
if C.Simulator
    d = fullfile(here, what);
else
    d = fullfile(fileparts(here), 'build', 'cars', car, what);
end
if ~isfolder(d), mkdir(d); end
end
