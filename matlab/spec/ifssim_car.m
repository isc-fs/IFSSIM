function name = ifssim_car(set)
%IFSSIM_CAR  Which prototype every tool is working on. IFS-08 unless you say so.
%
%   ifssim_car()            the active car's name
%   ifssim_car('IFS-09')    work on the IFS-09: car_spec, ifssim_params, the
%                           plant build and every department view follow it
%
%   FOR THIS MATLAB SESSION ONLY. A restart is back on the IFS-08, on purpose:
%   a choice that outlived the session would let somebody open MATLAB a week
%   later, build "the car", and get the wrong one with nothing to say so.
%   Batch jobs and CI choose with the environment variable IFSSIM_CAR.
%
%   Held on groot rather than in a persistent variable, so 'clear functions'
%   -- which MATLAB users run without thinking -- cannot silently switch the
%   session back to the IFS-08 halfway through an IFS-09 job.
%
%   Switching closes any plant model already loaded: the cars' models share
%   names (IFSSIM_Plant, ...), and a model left in memory from the other car
%   would be simulated in place of this one's.

DEFAULT = 'IFS-08';
if nargin >= 1
    known = car_list();
    if ~any(strcmp(known, set))
        error('ifssim_car:unknown', '"%s" is not a car. Cars in spec/cars: %s', set, strjoin(known, ', '));
    end
    was = ifssim_car();
    setappdata(groot, 'IFSSIM_car', set);
    if ~strcmp(was, set)
        close_plant_models();
        fprintf('active car: %s (was %s). Its plant builds to %s\n', set, was, ifssim_models_dir_hint(set));
    end
end
name = getappdata(groot, 'IFSSIM_car');
if isempty(name)
    env = getenv('IFSSIM_CAR');
    if ~isempty(env), name = env; else, name = DEFAULT; end
end
end

function close_plant_models()
if ~exist('find_system', 'file'), return; end
try
    open = find_system('type', 'block_diagram');
catch
    return
end
for k = 1:numel(open)
    if startsWith(open{k}, 'IFSSIM_')
        close_system(open{k}, 0);
    end
end
end

function d = ifssim_models_dir_hint(name)
try
    d = ifssim_models_dir(name);
catch
    d = '(plant not on the path)';
end
end
