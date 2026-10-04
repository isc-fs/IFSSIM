function C = car_spec(name)
%CAR_SPEC  A car: every number the simulator runs on, and where it came from.
%
%   C = CAR_SPEC()          the ACTIVE car (ifssim_car)
%   C = CAR_SPEC('IFS-09')  a named one
%
%   One file per prototype, in spec/cars/: ifs08.m, ifs09.m, ... THOSE are the
%   files you edit. This only picks one. Every tool that reads the car --
%   ifssim_params, the plant build, the department views -- comes through here,
%   so switching the active car switches all of them at once.
%
%   See also IFSSIM_CAR, CAR_LIST, CAR_DIFF, BUILD_CAR, CHECK_CAR.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'cars'));
if nargin < 1 || isempty(name), name = ifssim_car(); end
fn = car_function(name);
C = feval(fn);
if ~strcmp(C.Name, name)
    error('car_spec:name', '%s.m returned a car named %s, not %s', fn, C.Name, name);
end
end

function fn = car_function(name)
known = car_list();
if ~any(strcmp(known, name))
    error('car_spec:unknown', '"%s" is not a car. Cars in spec/cars: %s', name, strjoin(known, ', '));
end
fn = lower(strrep(name, '-', ''));
end
