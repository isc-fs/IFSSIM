function names = car_list()
%CAR_LIST  Every prototype with a spec file in spec/cars, by name ('IFS-08', ...).
%
%   A car exists by having a file: spec/cars/ifs09.m is the IFS-09. Adding a
%   prototype is adding a file, nothing else.
here = fileparts(mfilename('fullpath'));
d = dir(fullfile(here, 'cars', 'ifs*.m'));
names = cell(1, numel(d));
for k = 1:numel(d)
    t = regexp(d(k).name, '^ifs(\d+)\.m$', 'tokens', 'once');
    names{k} = sprintf('IFS-%s', t{1});
end
names = sort(names);
end
