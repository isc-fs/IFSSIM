function name = simulator_car()
%SIMULATOR_CAR  The prototype the UE5 simulator runs: its spec says Simulator = true.
%
%   Exactly one car may say so. Its settings.json, plant models and FMU are the
%   committed ones; every other car builds into matlab/build/cars/<name>/.
names = car_list();
is = false(size(names));
for k = 1:numel(names)
    C = car_spec(names{k});
    is(k) = C.Simulator;
end
if nnz(is) ~= 1
    error('simulator_car:count', ['exactly one car must have Simulator = true; found %d (%s).\n' ...
          'The simulator loads one car, and its build is the one committed.'], ...
          nnz(is), strjoin(names(is), ', '));
end
name = names{is};
end
