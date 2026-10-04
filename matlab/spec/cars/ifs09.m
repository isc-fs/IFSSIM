function C = ifs09()
%IFS09  The IFS-09, the next prototype. Starts as the IFS-08; says what is different.
%
%   EVERY VALUE HERE IS INHERITED FROM THE IFS-08 until this file says
%   otherwise, and its source says so ("INHERITED from IFS-08 -- ..."). That
%   is deliberate: a new car is not measured yet, and borrowing the old car's
%   numbers is fine as long as nothing pretends they are the new car's.
%   check_car counts what is still inherited.
%
%   TO MAKE A NUMBER THE IFS-09'S OWN, set it below with a source of its
%   own, exactly as in ifs08.m:
%
%       C = par(C,'Pack.CellsParallelPerModule', 8, '-', 'DESIGN accumulator team 2026-11: ...');
%
%   Only what differs goes here. A value that is the same on both cars stays
%   inherited -- copying it would make two copies of one number, free to drift.
%
%   WHAT IS EXPECTED TO CHANGE, so whoever fills this in knows where to start:
%     - the ACCUMULATOR's electrical configuration (cell, Pack.*): the plant's
%       Simscape Battery pack is generated from these, so a new arrangement
%       needs no code (battery_from_spec);
%     - and then, subsystem by subsystem, the rest.
%
%   Work on it with ifssim_car('IFS-09'); every tool then runs this car. Its
%   plant models and FMU build into matlab/build/cars/IFS-09/, never over the
%   IFS-08's. car_diff('IFS-08','IFS-09') lists what is different.

C = inherit(ifs08(), 'IFS-09');

%% ---- what is different about the IFS-09 --------------------------------
% (nothing yet)
end
