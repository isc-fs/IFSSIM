function ifssim_plant_build()
%IFSSIM_PLANT_BUILD  Rebuild the whole plant from source.
%
%   Runs the skeleton generator, then each subsystem builder in turn.
%
%   SAFE TO RUN. It regenerates the top-level wiring and every subsystem that
%   has a builder. If you are working on a subsystem BY HAND in Simulink and it
%   has no builder script, this will not touch it. If it does have a builder,
%   your hand edits are overwritten — so put your work in the builder, not in
%   the .slx. That is the price of a model you can review in a diff.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
ifssim_workdir();

fprintf('\n=== building plant ===\n');
build_plant_skeleton();          % top level + placeholders for anything unfilled

% Subsystems that have a real implementation. Add yours here when you write one.
builders = {@build_chassis, @build_tiresuspension, @build_steering, @build_powertrain, @build_aero, @build_brakes};
for i = 1:numel(builders), builders{i}(); end

% The drive harness comes last: it references the assembled plant.
build_drive_harness();
fprintf('\nbuild complete.\n');
end
