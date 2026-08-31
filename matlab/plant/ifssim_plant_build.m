function ifssim_plant_build(useVDB)
%IFSSIM_PLANT_BUILD  Rebuild the whole plant from source.
%
%   Runs the skeleton generator, then each subsystem builder in turn.
%
%   SAFE TO RUN. It regenerates the top-level wiring and every subsystem that
%   has a builder. If you are working on a subsystem BY HAND in Simulink and it
%   has no builder script, this will not touch it. If it does have a builder,
%   your hand edits are overwritten — so put your work in the builder, not in
%   the .slx. That is the price of a model you can review in a diff.
%
%   IFSSIM_PLANT_BUILD(TRUE) builds the VDB variants instead: MathWorks'
%   Double Wishbone suspension and Vehicle Body 6DOF in place of our own
%   closed forms. Off by default. See docs/vdb_plant_migration.md.
%
%   The flag is threaded to EVERY builder that has a variant, not just the one
%   being worked on. A plant half-migrated is not a configuration anyone asked
%   for, and it would be reported as whichever half was looked at.

if nargin < 1 || isempty(useVDB), useVDB = false; end
here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
ifssim_workdir();

fprintf('\n=== building plant ===\n');
build_plant_skeleton([], useVDB); % top level + placeholders for anything unfilled

% Subsystems that have a real implementation. Add yours here when you write one.
% The two VDB-aware builders are called BY NAME rather than through a loop,
% because their signatures genuinely differ: build_tiresuspension takes an
% overrides argument and build_chassis does not. Looping over them passed the
% flag into the wrong slot and the failure surfaced four frames away, inside
% override normalisation, as "overrides must be a struct". Spelling the calls
% out costs two lines and cannot go wrong that way.
build_chassis([], useVDB);
build_tiresuspension([], [], useVDB);

plain = {@build_steering, @build_powertrain, @build_aero, @build_brakes};
for i = 1:numel(plain), plain{i}(); end
if useVDB
    fprintf('  (VDB variants: Double Wishbone suspension, Vehicle Body 6DOF)\n');
end

% The drive harness comes last: it references the assembled plant.
build_drive_harness();
fprintf('\nbuild complete.\n');
end
