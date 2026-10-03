function [lib, block, B] = battery_library(P)
%BATTERY_LIBRARY  The generated Simscape Battery library for P's arrangement.
%
%   [lib, block, B] = BATTERY_LIBRARY(P)
%
%   Generates it with buildBattery the first time an arrangement is seen, and
%   reuses it after that -- generation takes ~50 s, and only the ARRANGEMENT is
%   in it (battery_from_spec), so the same library serves every cell and every
%   study of one.
%
%   WHERE IT GOES. The car_spec arrangement lives in plant/models/battery/ and
%   is committed with the other models, so the plant opens without
%   generating anything. Any other arrangement -- a study, or the next
%   prototype before it is in car_spec -- goes to the gitignored build folder,
%   so trying one cannot leave a library in the repository.
%
%   block is the pack block inside it, ready for add_block.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','spec'));
B = battery_from_spec(P);
lib = B.LibraryName;

Bspec = battery_from_spec(ifssim_params());
if strcmp(lib, Bspec.LibraryName)
    d = fullfile(here, 'models', 'battery');
else
    d = fullfile(ifssim_workdir(), 'battery');
end
if ~isfolder(d), mkdir(d); end
addpath(d);

if ~isfile(fullfile(d, [lib '.slx']))
    fprintf('  generating the Simscape Battery library %s (once per arrangement, ~1 min)\n', lib);
    here0 = pwd;  restore = onCleanup(@() cd(here0));
    cd(d);                                 % buildBattery compiles into the current folder too
    buildBattery(B.Pack, LibraryName=lib, Directory=d);
end
load_system(lib);
blk = find_system(lib, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^Pack\d*$');
if numel(blk) ~= 1
    error('battery_library:noPack', 'expected one Pack block in %s, found %d', lib, numel(blk));
end
block = blk{1};
end
