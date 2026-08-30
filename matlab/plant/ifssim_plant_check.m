function ok = ifssim_plant_check(verbose)
%IFSSIM_PLANT_CHECK  The one command. Build everything, compile it, test it.
%
%   ifssim_plant_check        summary only
%   ifssim_plant_check(true)  show every individual assertion
%
%   Run it before you commit and after you pull. It answers the only question
%   that matters day to day: is the plant still right?
%
%   If something fails and you are not sure whether you broke it, `git stash`
%   and run it again. Two minutes of certainty beats an afternoon of doubt.

if nargin < 1, verbose = false; end
here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));

t0 = tic;
results = {};   % name, ok

fprintf('\n================ IFSSIM PLANT CHECK ================\n');

% 1. Parameters -------------------------------------------------------
try
    P = ifssim_load_workspace();
    nDefault = sum(strcmp(struct2cell(P.Source), 'default'));
    fprintf('parameters  %s\n', P.SpecPath);
    fprintf('            %d field(s) using defaults (ifssim_params_report shows which)\n', nDefault);
    results(end+1,:) = {'parameters load', true};
catch ME
    fprintf('parameters  FAILED: %s\n', ME.message);
    ok = false; return
end

% 2. Build and compile ------------------------------------------------
try
    run_quiet(@ifssim_plant_build, verbose);
    results(end+1,:) = {'build', true};
catch ME
    fprintf('\nbuild FAILED: %s\n', ME.message);
    results(end+1,:) = {'build', false};
end
results(end+1,:) = {'all models compile', run_check(@verify_plant_skeleton, verbose)};

% 3. Physics ----------------------------------------------------------
% Add your subsystem's test here when you write one.
tests = { 'chassis physics',         @test_chassis_physics
          'tyre/suspension physics', @test_tiresuspension_physics
          'steering physics',        @test_steering_physics
          'powertrain physics',      @test_powertrain_physics
          'aero physics',            @test_aero_physics
          'brake physics',           @test_brakes_physics };
for i = 1:size(tests,1)
    results(end+1,:) = {tests{i,1}, run_check(tests{i,2}, verbose)}; %#ok<AGROW>
end

% Summary -------------------------------------------------------------
fprintf('\n--- summary ---\n');
ok = true;
for i = 1:size(results,1)
    pass = results{i,2};
    if ~pass, ok = false; end
    fprintf('  [%s] %s\n', ternary(pass,'ok  ','FAIL'), results{i,1});
end
fprintf('\n%s   (%.0f s)\n', ternary(ok,'PLANT OK','PLANT HAS FAILURES'), toc(t0));
if ~ok && ~verbose
    fprintf('Re-run as ifssim_plant_check(true) to see which assertion failed.\n');
end
fprintf('===================================================\n\n');
end

function pass = run_check(fn, verbose)
try
    if verbose
        pass = fn();
    else
        evalc('pass = fn();');
    end
catch ME
    fprintf('  error in %s: %s\n', func2str(fn), ME.message);
    pass = false;
end
end

function run_quiet(fn, verbose)
if verbose, fn(); else, evalc('fn()'); end
end

function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
