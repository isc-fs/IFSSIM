function accel_compare(s_target)
%ACCEL_COMPARE  The plant against the hand-written model, on the same event.
%
%   Runs the Formula Student acceleration event twice: once on the plant the
%   driverless simulator drives, once on the point-mass model in
%   DYNAMIC_MOD/GeneralCalculations. Prints both, and the parameters each is
%   using, because the parameters are usually where the disagreement is.
%
%   THIS IS THE ARGUMENT FOR SHARING THE PLANT, made concrete. The two models
%   currently disagree by most of a second and a half on a four-second event.
%   That gap is not a modelling nicety -- it is the difference between a
%   simulator you can set a car up with and one you cannot -- and it is only
%   visible because both were pointed at the same event.
%
%   The legacy model lives in an untracked folder, so this degrades to
%   plant-only if it is not there.

if nargin < 1, s_target = 75; end
here = fileparts(mfilename('fullpath'));

R = accel_run(s_target);

leg = fullfile(here,'..','archive','MODEL_IFS_08','DYNAMIC_MOD','GeneralCalculations');
if ~isfolder(leg)
    fprintf('\n(legacy model not present at %s -- plant only)\n', leg);
    return
end
addpath(fullfile(leg,'Data')); addpath(fullfile(leg,'Modelos'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','spec'));
try
    car = assembleCar();
    L   = accelSimDistance(car, s_target);
catch e
    fprintf('\n(legacy model would not run: %s)\n', regexprep(e.message,'\s+',' '));
    return
end

P = ifssim_load_workspace();
fprintf('\n=== the same event, two models ===\n');
fprintf('  %-24s %14s %14s\n','', 'PLANT', 'point-mass');
fprintf('  %-24s %14.3f %14.3f  s\n', sprintf('0 - %.0f m', s_target), R.t_target, L.t);
fprintf('  %-24s %14.1f %14.1f  km/h\n','speed at the line', R.v_end_kmh, L.v_end_kmh);
fprintf('\n  the parameters each is using:\n');
fprintf('  %-24s %14.3f %14.3f  kg\n','mass',          P.Mass,              car.m);
fprintf('  %-24s %14.3f %14.3f  -\n', 'peak mu',       P.TireMu,            car.mu_long_ref);
fprintf('  %-24s %14.3f %14.3f  -\n', 'rolling resistance', P.RollingResistance, car.Crr);
fprintf('  %-24s %14.3f %14.3f  m\n', 'wheel radius',  P.WheelRadius,       car.r_wheel);

fprintf('\n  Where the difference comes from, in order:\n');
fprintf('  1. The point-mass model CANNOT wheelspin -- it caps traction at\n');
fprintf('     mu*Fz, which is a car with perfect traction control. The plant\n');
fprintf('     integrates wheel speed, and off the line it spins to a slip\n');
fprintf('     ratio of %.0f. Neither is right: a real car without TC spins\n', max(R.hist(:,6)));
fprintf('     some, nowhere near that much.\n');
fprintf('  2. The two disagree about the tyre: %.2f against %.2f.\n', P.TireMu, car.mu_long_ref);
fprintf('  3. And about the car: %.0f kg against %.0f kg.\n', P.Mass, car.m);
fprintf('\n  Points 2 and 3 are settled by picking a number in car_spec.\n');
fprintf('  Point 1 is a real defect in the plant and is not settled yet.\n');
end
