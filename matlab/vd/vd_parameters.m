function T = vd_parameters()
%VD_PARAMETERS  Every parameter the suspension & dynamics department owns.
%
%   What you can change, what it is worth trusting, WHERE IT ACTUALLY REACHES,
%   and what it changes. Run it before changing anything.
%
%       cd matlab/vd
%       vd_parameters
%
%   The "reaches" column is the one people are surprised by. A parameter can be
%   declared in car_spec, exported to settings.json, and still be read by
%   nothing at all -- which means changing it does exactly nothing and the
%   engineer has no way to tell. Two of the numbers below are in that state
%   today, and one of them was in it for months while people assumed the car
%   had a balance it did not have.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','plant')); addpath(fullfile(here,'..','spec'));
P = ifssim_params();
C = car_spec();

% name, reaches, what it changes
SPEC = {
'Wheelbase'          'both'        'yaw response, Ackermann, longitudinal transfer'
'TrackFront'         'both'        'lateral transfer per axle, Ackermann'
'TrackRear'          'both'        'lateral transfer per axle'
'WeightDistFront'    'both'        'balance, static loads, longitudinal transfer'
'CoGHeight'          'both'        'ALL load transfer, linearly'
'Mass'               'both'        'everything'
'HeaveStiffness'     'both'        'ride frequency, damper coeff, and the bar rates'
'RollStiffnessFront' 'both'        'BALANCE. front bar rate = this minus the springs'
'RollStiffnessRear'  'both'        'BALANCE. rear bar rate = this minus the springs'
'SuspensionDamping'  'both'        'damper coefficient, transient response only'
'Susp.SpringRateFront'  'both'        'wheel rate front, as k*MR^2'
'Susp.SpringRateRear'   'both'        'wheel rate rear -- a front/rear split is now real'
'Susp.MotionRatioFront' 'both'        'wheel rate goes as the SQUARE of it'
'Susp.MotionRatioRear'  'both'        'as front'
'Susp.ArbRateFront'     'design only' 'bar rate. axle roll stiffness = springs + this'
'Susp.ArbRateRear'      'design only' 'as front. the RATIO of the two is the balance'
'Susp.StaticCamberFront' 'both'        'INERT for grip: cost is charged vs departure from it'
'Susp.StaticCamberRear'  'both'        'INERT for grip, same reason'
'Susp.CamberGainFront'  'both'        'how much roll the geometry takes back out of the tyre'
'Susp.CamberGainRear'   'both'        'as front'
'Susp.BumpSteerFront'   'design only' 'roll steer per m of travel. ZERO by design'
'Susp.BumpSteerRear'    'design only' 'as front'
'Susp.CamberGripSensitivity' 'design only' 'the ONE part of camber that needs tyre data'
'RollCenterFront'    'design only' 'geometric share of front transfer'
'RollCenterRear'     'design only' 'geometric share of rear transfer'
'PitchStiffness'     'NOTHING'     'nothing. declared, exported, read by no code'
'MaxSteerAngle'      'both'        'steering range, Ackermann'
'WheelRadius'        'both'        'speed from rpm, tyre size, gearing'
'WheelWidth'         'both'        'tyre contact width'
'TireMu'             'both'        'grip level, cornering stiffness'
'Tyre.LoadSensitivity' 'both'      'what load transfer COSTS in grip. pairs with balance'
'Tyre.NominalLoad'   'both'        'the load the tyre data belongs to. NOT the car''s corner load'
'Tyre.StiffnessPeakLoadRatio' 'both' 'how cornering stiffness varies with load'
'RollingResistance'  'plant only'  'drag and lap energy; the design model prescribes speed'
};

fprintf('\n================ SUSPENSION & VEHICLE DYNAMICS ================\n');
fprintf('  %-28s %11s %-7s %-12s %s\n','PARAMETER','VALUE','UNIT','PROVENANCE','REACHES');
sources = struct();
for i = 1:size(SPEC,1)
    nm = SPEC{i,1};
    [val, unit, src] = lookup(C, nm);
    cls = provclass(src);
    sources.(matlab.lang.makeValidName(nm)) = cls;
    fprintf('  %-28s %11.5g %-7s %-12s %s\n', nm, val, unit, cls, SPEC{i,2});
end

fprintf('\n---- what each one changes ------------------------------------\n');
for i = 1:size(SPEC,1)
    fprintf('  %-28s %s\n', SPEC{i,1}, SPEC{i,3});
end

fprintf('\n---- READ THIS BEFORE TRUSTING A RESULT -----------------------\n');
fprintf('  PitchStiffness reaches NOTHING. It is declared, it is exported to\n');
fprintf('  settings.json, and no line of code reads it. Changing it does not\n');
fprintf('  change the car. It is also UNKNOWN -- no source anywhere -- so\n');
fprintf('  there is nothing to lose by ignoring it until pitch is modelled.\n\n');
fprintf('  RollCenterFront/Rear reach the DESIGN model only. The plant applies\n');
fprintf('  tyre forces at the contact patch with the roll centre at ground\n');
fprintf('  level, so all lateral transfer there is elastic. The design model\n');
fprintf('  splits it geometric/elastic and gets ~11%% geometric at the front.\n');
fprintf('  So the two models disagree on the SPLIT, never on the total.\n\n');
fprintf('  STATIC CAMBER IS INERT FOR GRIP, on purpose. The camber penalty is\n');
fprintf('  charged against DEPARTURE from static, because the static setting\n');
fprintf('  was presumably chosen near the tyre''s optimum and no data says where\n');
fprintf('  that optimum is. So these two move the camber CURVE and not the lap\n');
fprintf('  time. Choosing static camber needs a tyre on a rig.\n\n');
fprintf('  RollStiffnessFront/Rear have NO SOURCE, and their RATIO is the\n');
fprintf('  car''s balance -- the single number the whole handling picture turns\n');
fprintf('  on. Currently %.1f%% front.\n\n', ...
        100*P.Derived.RollStiffnessFront/(P.Derived.RollStiffnessFront+P.Derived.RollStiffnessRear));
fprintf('  CoGHeight is DISPUTED: %.3f here, 0.3441 in the VD department file,\n', P.CoGHeight);
fprintf('  nobody has measured the IFS-08. Load transfer is LINEAR in it, so a\n');
fprintf('  15%% error here is a 15%% error in every corner load on this page.\n');

fprintf('\n---- the four measurements that would change the most ----------\n');
fprintf('  1. CoG height           tape measure and corner scales, on a ramp\n');
fprintf('  2. Roll stiffnesses     spring rates and bar rates off the car\n');
fprintf('  3. Tyre on a rig        mu, load sensitivity, cornering stiffness\n');
fprintf('  4. Yaw inertia Izz      currently %.0f kg.m^2, a typical value\n', P.Assumed.Izz);
fprintf('===============================================================\n');
fprintf('  vd_report      what the car does\n');
fprintf('  vd_plots       the same, as figures\n');
fprintf('  vd_study(...)  try a change on the design model\n');
fprintf('  plant_study(...) try a change on the full Simulink plant\n\n');

T = cell2table(SPEC, 'VariableNames', {'Parameter','Reaches','Changes'});
end

% -----------------------------------------------------------------------
function [val, unit, src] = lookup(C, name)
key = strrep(name,'.','_');
val = NaN; unit = '?'; src = 'UNKNOWN';
if isfield(C.Fields, key)
    f = C.Fields.(key);
    val = f.value; unit = f.unit; src = f.source;
end
end

function c = provclass(src)
% The first word of a source string is its class, by the convention car_spec
% enforces. Anything that does not start with one is treated as unsourced.
w = upper(strtok(src));
known = {'MEASURED','MEASURED-ISH','GEOMETRY','DERIVED','DATASHEET', ...
         'SECONDARY','ASSUMED','DISPUTED','ZEROED','UNKNOWN'};
if any(strcmp(w, known)), c = w; else, c = 'UNKNOWN'; end
if strcmp(c,'MEASURED-ISH'), c = 'MEASURED~'; end
end
