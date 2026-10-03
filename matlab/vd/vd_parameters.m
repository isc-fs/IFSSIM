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
%   engineer has no way to tell.
%
%   THAT COLUMN IS COMPUTED (spec_reach), not written. It used to be a column
%   of words in the table below, and when it was first checked against the
%   code 7 of its 34 entries were wrong: both declared roll stiffnesses said
%   "both" and reached nothing, the bar rates said "design only" and reach the
%   plant too, and three plant-only parameters claimed the design model. Only
%   the "what it changes" sentence is still written by hand.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','plant')); addpath(fullfile(here,'..','spec'));
P = ifssim_params();
C = car_spec();

% name, what it changes
SPEC = {
'Wheelbase'          'yaw response, Ackermann, longitudinal transfer'
'TrackFront'         'lateral transfer per axle, Ackermann'
'TrackRear'          'lateral transfer per axle'
'WeightDistFront'    'balance, static loads, longitudinal transfer'
'CoGHeight'          'ALL load transfer, linearly'
'Mass'               'everything'
'HeaveStiffness'     'ride frequency, damper coeff, and the bar rates'
'RollStiffnessFront' 'DEAD. the declared total; the models read the DERIVED one'
'RollStiffnessRear'  'DEAD. as front -- change the springs or the bar instead'
'SuspensionDamping'  'damper coefficient, transient response only'
'Susp.SpringRateFront'  'wheel rate front, as k*MR^2'
'Susp.SpringRateRear'   'wheel rate rear -- a front/rear split is now real'
'Susp.MotionRatioFront' 'wheel rate goes as the SQUARE of it'
'Susp.MotionRatioRear'  'as front'
'Susp.ArbRateFront'     'bar rate. axle roll stiffness = springs*MR^2 + this'
'Susp.ArbRateRear'      'as front. the RATIO of the two is the balance'
'Susp.StaticCamberFront' 'INERT for grip: cost is charged vs departure from it'
'Susp.StaticCamberRear'  'INERT for grip, same reason'
'Susp.CamberGainFront'  'how much roll the geometry takes back out of the tyre'
'Susp.CamberGainRear'   'as front'
'Susp.BumpSteerFront'   'roll steer per m of travel. ZERO by design'
'Susp.BumpSteerRear'    'as front'
'Susp.CamberGripSensitivity' 'the ONE part of camber that needs tyre data'
'RollCenterFront'    'geometric share of front transfer'
'RollCenterRear'     'geometric share of rear transfer'
'PitchStiffness'     'nothing. declared, exported, read by no code'
'MaxSteerAngle'      'steering range, Ackermann'
'WheelRadius'        'speed from rpm, tyre size, gearing'
'WheelWidth'         'tyre contact width'
'TireMu'             'grip level, cornering stiffness'
'Tyre.LoadSensitivity' 'what load transfer COSTS in grip. pairs with balance'
'Tyre.NominalLoad'   'the load the tyre data belongs to. NOT the car''s corner load'
'Tyre.StiffnessPeakLoadRatio' 'how cornering stiffness varies with load'
'RollingResistance'  'drag and lap energy; the design model prescribes speed'
};

R = spec_reach(SPEC(:,1));
SPEC = [SPEC(:,1), R.Reaches, SPEC(:,2)];

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
dead = SPEC(startsWith(SPEC(:,2),'NOTHING') | startsWith(SPEC(:,2),'NOT STUDYABLE'), 1);
if ~isempty(dead)
    fprintf('  CHANGING THESE DOES NOT CHANGE THE CAR (computed just now):\n');
    fprintf('    %s\n', strjoin(dead, ', '));
    fprintf('  vd_study refuses them rather than print two identical cars.\n\n');
end
fprintf('  RollStiffnessFront/Rear in car_spec are DECLARED totals that nothing\n');
fprintf('  reads. The models use the DERIVED axle stiffness, built from the\n');
fprintf('  springs, motion ratio and bar. Balance moves with Susp.ArbRate*.\n\n');
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
fprintf('  The roll stiffness SPLIT is the car''s balance -- the single number\n');
fprintf('  the whole handling picture turns on -- and the bar rates under it are\n');
fprintf('  DERIVED placeholders, not bars off the car. Currently %.1f%% front.\n\n', ...
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
