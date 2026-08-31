function P = ifssim_params(overrides)
%IFSSIM_PARAMS  The car, from car_spec.m. The single source of truth.
%
%   P = IFSSIM_PARAMS()             the car as specified
%   P = IFSSIM_PARAMS({'TireMu',1.65})   with study overrides applied
%
%   CAR_SPEC IS THE SOURCE OF TRUTH, and nothing here reads settings.json.
%
%   The dependency used to run through it: car_spec wrote settings.json and
%   this read it back. That made every MATLAB parameter round-trip through the
%   game engine's config file, which is backwards for work that has nothing to
%   do with the engine. It cost real bugs -- a dotted name flattened on the way
%   out once wrote Cell.Mass into the car's Mass, and settings.json went from
%   275 kg to 0.0467 -- and it meant the vehicle-dynamics team could not open
%   MATLAB and study a parameter without the simulator's file being correct.
%
%   settings.json still exists and is still generated, by build_car, because
%   the UE5 bridge and the ROS tools parse it. But it is now an EXPORT. It is
%   downstream of car_spec, the same as the Simulink models are, and nothing
%   in MATLAB depends on it.
%
%   Every value carries the source string car_spec gave it, so
%   ifssim_params_report can say where each number came from rather than just
%   "settings.json", which was never an answer to that question.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'spec'));
C = car_spec();

% Field names the plant and the design model rely on. Listed so that deleting
% one from car_spec fails here, with the name, instead of somewhere downstream
% as a missing-field error three files away.
REQUIRED = {'Mass','Wheelbase','TrackFront','TrackRear','CoGHeight','WeightDistFront', ...
            'WheelRadius','WheelWidth','TireMu','RollingResistance','MaxSteerAngle', ...
            'HeaveStiffness','PitchStiffness','RollStiffnessFront','RollStiffnessRear', ...
            'RollCenterFront','RollCenterRear','SuspensionDamping','CdA','ClA', ...
            'AeroBalanceFront','MotorMaxTorque','MotorMaxPower','MaxRegenTorque', ...
            'GearRatio','DrivetrainEfficiency'};

P = struct();  P.Source = struct();
P.Pacejka = struct();  P.Cell = struct();  P.Pack = struct();  P.Assumed = struct();

for i = 1:numel(C.Order)
    f    = C.Fields.(C.Order{i});
    name = f.name;
    if ~contains(name,'.')
        P.(name) = f.value;
        P.Source.(name) = f.source;
        continue;
    end
    parts = split(name,'.');
    grp = parts{1};  key = parts{2};
    switch grp
        case 'Pacejka'
            P.Pacejka.(key) = f.value;
            P.Source.(['Pacejka_' key]) = f.source;
        case 'Tyre'
            % Tyre.X is a coefficient of the Magic Formula set, not of the
            % simulator's car, so it lands under Assumed as TyreX -- which is
            % where build_tyre_paramset and the design model already look for
            % it. There used to be a second, hand-written copy of these in
            % this file and a guard in build_car to stop the two drifting.
            % There is one copy now, so there is nothing to drift.
            P.Assumed.(['Tyre' key]) = f.value;
            P.Source.(['Tyre_' key]) = f.source;
        case {'Cell','Pack'}
            P.(grp).(key) = f.value;
            P.Source.([grp '_' key]) = f.source;
        otherwise
            error('ifssim_params:unknownGroup', ...
                  ['car_spec declares "%s" but nothing knows where group "%s" ' ...
                   'belongs. Add a case here.'], name, grp);
    end
end

missing = REQUIRED(~cellfun(@(k) isfield(P,k), REQUIRED));
if ~isempty(missing)
    error('ifssim_params:missing', ...
          'car_spec is missing parameters the plant needs: %s', strjoin(missing,' '));
end

P.SpecName = C.Name;
P.SpecPath = fullfile(here, '..', 'spec', 'car_spec.m');

% ---- study overrides --------------------------------------------------
% An engineer asking "what if the CoG were 40 mm higher" should not have to
% edit the spec to find out. Overrides are applied HERE, before anything is
% derived, so every downstream quantity recomputes -- change the wheel rate
% and the ride frequency, the damping coefficient and the anti-roll bar rates
% all move with it. Applying them afterwards would give a car whose numbers
% disagree with each other.
%
% They are a STUDY tool and deliberately leave no trace. To make a change
% real, edit matlab/spec/car_spec.m, where a number has to carry a source.
if nargin < 1, overrides = struct(); end
overrides = ifssim_normalise_overrides(overrides);
% Snapshot the legitimate names BEFORE any override is applied. The first pass
% creates whatever it is given, so checking after it would find the typo it
% just invented and pass it.
VALID = ifssim_valid_names(P);
P = ifssim_apply_overrides(P, overrides);
P.Overrides = overrides;

% Derived once here so no subsystem re-derives them differently.
P.Derived = struct();
P.Derived.MaxSteerAngleRad = deg2rad(P.MaxSteerAngle);
P.Derived.WheelRateEach    = P.HeaveStiffness / 4;                     % N/m per corner
P.Derived.CoGFromFrontAxle = P.Wheelbase * (1 - P.WeightDistFront);    % m
P.Derived.CoGOffsetFromMid = P.Wheelbase * (P.WeightDistFront - 0.5);  % m, -ve = rearward
P.Derived.StaticLoadFront  = P.Mass * 9.81 * P.WeightDistFront / 2;    % N per wheel
P.Derived.StaticLoadRear   = P.Mass * 9.81 * (1-P.WeightDistFront) / 2;

% Slip stiffnesses, in the form a physically-parameterised tyre model wants.
% NOT new information: they are the initial slope of the Magic Formula we
% already fitted. For y = D*sin(C*atan(B*x - ...)), dy/dx at the origin is
% D*C*B, and D is mu*Fz — so these are what our own curve already implies,
% expressed as N/rad and N per unit slip ratio instead of as shape factors.
% Derived rather than assumed on purpose: change the Pacejka fit and these
% follow, so the two descriptions of the tyre cannot drift apart.
% The tyre's reference load. NOT the car's static corner load, though it
% equals it for the car as built -- see Tyre.NominalLoad in car_spec for why
% the two were separated, and what tying them together silently cost.
P.Derived.NominalWheelLoad  = P.Assumed.TyreNominalLoad;   % N
P.Derived.CorneringStiffness = P.TireMu * P.Derived.NominalWheelLoad * ...
                               P.Pacejka.LatC * P.Pacejka.LatB;   % N/rad
P.Derived.LongSlipStiffness  = P.TireMu * P.Derived.NominalWheelLoad * ...
                               P.Pacejka.LonC * P.Pacejka.LonB;   % N per unit slip
P.Derived.PeakWheelTorque  = P.MotorMaxTorque * P.GearRatio * P.DrivetrainEfficiency;
P.Derived.UnsprungPerCorner = 10.0;                    % kg, from the wheel classes  ASSUMPTION
P.Derived.SprungPerCorner   = (P.Mass - 4*P.Derived.UnsprungPerCorner)/4;
P.Derived.RideFreqHz        = sqrt(P.Derived.WheelRateEach / P.Derived.SprungPerCorner)/(2*pi);

% ---- anti-roll bars ---------------------------------------------------
% RollStiffnessFront/Rear are the TOTAL roll stiffness of each axle. The
% springs already supply 0.5*k_wheel*track^2 of it; the bar is the remainder.
%
% Deriving it this way rather than declaring a bar rate is what keeps the
% three suspension numbers honest. If somebody raises HeaveStiffness without
% raising the roll stiffnesses, the springs eventually supply more roll
% stiffness than the axle is declared to have, the bar rate goes negative,
% and the guard below stops the build instead of quietly modelling a bar that
% pushes the car over in corners. That is exactly the state this file was in:
% HeaveStiffness 227600 needed a front bar of -13968 N*m/rad.
P.Derived.RollStiffFromSpringsFront = 0.5*P.Derived.WheelRateEach*P.TrackFront^2;
P.Derived.RollStiffFromSpringsRear  = 0.5*P.Derived.WheelRateEach*P.TrackRear^2;
P.Derived.ArbFront = P.RollStiffnessFront - P.Derived.RollStiffFromSpringsFront;
P.Derived.ArbRear  = P.RollStiffnessRear  - P.Derived.RollStiffFromSpringsRear;
if P.Derived.ArbFront < 0 || P.Derived.ArbRear < 0
    error('ifssim_params:impossibleRollStiffness', ...
          ['These three numbers cannot all be true.\n\n' ...
           '  HeaveStiffness %.0f N/m puts %.0f N/m under each corner, and springs\n' ...
           '  at that rate on a %.3f/%.3f m track already supply %.0f/%.0f N*m/rad of\n' ...
           '  roll stiffness front/rear -- but the axles are declared to have\n' ...
           '  %.0f/%.0f. An anti-roll bar can only ADD, never subtract.\n\n' ...
           '  Pick one:\n' ...
           '    raise the bars    RollStiffnessFront >= %.0f, RollStiffnessRear >= %.0f\n' ...
           '    soften the springs  HeaveStiffness <= %.0f N/m (ride %.2f Hz)\n\n' ...
           '  Roll stiffness DISTRIBUTION is what sets the balance, so if you are\n' ...
           '  changing springs to change balance, change the bars instead.'], ...
          P.HeaveStiffness, P.Derived.WheelRateEach, P.TrackFront, P.TrackRear, ...
          P.Derived.RollStiffFromSpringsFront, P.Derived.RollStiffFromSpringsRear, ...
          P.RollStiffnessFront, P.RollStiffnessRear, ...
          ceil(P.Derived.RollStiffFromSpringsFront), ceil(P.Derived.RollStiffFromSpringsRear), ...
          floor(4*2*min(P.RollStiffnessFront/P.TrackFront^2, P.RollStiffnessRear/P.TrackRear^2)), ...
          sqrt(2*min(P.RollStiffnessFront/P.TrackFront^2, P.RollStiffnessRear/P.TrackRear^2) ...
               / P.Derived.SprungPerCorner)/(2*pi));
end
% The fraction of roll stiffness at the front. This one number is the car's
% balance: raise it and the front axle takes more of the lateral transfer,
% loses more grip to load sensitivity, and the car understeers.
P.Derived.RollStiffnessFrontFraction = ...
    P.RollStiffnessFront / (P.RollStiffnessFront + P.RollStiffnessRear);

% Values the plant needs that settings.json does not carry. Separate from the
% configured parameters on purpose, so what is guessed is visible at a glance.
% README has the table of why each matters and how to measure it.
% NOT re-initialised: the Tyre.* group from car_spec already landed here as
% P.Assumed.Tyre*, and a fresh struct() would silently wipe it.

% Typical values for a ~275 kg FS car. settings.json has no inertia and UE only
% scales what its physics asset computes, so there is nothing to read.
% Izz sets yaw response, which is what the controller is tuned against.
P.Assumed.Ixx = 30.0;    % kg*m^2   roll     ASSUMPTION
P.Assumed.Iyy = 110.0;   % kg*m^2   pitch    ASSUMPTION
% Yaw inertia, as a RADIUS OF GYRATION rather than a fixed number, so that it
% follows the mass. Held fixed while lateral force scaled with mass, it made a
% heavier car answer the wheel FASTER: step-steer response to 90% measured
% 0.163 s at 200 kg and 0.083 s at 400 kg, which is backwards.
%
% 0.6742 m gives exactly 125 kg*m^2 at the 275 kg car, so nothing about today's
% car changes. It is about 43% of the wheelbase, which is the usual range for
% a single-seater.
P.Assumed.YawRadiusOfGyration = 0.6742;   % m   ASSUMPTION
P.Assumed.Izz = P.Mass * P.Assumed.YawRadiusOfGyration^2;   % kg*m^2  DERIVED
P.Assumed.InertiaSource = 'ESTIMATE — typical FS values, not measured for this car';

% Wheel+tyre. Corroborated two ways: a solid disc at 10 kg / r=0.202 gives 0.204,
% and IFS_Sim carried 0.215. Sets how fast a wheel spins up or locks.
P.Assumed.WheelInertia = 0.21;   % kg*m^2 per corner   ASSUMPTION

% Slip divides by forward speed, which is zero at rest. Slip is computed against
% max(|vx|, this). Below this speed the tyre model is not trustworthy.
P.Assumed.SlipRegularisationSpeed = 1.0;   % m/s

% The same idea one level down, for WHEEL speed. Brake and rolling-resistance
% torques oppose rotation, so they must change sign with it, and the direction
% term has to be smoothed or it chatters about zero every step.
%
% The scale is not free: it has to be wider than the wheel-speed change one
% step can produce, or the smoothing does nothing. The EBS applies 286 N.m to
% a 0.21 kg.m^2 wheel, which is 1.42 rad/s in a single 1/960 s step -- so a
% 0.1 rad/s transition, as this used to be, IS sign(), and the brake bang-bangs
% the wheel across zero at the end of every stop. 2 rad/s is wider than that
% step, which turns the last part of the stop into proportional damping
% instead of a limit cycle.
P.Assumed.WheelSpeedRegularisation = 2.0;   % rad/s

% RELAXATION LENGTH. Slip is a STATE, not an algebraic quantity: a tyre needs
% to roll a certain distance before its carcass has deformed enough to build
% the force. dkappa/dt = (|vx|/sigma) * (kappa_steady - kappa).
%
% This is physically real, and it is also what makes the model integrable at a
% fixed step. With algebraic slip the wheel is numerically stiff at launch: from
% rest, one 1/960 s step adds ~0.15 of slip ratio while the longitudinal curve
% peaks at ~0.10, so the wheel overshoots the peak before the tyre can react,
% and past the peak more slip means less force — spurious runaway wheelspin at
% throttle levels the tyre could easily have held.
%
% 0.2-0.3 m is typical for a race slick.
P.Assumed.RelaxLengthLong = 0.20;   % m   ASSUMPTION
P.Assumed.RelaxLengthLat  = 0.30;   % m   ASSUMPTION

% SLIDING vs PEAK friction. Our Magic Formula has a single mu: the force at
% full slide equals the force at the peak. Real tyres fall away past the peak,
% and the Fiala tyre block requires muMin < muMax and will not accept a single
% value at all. 0.95 is deliberately close to 1 so the swap to that block stays
% behaviour-preserving; a real slick is nearer 0.7-0.8, and moving this number
% is the single easiest way to make the car harder to catch once it lets go.
P.Assumed.SlideFrictionRatio = 0.95;   % muMin/muMax   ASSUMPTION

% The tyre's own assumptions -- pressure, rim width, mass, load sensitivity,
% stiffness peak load and reference velocity -- are declared in car_spec under
% Tyre.*, and arrive here as P.Assumed.Tyre*. They used to be hand-written in
% BOTH files, with a guard in build_car to stop the two copies drifting apart.
% There is one copy now, so there is nothing to guard.
%
% Contact width is not among them: it is the same number as WheelWidth, which
% car_spec already declares from the workbook, and two names for one width is
% how they end up disagreeing.




% Air density. Not a property of the car. Sea level, 15 C. Aero scales linearly with
% it, so a hot day at altitude is a real few percent — worth a parameter rather
% than a constant buried in the aero block.
P.Assumed.AirDensity = 1.225;      % kg/m^3   ASSUMPTION

% Height of the centre of pressure above the CoG. Drag acting above the CoG
% pitches the nose down under braking and lifts it under power. Not measured;
% zero means drag acts through the CoG and produces no pitch moment, which is
% the honest default until someone runs the CFD or the wind tunnel.
P.Assumed.CoPHeightAboveCoG = 0.0;   % m   ASSUMPTION (no drag pitch couple)

% --- EBS --------------------------------------------------------------
% Emergency brake torque per wheel, pneumatic, acting on all four corners.
%
% Sized as a MULTIPLE OF THE GRIP LIMIT rather than as an absolute number,
% because the absolute number is not what determines stopping distance. Above
% the grip limit the wheel locks, and from then on deceleration is set by the
% TYRE at full slip — not by how much more torque the caliper could apply. 1.5x
% guarantees lock-up, which is what an emergency system without ABS does.
%
% The value it replaces was Chaos's default 3000 N.m per wheel: roughly 15x the
% grip limit, chosen by nobody, and the number the entire emergency-braking case
% silently rested on.
%
% MEASURE THE REAL SYSTEM. Not because the torque matters much above the grip
% limit, but because the FILL TIME does — see SteerLagTau for the same argument.
P.Assumed.EbsGripMultiple = 1.5;     % ASSUMPTION

% Pneumatic fill time, defaulted to effectively instant. Better no lag than the
% wrong lag; this one directly flatters every stopping-distance figure.
P.Assumed.EbsFillTau = 1e-3;         % s   ASSUMPTION (effectively none)

% --- steering ---------------------------------------------------------
% 1.0 = full geometric Ackermann (inner wheel steers more, common turn centre).
% 0 = parallel steer. Real FS cars run partial or even anti-Ackermann; the
% IFS-08 steering-arm geometry is not recorded anywhere in this repo.
P.Assumed.AckermannFraction = 1.0;        % ASSUMPTION

% Actuator rate limit and lag, defaulted to effectively none. Better no lag than
% the wrong lag: the autonomy is tuned against this transient. Bench the real
% steering motor and set them.
P.Assumed.SteerRateLimit = 100.0;         % rad/s   ASSUMPTION (effectively none)
P.Assumed.SteerLagTau    = 1e-3;          % s       ASSUMPTION (effectively none)

% --- battery ----------------------------------------------------------
% THE PACK LIVES IN car_spec NOW, as a cell part number and an arrangement;
% pack_from_cells derives voltage, capacity, resistance and current from it.
% What stood here was a THIRD description of a pack -- 95s, 8.5 A*h, 0.10 ohm,
% carried over from matlab/IFS_Sim (2024-25), a model that also lists 237 kg
% and a 0.30 m tyre radius and so may not even be this car. Nothing read it
% any more except a printout, which used it to announce "399 V 3.4 kWh" about
% a 6.3 kWh accumulator. Removed rather than left to be reconnected by
% accident.
%
% Initial state of charge stays, because the Simscape pack genuinely needs it
% and it belongs to the RUN rather than to the cell.
P.Assumed.BatteryInitialSoC  = 0.9;

% CoG height above ground. settings.json declares 0.3 m; kept here as the value
% the chassis actually uses so there is one place to change it.
P.Assumed.CoGHeightUsed = P.CoGHeight;

% Assumed defaults are set above, so any override of one has to be re-applied
% now -- before the block below derives anything from them. This is also the
% first point at which a name can be CHECKED, because Assumed now exists.
% Assumed's legitimate names, MINUS anything the first pass invented. Pass one
% creates whatever it is handed, so a misspelled Assumed.* override would
% otherwise appear in this snapshot and validate itself.
invented = {};
fo = fieldnames(overrides);
for i = 1:numel(fo)
    k = strrep(fo{i}, '_DOT_', '.');
    pp = split(k,'.');
    if numel(pp) == 2 && any(strcmp(pp{1},{'Tyre','Assumed'}))
        invented{end+1} = ['Tyre' pp{2}]; %#ok<AGROW>
        invented{end+1} = pp{2};           %#ok<AGROW>
    end
end
Acl = P.Assumed;
fa = fieldnames(Acl);
for i = 1:numel(fa)
    if any(strcmp(fa{i}, invented)) && ~isAssumedDefault(fa{i})
        Acl = rmfield(Acl, fa{i});
    end
end
VALID = [VALID, ifssim_valid_names(struct('Assumed', Acl))];
P = ifssim_apply_overrides(P, overrides, VALID);

% --- suspension, derived from the declared stiffness ------------------
% settings.json SuspensionDamping is a RATIO (zeta), not a coefficient:
% c = 2*zeta*sqrt(k*m). Converting needs the corner sprung mass, so it is done here.
P.Derived.SuspensionDampingCoeff = ...
    2 * P.SuspensionDamping * sqrt(P.Derived.WheelRateEach * P.Derived.SprungPerCorner);

% Deflection reference: delta = 0 means sitting at ride height.
P.Derived.StaticSuspLength = P.CoGHeight - P.WheelRadius;

% Wf is the CoG-to-REAR distance over the wheelbase, so the front arm is the
% complement. Backwards mirrors the car's balance and is hard to spot in a lap time.
P.Derived.aFront = P.Wheelbase * (1 - P.WeightDistFront);   % m, CoG -> front axle
P.Derived.bRear  = P.Wheelbase * P.WeightDistFront;         % m, CoG -> rear axle

% --- battery, derived -------------------------------------------------
% Pack voltage, capacity and energy are derived by pack_from_cells from the
% cell and the arrangement in car_spec. They are deliberately NOT restated
% here: this file used to carry a second set, computed from a different car's
% numbers, and anything reading them got a 3.4 kWh answer about a 6.3 kWh pack.

% Regen is POWER limited long before torque limited, and by a lot. This is the
% number that actually sets braking capability.
% Downforce at a reference speed, so the number is visible rather than implied.
P.Derived.DownforceAt20ms = 0.5 * P.Assumed.AirDensity * 20^2 * P.ClA;   % N
P.Derived.DragAt20ms      = 0.5 * P.Assumed.AirDensity * 20^2 * P.CdA;   % N

% EBS torque per wheel, from the grip limit at static load.
P.Derived.EbsTorquePerWheel = P.Assumed.EbsGripMultiple * P.TireMu * ...
    (P.Mass * 9.81 / 4) * P.WheelRadius;     % N*m

P.Derived.RegenTorqueAt10ms = P.MaxRegenPower / ...
    ((10 / P.WheelRadius) * P.GearRatio);   % Nm at the motor
end

% =======================================================================
function ov = ifssim_normalise_overrides(ov)
%IFSSIM_NORMALISE_OVERRIDES  Accept a struct or name/value pairs.
if isempty(ov), ov = struct(); return; end
if iscell(ov)
    if mod(numel(ov),2) ~= 0
        error('ifssim_params:badOverrides', ...
              'overrides given as name/value pairs must come in pairs');
    end
    c = ov;  ov = struct();
    for i = 1:2:numel(c), ov.(strrep(c{i},'.','_DOT_')) = c{i+1}; end
end
if ~isstruct(ov)
    error('ifssim_params:badOverrides', ...
          'overrides must be a struct or a cell of name/value pairs');
end
end

% =======================================================================
function P = ifssim_apply_overrides(P, ov, valid)
%IFSSIM_APPLY_OVERRIDES  Set P.a.b from an override named 'a.b' or 'a_DOT_b'.
%
%   Tyre.X is accepted and resolves to Assumed.TyreX, which is where car_spec's
%   Tyre group lands. Both names appear in the docs and in car_spec, and a
%   study that silently wrote a dead field because the user typed the name the
%   documentation gave them is worse than no study.
%
%   validate=true checks the field EXISTS. It can only be done on the second
%   apply pass: the first runs before the Assumed block has been built, so
%   Assumed.Izz would not exist yet and every legitimate override of one would
%   be rejected.
if nargin < 3, valid = {}; end
f = fieldnames(ov);
for i = 1:numel(f)
    key = strrep(f{i}, '_DOT_', '.');
    parts = split(key, '.');
    if numel(parts) == 2 && strcmp(parts{1}, 'Tyre')
        parts = {'Assumed', ['Tyre' parts{2}]};     % the documented alias
    end
    switch numel(parts)
        case 1
            if ~isempty(valid) && ~any(strcmp(parts{1}, valid))
                suggest(valid, key);
            end
            P.(parts{1}) = ov.(f{i});
        case 2
            if ~isempty(valid) && ~any(strcmp([parts{1} '.' parts{2}], valid))
                suggest(valid, key);
            end
            if ~isfield(P, parts{1}), P.(parts{1}) = struct(); end
            P.(parts{1}).(parts{2}) = ov.(f{i});
        otherwise
            error('ifssim_params:badOverrideName', ...
                  '"%s" is nested too deep; use Group.Name', key);
    end
end
end

function suggest(valid, key)
%SUGGEST  Refuse an override that names nothing, and say what was meant.
tgt  = lower(strrep(key,'.',''));
pool = unique(valid(:), 'stable');
pool = pool(~startsWith(pool, 'Source.'));   % provenance mirrors, not knobs
d = cellfun(@(c) namedist(lower(strrep(c,'.','')), tgt), pool);
[~, i] = sort(d);
near = strjoin(pool(i(1:min(5,numel(i))))', ', ');
error('ifssim_params:unknownOverride', ...
      ['"%s" is not a parameter. Nothing would have changed, and the study\n' ...
       'would have returned a table of zeroes that looked like an answer.\n\n' ...
       '  did you mean: %s\n\n' ...
       '  vd_parameters        lists what the VD department owns\n' ...
       '  ifssim_params_report lists everything, with its source'], key, near);
end

% =======================================================================
function n = ifssim_valid_names(P)
%IFSSIM_VALID_NAMES  Every name an override may legitimately address.
n = {};
f = fieldnames(P);
for i = 1:numel(f)
    v = P.(f{i});
    if isstruct(v)
        g = fieldnames(v);
        for j = 1:numel(g), n{end+1} = [f{i} '.' g{j}]; end %#ok<AGROW>
        % car_spec's Tyre group lands under Assumed as TyreX; accept both.
        if strcmp(f{i},'Assumed')
            for j = 1:numel(g)
                if startsWith(g{j},'Tyre')
                    n{end+1} = ['Tyre.' extractAfter(g{j},'Tyre')]; %#ok<AGROW>
                end
            end
        end
    else
        n{end+1} = f{i}; %#ok<AGROW>
    end
end
end

% =======================================================================
function d = namedist(a, b)
%NAMEDIST  Cheap edit-distance-ish score, good enough to rank suggestions.
la = numel(a); lb = numel(b);
D = zeros(la+1, lb+1);
D(:,1) = (0:la)';  D(1,:) = 0:lb;
for i = 1:la
    for j = 1:lb
        D(i+1,j+1) = min([D(i,j+1)+1, D(i+1,j)+1, D(i,j)+(a(i)~=b(j))]);
    end
end
d = D(end,end);
end

% =======================================================================
function tf = isAssumedDefault(name)
%ISASSUMEDDEFAULT  Is this a field the Assumed block itself declares?
%
%   Listed rather than discovered, because the whole point is to tell a real
%   Assumed parameter apart from one an override invented a moment ago, and
%   both are sitting in the same struct by the time we look.
persistent L
if isempty(L)
    L = {'Ixx','Iyy','Izz','YawRadiusOfGyration','InertiaSource','WheelInertia', ...
         'SlipRegularisationSpeed','WheelSpeedRegularisation','RelaxLengthLong', ...
         'RelaxLengthLat','SlideFrictionRatio','TyrePressure','TyreRimWidth', ...
         'TyreMass','TyreNominalLoad','TyreLoadSensitivity', ...
         'TyreStiffnessPeakLoadRatio','TyreRefVelocity','AirDensity', ...
         'CoPHeightAboveCoG','EbsGripMultiple','EbsFillTau','AckermannFraction', ...
         'SteerRateLimit','SteerLagTau','BatteryInitialSoC','CoGHeightUsed'};
end
tf = any(strcmp(name, L));
end
