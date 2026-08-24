function P = ifssim_params(settingsPath)
%IFSSIM_PARAMS  Load the vehicle parameters from IFSSIM's settings.json.
%
%   P = IFSSIM_PARAMS()            reads <repo>/settings.json
%   P = IFSSIM_PARAMS(path)        reads a specific file
%
%   settings.json IS THE SOURCE OF TRUTH. Not a copy of it, not a set of
%   numbers retyped into a script — the same file the simulator itself parses.
%   That is the entire point: the plant model and the simulator must not be
%   able to disagree about what car they are simulating.
%


if nargin < 1 || isempty(settingsPath)
    here = fileparts(mfilename('fullpath'));
    settingsPath = fullfile(here, '..', '..', 'settings.json');
end
settingsPath = char(settingsPath);
if ~isfile(settingsPath)
    error('ifssim_params:notFound', 'settings.json not found at %s', settingsPath);
end

raw = jsondecode(fileread(settingsPath));

% Locate the vehicle block. settings.json nests as Vehicles.<name>.VehiclePhysics
phys = struct();
vehicleName = '';
if isfield(raw,'Vehicles')
    names = fieldnames(raw.Vehicles);
    if ~isempty(names)
        vehicleName = names{1};
        v = raw.Vehicles.(vehicleName);
        if isfield(v,'VehiclePhysics'), phys = v.VehiclePhysics; end
    end
end
if isempty(vehicleName)
    warning('ifssim_params:noVehicle', ...
        'no Vehicles block in %s — every parameter will fall back to defaults', settingsPath);
end

% ---------------------------------------------------------------------
% Defaults MIRROR the C++ struct. If FSDSSettings.h changes, change these
% and say so in the same commit — a silent divergence here reintroduces
% precisely the class of bug this function exists to prevent.
% Source: Plugins/FSDSPlugin/Source/FSDSPlugin/Public/FSDSSettings.h
%         Plugins/FSDSPlugin/Source/FSDSPlugin/Public/FSDSPacejkaTireModel.h
% ---------------------------------------------------------------------
D = struct( ...
    'Mass',                 290.0, ...    % kg
    'WheelRadius',          0.200, ...    % m
    'WheelWidth',           0.190, ...    % m
    'MaxSteerAngle',        28.0, ...     % deg, road wheel
    'MotorMaxTorque',       230.0, ...    % Nm at motor
    'MotorMaxPower',        80000.0, ...  % W
    'MaxRegenTorque',       230.0, ...    % Nm at motor, negative side
    'MaxRegenPower',        6000.0, ...   % W, cell input current limit
    'GearRatio',            2.909, ...
    'DrivetrainEfficiency', 0.92, ...
    'CdA',                  0.95, ...     % m^2
    'ClA',                  3.0, ...      % m^2, positive = downforce
    'AeroBalanceFront',     0.45, ...
    'TireMu',               1.65, ...
    'WeightDistFront',      0.438, ...
    'CoGHeight',            0.344, ...    % m
    'SuspensionDamping',    1.5, ...      % damping RATIO (zeta)
    'Wheelbase',            1.627, ...    % m
    'TrackFront',           1.220, ...    % m
    'TrackRear',            1.190, ...    % m
    'RollCenterFront',      0.040, ...    % m
    'RollCenterRear',       0.060, ...    % m
    'RollStiffnessFront',   27000.0, ...  % Nm/rad
    'RollStiffnessRear',    22000.0, ...  % Nm/rad
    'HeaveStiffness',       227600.0, ... % N/m, SUM of 4 wheel rates
    'PitchStiffness',       155600.0);    % Nm/rad

P = struct(); P.Source = struct();
f = fieldnames(D);
for i = 1:numel(f)
    k = f{i};
    if isfield(phys, k) && isnumeric(phys.(k)) && isscalar(phys.(k))
        P.(k) = double(phys.(k));  P.Source.(k) = 'settings.json';
    else
        P.(k) = D.(k);             P.Source.(k) = 'default';
    end
end

% Pacejka lives in its own nested block.
PD = struct('LatB',10.0,'LatC',1.9,'LatE',-1.5,'LonB',12.0,'LonC',1.7,'LonE',-0.5);
pf = fieldnames(PD);
pj = struct();
if isfield(phys,'Pacejka'), pj = phys.Pacejka; end
P.Pacejka = struct();
for i = 1:numel(pf)
    k = pf{i};
    if isfield(pj,k) && isnumeric(pj.(k)) && isscalar(pj.(k))
        P.Pacejka.(k) = double(pj.(k)); P.Source.(['Pacejka_' k]) = 'settings.json';
    else
        P.Pacejka.(k) = PD.(k);         P.Source.(['Pacejka_' k]) = 'default';
    end
end

P.SettingsPath = settingsPath;
P.VehicleName  = vehicleName;

% ---------------------------------------------------------------------
% DERIVED quantities, computed here so no subsystem re-derives them
% differently. Each is a place a second copy could otherwise appear.
% ---------------------------------------------------------------------
P.Derived = struct();
P.Derived.MaxSteerAngleRad = deg2rad(P.MaxSteerAngle);
% Wheel rate per corner, from the declared SUM of four.
P.Derived.WheelRateEach    = P.HeaveStiffness / 4;                     % N/m
% CoG measured from the front axle, and as an offset from the wheelbase
% midpoint (negative = rearward), which is what the sim's CoM override uses.
P.Derived.CoGFromFrontAxle = P.Wheelbase * (1 - P.WeightDistFront);    % m
P.Derived.CoGOffsetFromMid = P.Wheelbase * (P.WeightDistFront - 0.5);  % m
P.Derived.StaticLoadFront  = P.Mass * 9.81 * P.WeightDistFront / 2;    % N per wheel
P.Derived.StaticLoadRear   = P.Mass * 9.81 * (1-P.WeightDistFront) / 2;
% Peak torque referred to the wheel, through the single reduction.
P.Derived.PeakWheelTorque  = P.MotorMaxTorque * P.GearRatio * P.DrivetrainEfficiency;
% Unsprung mass is not in settings.json; 10 kg/corner is the wheel-class
% figure. Flagged as an assumption rather than silently folded in.
P.Derived.UnsprungPerCorner = 10.0;                                    % kg  ASSUMPTION
P.Derived.SprungPerCorner   = (P.Mass - 4*P.Derived.UnsprungPerCorner)/4;
P.Derived.RideFreqHz        = sqrt(P.Derived.WheelRateEach / P.Derived.SprungPerCorner)/(2*pi);

% ---------------------------------------------------------------------
% ASSUMPTIONS. Values the plant needs that settings.json does not carry.
% Kept in a separate struct, not mixed in with configured parameters, so a
% reader can see at a glance what is measured and what is guessed.
% ---------------------------------------------------------------------
P.Assumed = struct();

% INERTIA TENSOR — the largest single unknown in this model.
%
% settings.json has no inertia. The simulator only has UE's
% InertiaTensorScale = (1.0, 1.4, 1.1), which SCALES whatever the physics asset
% happens to compute from the mesh — so there is no authoritative number to read
% and none to copy.
%
% These are typical measured values for a ~275 kg Formula Student car, and the
% ratios agree with the shape UE's scale implies (pitch > yaw > roll).
%
%   Ixx  roll   — smallest: the car is narrow and low
%   Iyy  pitch  — largest: mass spread fore and aft along the wheelbase
%   Izz  yaw    — dominates the transient the autonomy actually feels
%
% Yaw inertia is the one that matters most here: it sets how quickly the car
% responds to a steering input, which is precisely what the controller is tuned
% against. A 20 % error in Izz is a 20 % error in yaw response, and it will be
% mistaken for a controller gain problem.
%
% MEASURE THESE. A bifilar pendulum test, or a CAD mass-properties export, and
% then move them into settings.json so they stop being an assumption. Until
% then, treat any lateral-transient result from this model as provisional.
P.Assumed.Ixx = 30.0;    % kg*m^2   roll     ASSUMPTION
P.Assumed.Iyy = 110.0;   % kg*m^2   pitch    ASSUMPTION
P.Assumed.Izz = 125.0;   % kg*m^2   yaw      ASSUMPTION
P.Assumed.InertiaSource = 'ESTIMATE — typical FS values, not measured for this car';

% Rotational inertia of one wheel+tyre assembly. Not in settings.json. The wheel
% classes give 10 kg per corner; a solid disc at r=0.202 would be 0.5*m*r^2 =
% 0.204, and the 2024-25 IFS_Sim model carried 0.215 for wheel+tyre. They agree,
% which is weak but real corroboration.
%
% This sets how fast a wheel can spin up or lock. Too large and the car will not
% wheelspin when it should; too small and it locks under trivial brake torque.
P.Assumed.WheelInertia = 0.21;   % kg*m^2 per corner   ASSUMPTION

% Slip regularisation speed. Slip ratio and slip angle both divide by forward
% speed, which is zero at standstill. Dividing by max(|vx|, this) keeps the
% tyre model finite at rest instead of producing Inf on the first step.
% 1 m/s is the usual choice; below it the tyre model is not to be trusted
% anyway, and a launch-from-rest study should say so rather than quietly
% believing the number.
P.Assumed.SlipRegularisationSpeed = 1.0;   % m/s

% --- steering ---------------------------------------------------------
% ACKERMANN FRACTION. 1.0 = full geometric Ackermann, where the inner wheel
% steers more so both front wheels roll about a common centre. 0 = parallel
% steer. Real FS cars are usually somewhere between, and some run deliberate
% ANTI-Ackermann because a loaded outer tyre peaks at a larger slip angle.
%
% The IFS-08's actual steering-arm geometry is not recorded anywhere in this
% repo, so this is an assumption. It is a defensible one: full Ackermann is
% physically motivated, and it is a large improvement on what the simulator was
% doing, which was Chaos's default AngleRatio 0.7 — REVERSE Ackermann, giving
% the inner wheel LESS angle than the outer. Nobody chose that either.
%
% Measure it from the steering arms and set it here.
P.Assumed.AckermannFraction = 1.0;        % ASSUMPTION

% STEERING ACTUATOR. A rate limit and a first-order lag, both defaulted to
% effectively instantaneous.
%
% This is deliberate and follows the rule applied when Chaos's hidden 0.4 s
% steering rate limit was removed: better NO lag than the WRONG lag. An
% unmeasured actuator model produces confident, wrong transient behaviour, and
% the autonomy is tuned against exactly that transient.
%
% The real DV steering motor does have a finite slew rate, and the bench data
% shows the wheel angle sensor diverging from the command by ~22.9 deg mean —
% which is either a calibration error or real actuator lag, and nobody has
% separated the two yet. When that is resolved, set these and re-tune.
P.Assumed.SteerRateLimit = 100.0;         % rad/s   ASSUMPTION (effectively none)
P.Assumed.SteerLagTau    = 1e-3;          % s       ASSUMPTION (effectively none)

% --- battery ----------------------------------------------------------
% HARVESTED FROM matlab/IFS_Sim (the 2024-25 drive-cycle model), not from
% settings.json. Provenance matters here: that model also carried mass 237 kg
% and tyre radius 0.30 m, both of which disagree with settings.json, so it is
% evidently a different car or a different year. The pack numbers are probably
% still right, but "probably" is the operative word.
%
%   n_series 19, n_stacks 5, max_cellV 4.2  ->  95 cells in series, 399 V
%   Battery_Capacity 8.5 Ah, initial SoC 0.9
%
% Move these into settings.json once someone confirms they describe THIS car.
P.Assumed.BatterySeriesCells = 95;
P.Assumed.BatteryCellVMax    = 4.2;      % V, fully charged
P.Assumed.BatteryCellVMin    = 3.2;      % V, empty. IFS_Sim did not record this.
P.Assumed.BatteryCapacityAh  = 8.5;      % Ah
P.Assumed.BatteryInitialSoC  = 0.9;
% Pack internal resistance. NOT in IFS_Sim and not measured. It only affects
% terminal voltage sag, which nothing downstream currently consumes, but a
% voltage that never sags is a battery model that will flatter any power study.
P.Assumed.BatteryResistance  = 0.10;     % ohm     ASSUMPTION

% CoG height above ground. settings.json declares 0.3 m; kept here as the value
% the chassis actually uses so there is one place to change it.
P.Assumed.CoGHeightUsed = P.CoGHeight;

% --- suspension, derived from the declared stiffness ------------------
% SuspensionDamping in settings.json is a damping RATIO (zeta), not a
% coefficient. Converting it needs the sprung mass at the corner, so it is done
% here once rather than in whichever subsystem needs it first.
%   c = 2 * zeta * sqrt(k * m)
P.Derived.SuspensionDampingCoeff = ...
    2 * P.SuspensionDamping * sqrt(P.Derived.WheelRateEach * P.Derived.SprungPerCorner);

% Static suspension length: CoG height minus wheel radius. At rest the corner
% attachment sits at CoG height and the wheel centre one radius above the road,
% so this is the deflection reference — delta = 0 means sitting at ride height.
P.Derived.StaticSuspLength = P.CoGHeight - P.WheelRadius;

% Longitudinal wheel positions from the CoG. Front axle load fraction Wf is the
% ratio of the CoG-to-REAR distance to the wheelbase, so the front arm is the
% complement. Getting this backwards mirrors the car's balance and is very hard
% to spot from a lap time.
P.Derived.aFront = P.Wheelbase * (1 - P.WeightDistFront);   % m, CoG -> front axle
P.Derived.bRear  = P.Wheelbase * P.WeightDistFront;         % m, CoG -> rear axle

% --- battery, derived -------------------------------------------------
P.Derived.BatteryVMax  = P.Assumed.BatterySeriesCells * P.Assumed.BatteryCellVMax;
P.Derived.BatteryVMin  = P.Assumed.BatterySeriesCells * P.Assumed.BatteryCellVMin;
P.Derived.BatteryAs    = P.Assumed.BatteryCapacityAh * 3600;   % amp-seconds
P.Derived.BatteryWh    = P.Derived.BatteryVMax * P.Assumed.BatteryCapacityAh;

% Regen is POWER limited long before it is torque limited. MaxRegenPower is the
% cell input-current cap; at any real speed it binds first, and by a lot. Worth
% having as a number rather than a surprise: at 10 m/s the motor turns about
% v/Rw*GearRatio rad/s, so the available regen torque is MaxRegenPower divided
% by that, which is a small fraction of the 230 Nm envelope.
P.Derived.RegenTorqueAt10ms = P.MaxRegenPower / ...
    ((10 / P.WheelRadius) * P.GearRatio);   % Nm at the motor
end
