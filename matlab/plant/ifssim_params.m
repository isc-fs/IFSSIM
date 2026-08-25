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

% Derived once here so no subsystem re-derives them differently.
P.Derived = struct();
P.Derived.MaxSteerAngleRad = deg2rad(P.MaxSteerAngle);
P.Derived.WheelRateEach    = P.HeaveStiffness / 4;                     % N/m per corner
P.Derived.CoGFromFrontAxle = P.Wheelbase * (1 - P.WeightDistFront);    % m
P.Derived.CoGOffsetFromMid = P.Wheelbase * (P.WeightDistFront - 0.5);  % m, -ve = rearward
P.Derived.StaticLoadFront  = P.Mass * 9.81 * P.WeightDistFront / 2;    % N per wheel
P.Derived.StaticLoadRear   = P.Mass * 9.81 * (1-P.WeightDistFront) / 2;
P.Derived.PeakWheelTorque  = P.MotorMaxTorque * P.GearRatio * P.DrivetrainEfficiency;
P.Derived.UnsprungPerCorner = 10.0;                    % kg, from the wheel classes  ASSUMPTION
P.Derived.SprungPerCorner   = (P.Mass - 4*P.Derived.UnsprungPerCorner)/4;
P.Derived.RideFreqHz        = sqrt(P.Derived.WheelRateEach / P.Derived.SprungPerCorner)/(2*pi);

% Values the plant needs that settings.json does not carry. Separate from the
% configured parameters on purpose, so what is guessed is visible at a glance.
% README has the table of why each matters and how to measure it.
P.Assumed = struct();

% Typical values for a ~275 kg FS car. settings.json has no inertia and UE only
% scales what its physics asset computes, so there is nothing to read.
% Izz sets yaw response, which is what the controller is tuned against.
P.Assumed.Ixx = 30.0;    % kg*m^2   roll     ASSUMPTION
P.Assumed.Iyy = 110.0;   % kg*m^2   pitch    ASSUMPTION
P.Assumed.Izz = 125.0;   % kg*m^2   yaw      ASSUMPTION
P.Assumed.InertiaSource = 'ESTIMATE — typical FS values, not measured for this car';

% Wheel+tyre. Corroborated two ways: a solid disc at 10 kg / r=0.202 gives 0.204,
% and IFS_Sim carried 0.215. Sets how fast a wheel spins up or locks.
P.Assumed.WheelInertia = 0.21;   % kg*m^2 per corner   ASSUMPTION

% Slip divides by forward speed, which is zero at rest. Slip is computed against
% max(|vx|, this). Below this speed the tyre model is not trustworthy.
P.Assumed.SlipRegularisationSpeed = 1.0;   % m/s

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

% Air density. Not in settings.json. Sea level, 15 C. Aero scales linearly with
% it, so a hot day at altitude is a real few percent — worth a parameter rather
% than a constant buried in the aero block.
P.Assumed.AirDensity = 1.225;      % kg/m^3   ASSUMPTION

% Height of the centre of pressure above the CoG. Drag acting above the CoG
% pitches the nose down under braking and lifts it under power. Not measured;
% zero means drag acts through the CoG and produces no pitch moment, which is
% the honest default until someone runs the CFD or the wind tunnel.
P.Assumed.CoPHeightAboveCoG = 0.0;   % m   ASSUMPTION (no drag pitch couple)

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
% From matlab/IFS_Sim (2024-25), NOT settings.json. That model also carries mass
% 237 kg and tyre radius 0.30 m, which disagree with settings.json — so it may
% describe a different car. Confirm, then move these into settings.json.
P.Assumed.BatterySeriesCells = 95;
P.Assumed.BatteryCellVMax    = 4.2;      % V, fully charged
P.Assumed.BatteryCellVMin    = 3.2;      % V, empty. IFS_Sim did not record this.
P.Assumed.BatteryCapacityAh  = 8.5;      % Ah
P.Assumed.BatteryInitialSoC  = 0.9;
% Not in IFS_Sim and not measured. Only affects terminal voltage sag.
P.Assumed.BatteryResistance  = 0.10;     % ohm     ASSUMPTION

% CoG height above ground. settings.json declares 0.3 m; kept here as the value
% the chassis actually uses so there is one place to change it.
P.Assumed.CoGHeightUsed = P.CoGHeight;

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
P.Derived.BatteryVMax  = P.Assumed.BatterySeriesCells * P.Assumed.BatteryCellVMax;
P.Derived.BatteryVMin  = P.Assumed.BatterySeriesCells * P.Assumed.BatteryCellVMin;
P.Derived.BatteryAs    = P.Assumed.BatteryCapacityAh * 3600;   % amp-seconds
P.Derived.BatteryWh    = P.Derived.BatteryVMax * P.Assumed.BatteryCapacityAh;

% Regen is POWER limited long before torque limited, and by a lot. This is the
% number that actually sets braking capability.
% Downforce at a reference speed, so the number is visible rather than implied.
P.Derived.DownforceAt20ms = 0.5 * P.Assumed.AirDensity * 20^2 * P.ClA;   % N
P.Derived.DragAt20ms      = 0.5 * P.Assumed.AirDensity * 20^2 * P.CdA;   % N

P.Derived.RegenTorqueAt10ms = P.MaxRegenPower / ...
    ((10 / P.WheelRadius) * P.GearRatio);   % Nm at the motor
end
