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
    'MaxSteerAngle',        22.4, ...     % deg, road wheel (peak-grip clamp)
    'RollingResistance',    0.020, ...   % Crr, ASSUMED not measured
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

% Slip stiffnesses, in the form a physically-parameterised tyre model wants.
% NOT new information: they are the initial slope of the Magic Formula we
% already fitted. For y = D*sin(C*atan(B*x - ...)), dy/dx at the origin is
% D*C*B, and D is mu*Fz — so these are what our own curve already implies,
% expressed as N/rad and N per unit slip ratio instead of as shape factors.
% Derived rather than assumed on purpose: change the Pacejka fit and these
% follow, so the two descriptions of the tyre cannot drift apart.
P.Derived.NominalWheelLoad  = (P.Derived.StaticLoadFront + P.Derived.StaticLoadRear)/2;  % N
P.Derived.CorneringStiffness = P.TireMu * P.Derived.NominalWheelLoad * ...
                               P.Pacejka.LatC * P.Pacejka.LatB;   % N/rad
P.Derived.LongSlipStiffness  = P.TireMu * P.Derived.NominalWheelLoad * ...
                               P.Pacejka.LonC * P.Pacejka.LonB;   % N per unit slip
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

% Tyre pressure. Only used by force models that are pressure-sensitive, which
% ours are configured NOT to be — the block's nominal pressure is set equal to
% this and the pressure input to the same again, so every pressure ratio is
% exactly 1 and nothing scales. It exists so the port has something honest on
% it, not because we know the pressure. It was 220 kPa, which is not a
% "neutral" number at all: it is the shipped passenger-car set's, 32 psi,
% about three times what an FS slick runs. Inert either way, but a number
% nobody could read without being misled.
P.Assumed.TyrePressure = 83000;   % Pa (12 psi)   ASSUMPTION, not measured

% Contact width. Only reaches the overturning moment Mx, which we do not
% currently feed back into the chassis. Hoosier 16x7.5-10 is about this.
P.Assumed.TyreWidth = 0.190;   % m   ASSUMPTION

% Rim width. Reaches nothing we compute — the tyre model wants it for contact
% patch geometry, which turn slip uses and we have off. A 7.5 in tyre on a
% 7.5 in rim; the base file's 5.9 in was the passenger car's.
P.Assumed.RimWidth = 0.1905;   % m   ASSUMPTION

% Mass of the rubber alone, as distinct from P.Assumed.WheelInertia which is
% the whole rotating corner. The tyre model carries it for its own inertia
% terms, which the block's vertical model (switched off) would use.
P.Assumed.TyreMass = 5.0;   % kg   ASSUMPTION, an FS 16x7.5-10 is about this

% Reference velocity for the tyre model. NOT cosmetic: it sets how quickly
% friction falls with slip speed, mu/(1 + Vs/RefVelocity). Inherited at 16 m/s
% from a passenger-car tyre set, where it governed launch recovery in a car
% that never sees a passenger car's slip speeds. Set far above anything this
% car reaches, which disables an effect we have no measurement of.
%
% This is the ONLY handle on that effect. The scaling factor LMUV, which the
% Magic Formula spec says switches the decay off when it is zero, is written
% as zero into the parameter file and ignored by the block -- measured: with
% LMUV = 0 and this back at 16, 0-75 m at full throttle regresses 5.085 ->
% 6.745 s. Do not "correct" this to a realistic rig speed.
P.Assumed.TyreRefVelocity = 1000;   % m/s  ZEROED, see car_spec Tyre.RefVelocity

% Air density. Not in settings.json. Sea level, 15 C. Aero scales linearly with
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
