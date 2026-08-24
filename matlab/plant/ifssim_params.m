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
%   This project has already been burned twice by exactly that failure. The
%   Chaos suspension ran at 25 kN/m per corner while the parametric
%   load-transfer model used 56.9 kN/m — the same car, two stiffnesses, 2.3x
%   apart, for an unknown length of time, because each model held its own
%   copy of the number. Separately, IFS_Sim (the 2024-25 Simulink drive-cycle
%   model) carries mass 237 kg and tyre radius 0.30 m against settings.json's
%   275 kg and 0.202 m. A retyped parameter is a divergence with a delay fuse.
%
%   Every field records WHERE it came from, in P.Source.<field>:
%       'settings.json'  the file said so
%       'default'        the file was silent; this mirrors the C++ default in
%                        FSDSSettings.h / FSDSPacejkaTireModel.h
%   so a model built on defaults cannot be mistaken for one built on
%   configured values. Call IFSSIM_PARAMS_REPORT(P) to print the breakdown.

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
    'Mass',                 290.0, ...    % kg   (settings.json says 275)
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
end
