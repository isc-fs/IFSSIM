function P = ifssim_load_workspace()
%IFSSIM_LOAD_WORKSPACE  Put parameters and buses where Simulink can see them.
%
%   Call before building, compiling or simulating any plant model.
%
%   Simulink resolves data types and block parameters from the BASE workspace,
%   so everything a model needs has to be there first. Two forms are provided,
%   for two different consumers:
%
%     IFSSIM_P            the full struct, with provenance and derived values.
%                         Use it in mask expressions and block parameters.
%
%     IFSSIM_Mass, ...    flat scalars. MATLAB Function blocks resolve workspace
%                         variables as tunable PARAMETERS, and that resolution
%                         does not reach into nested structs — 'IFSSIM_P.Assumed.Izz'
%                         inside a MATLAB Function block fails to determine its
%                         type at compile time. The flat names are the workaround.
%
%   These are NOT a second copy of the numbers. They are assigned here from
%   ifssim_params(), which reads settings.json, so there is still exactly one
%   source and it is still the file. Do not hand-edit them.

P = ifssim_params();
% The accumulator is a cell part number and an arrangement, in
% matlab/spec/car_spec; everything the plant needs about it is derived.
addpath(fullfile(fileparts(mfilename('fullpath')),'..','spec'));
ifssim_workdir();   % keep Simulink's cache out of the repo
PK = pack_from_cells(car_spec());
assignin('base','IFSSIM_P', P);
ifssim_plant_buses();

flat = struct( ...
    'IFSSIM_Ts',    1/960, ...            % s, the plant's fixed step
    'IFSSIM_Mass',  P.Mass, ...           % kg
    'IFSSIM_Ixx',   P.Assumed.Ixx, ...    % kg*m^2  ASSUMPTION
    'IFSSIM_Iyy',   P.Assumed.Iyy, ...
    'IFSSIM_Izz',   P.Assumed.Izz, ...
    'IFSSIM_CoGH',  P.Assumed.CoGHeightUsed, ...  % m
    ... % --- geometry ---
    'IFSSIM_aF',    P.Derived.aFront, ...         % m, CoG -> front axle
    'IFSSIM_bR',    P.Derived.bRear, ...          % m, CoG -> rear axle
    'IFSSIM_tF',    P.TrackFront, ...             % m
    'IFSSIM_tR',    P.TrackRear, ...              % m
    'IFSSIM_Rw',    P.WheelRadius, ...            % m
    ... % --- suspension ---
    'IFSSIM_kw',    P.Derived.WheelRateEach, ...  % N/m per corner
    'IFSSIM_cw',    P.Derived.SuspensionDampingCoeff, ...  % N*s/m
    'IFSSIM_arbF',  P.Derived.ArbFront, ...                % N*m/rad, anti-roll bar
    'IFSSIM_arbR',  P.Derived.ArbRear, ...                 % N*m/rad
    'IFSSIM_L0',    P.Derived.StaticSuspLength, ...        % m
    'IFSSIM_FzF',   P.Derived.StaticLoadFront, ...         % N per front wheel
    'IFSSIM_FzR',   P.Derived.StaticLoadRear, ...          % N per rear wheel
    ... % --- tyre ---
    'IFSSIM_mu',    P.TireMu, ...
    'IFSSIM_LatB',  P.Pacejka.LatB, ...
    'IFSSIM_LatC',  P.Pacejka.LatC, ...
    'IFSSIM_LatE',  P.Pacejka.LatE, ...
    'IFSSIM_LonB',  P.Pacejka.LonB, ...
    'IFSSIM_LonC',  P.Pacejka.LonC, ...
    'IFSSIM_LonE',  P.Pacejka.LonE, ...
    'IFSSIM_Iw',    P.Assumed.WheelInertia, ...   % kg*m^2  ASSUMPTION
    'IFSSIM_Crr',   P.RollingResistance, ...                 % rolling resistance coeff
    'IFSSIM_vreg',  P.Assumed.SlipRegularisationSpeed, ... % m/s
    'IFSSIM_wreg',  P.Assumed.WheelSpeedRegularisation, ... % rad/s
    'IFSSIM_sigk',  P.Assumed.RelaxLengthLong, ...          % m  ASSUMPTION
    'IFSSIM_siga',  P.Assumed.RelaxLengthLat, ...           % m  ASSUMPTION
    ... % --- tyre, as the Fiala block wants it stated ---
    'IFSSIM_Calpha', P.Derived.CorneringStiffness, ...      % N/rad, from our Pacejka
    'IFSSIM_Ckappa', P.Derived.LongSlipStiffness, ...       % N per unit slip
    'IFSSIM_muMax',  P.TireMu, ...
    'IFSSIM_muMin',  P.TireMu * P.Assumed.SlideFrictionRatio, ...   % ASSUMPTION
    'IFSSIM_Ppres',  P.Assumed.TyrePressure, ...            % Pa, neutral
    'IFSSIM_Twidth', P.WheelWidth, ...               % m  ASSUMPTION
    ... % --- steering ---
    'IFSSIM_dmax',  P.Derived.MaxSteerAngleRad, ...          % rad at the road wheel
    'IFSSIM_L',     P.Wheelbase, ...                         % m
    'IFSSIM_ack',   P.Assumed.AckermannFraction, ...         % ASSUMPTION
    'IFSSIM_drate', P.Assumed.SteerRateLimit, ...            % rad/s ASSUMPTION
    'IFSSIM_dtau',  P.Assumed.SteerLagTau, ...               % s     ASSUMPTION
    ... % --- powertrain ---
    'IFSSIM_gr',    P.GearRatio, ...
    'IFSSIM_eta',   P.DrivetrainEfficiency, ...
    'IFSSIM_Tmax',  P.MotorMaxTorque, ...        % Nm at the motor
    'IFSSIM_Pmax',  P.MotorMaxPower, ...         % W
    'IFSSIM_Treg',  P.MaxRegenTorque, ...        % Nm at the motor, negative side
    'IFSSIM_Preg',  P.MaxRegenPower, ...         % W, cell input current limit
    ... % --- accumulator, derived from the cell and the arrangement ---
    ... % See matlab/spec/pack_from_cells. These come from the CELL PART NUMBER
    ... % and how many are arranged which way, so a different cell or a
    ... % different module count moves them without anyone editing a pack figure.
    ... % The MEASURED operating limit, not the sum of the cell ratings. Cells
    ... % deliver what is asked of them; the rating is about heat and cycle
    ... % life. The car has been logged at 200 A, which is above the cells'
    ... % own 30 A -- check_car says so rather than the plant pretending it
    ... % cannot happen.
    'IFSSIM_Ipk',   PK.IOperating, ...          % A, what the car actually draws
    'IFSSIM_Icont', PK.IMaxCont, ...            % A, what the cells are rated to hold
    ... % --- battery (harvested from IFS_Sim, NOT settings.json) ---
    ... % Vmax/Vmin/BAs/Rint are gone: the Simscape pack owns open-circuit
    ... % voltage, capacity and resistance now, and exporting a second set from
    ... % a different car's numbers is how they came to disagree.
    'IFSSIM_SoC0',  P.Assumed.BatteryInitialSoC, ...
    ... % --- aero ---
    'IFSSIM_rho',   P.Assumed.AirDensity, ...   % kg/m^3  ASSUMPTION
    'IFSSIM_CdA',   P.CdA, ...                  % m^2
    'IFSSIM_ClA',   P.ClA, ...                  % m^2, positive = downforce
    'IFSSIM_abal',  P.AeroBalanceFront, ...     % fraction of downforce on the front
    'IFSSIM_hcop',  P.Assumed.CoPHeightAboveCoG, ...  % m  ASSUMPTION
    ... % --- brakes ---
    'IFSSIM_Tebs',  P.Derived.EbsTorquePerWheel, ...  % N*m per wheel
    'IFSSIM_ebstau',P.Assumed.EbsFillTau);            % s  ASSUMPTION

f = fieldnames(flat);
for i = 1:numel(f), assignin('base', f{i}, flat.(f{i})); end

% Zeroed bus structs, referenced by name from the placeholder Constant blocks.
for b = {'IFSSIM_PoseBus','IFSSIM_WheelsBus','IFSSIM_PowertrainBus','IFSSIM_StatusBus'}
    assignin('base',[b{1} '_zero'], Simulink.Bus.createMATLABStruct(b{1}));
end

% A pose bus of ZEROS is not a valid car: it puts the chassis at z=0 (buried,
% because the corners sit at CoG height) and gives it a zero quaternion, which
% has no norm and therefore no orientation. Used as the initial condition of the
% pose feedback delay it produces a 17.8 kN suspension spike on step one and
% throws the car into the air before it has done anything.
%
% This is the pose the plant should START from: at ride height, level, at rest.
poseInit = Simulink.Bus.createMATLABStruct('IFSSIM_PoseBus');
poseInit.position = [0; 0; P.CoGHeight];
poseInit.quat     = [1; 0; 0; 0];          % identity, NOT zeros
assignin('base','IFSSIM_PoseBus_init', poseInit);
end
