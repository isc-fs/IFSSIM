function C = ifs09()
%IFS09  The IFS-09, the next prototype. Starts as the IFS-08; says what is different.
%
%   EVERY VALUE HERE IS INHERITED FROM THE IFS-08 until this file says
%   otherwise, and its source says so ("INHERITED from IFS-08 -- ..."). That
%   is deliberate: a new car is not measured yet, and borrowing the old car's
%   numbers is fine as long as nothing pretends they are the new car's.
%   check_car counts what is still inherited.
%
%   TO MAKE A NUMBER THE IFS-09'S OWN, set it below with a source of its
%   own, exactly as in ifs08.m:
%
%       C = par(C,'Pack.CellsParallelPerModule', 8, '-', 'DESIGN accumulator team 2026-11: ...');
%
%   Only what differs goes here. A value that is the same on both cars stays
%   inherited -- copying it would make two copies of one number, free to drift.
%
%   WHAT IS EXPECTED TO CHANGE, so whoever fills this in knows where to start:
%     - the ACCUMULATOR's electrical configuration (cell, Pack.*): the plant's
%       Simscape Battery pack is generated from these, so a new arrangement
%       needs no code (battery_from_spec);
%     - and then, subsystem by subsystem, the rest.
%
%   Work on it with ifssim_car('IFS-09'); every tool then runs this car. Its
%   plant models and FMU build into matlab/build/cars/IFS-09/, never over the
%   IFS-08's. car_diff('IFS-08','IFS-09') lists what is different.

C = inherit(ifs08(), 'IFS-09');

%% ---- what is different about the IFS-09 --------------------------------

% ACCUMULATOR. Molicel INR-18650-P30B cells, in 12s4p modules, twelve of
% them in series: 144s4p, 576 cells. The pack the plant generates is
% IFSSIM_Battery_4p12s12m1x (battery_from_spec).
% Consequences check_car reports rather than this file hiding:
%   - 144 x 4.20 V = 604.8 V at full charge, above the FS 600 V limit on
%     the tractive system;
%   - the inherited 200 A current limit is 50 A a cell across 4p, above the
%     P30B's 30 A continuous rating.

% THE CELL: Molicel INR-18650-P30B, product data sheet Version 1.0.
% Not on the datasheet, so still inherited: the open-circuit curve (a
% generic NMC 18650 shape, top at the same 4.2 V) and the specific heat.
C = par(C,'Cell.Name',           0,      '-',  'DATASHEET Molicel INR-18650-P30B, product data sheet Version 1.0 -- the value is a placeholder; the name is the point');
C = par(C,'Cell.CapacityAh',     3.000,  'A*h','DATASHEET P30B: 3000 mAh typical (2900 minimum)');
C = par(C,'Cell.VMax',           4.20,   'V',  'DATASHEET P30B charge voltage');
C = par(C,'Cell.VNom',           3.60,   'V',  'DATASHEET P30B nominal voltage');
C = par(C,'Cell.VMin',           2.50,   'V',  'DATASHEET P30B discharge cut-off');
C = par(C,'Cell.Rint',           0.017,  'ohm','DATASHEET P30B DC impedance at 50% SoC (typical). The 8 mOhm also quoted is AC at 30% SoC, and is NOT what sags under load.');
C = par(C,'Cell.ICharacterised', 30,     'A',  'DATASHEET P30B: highest rate on its discharge curves');
C = par(C,'Cell.ISustained',     30,     'A',  'DATASHEET P30B continuous discharge rating, 80 C cut-off. Published, unlike the VTC6''s.');
C = par(C,'Cell.Mass',           0.047,  'kg', 'DATASHEET P30B: 47 g maximum');
C = par(C,'Cell.Diameter',       0.0186, 'm',  'DATASHEET P30B: 18.6 mm maximum');
C = par(C,'Cell.Height',         0.0652, 'm',  'DATASHEET P30B: 65.2 mm maximum');
C = par(C,'Pack.CellsSeriesPerModule',    12, '-','DESIGN IFS-09 accumulator: 12s4p modules (2026-10-20)');
C = par(C,'Pack.CellsParallelPerModule',   4, '-','DESIGN IFS-09 accumulator: 12s4p modules (2026-10-20)');
C = par(C,'Pack.ModulesInSeries',         12, '-','DESIGN IFS-09 accumulator: 12 modules in series -> 144s (2026-10-20)');
C = par(C,'Pack.ModulesInParallel',        1, '-','DESIGN IFS-09 accumulator: one string of 12 modules (2026-10-20)');
% Charged to 4.15 V a cell, not the P30B's own 4.20 V: 144 x 4.15 = 597.6 V,
% under the FS 600 V limit. 144 x 4.20 would be 604.8 V.
C = par(C,'Pack.CellChargeCutoff',    4.15, 'V','DESIGN IFS-09: charge cut-off 4.15 V a cell, keeping 144s at 597.6 V under the FS 600 V limit (2026-10-20)');
% THE LIMIT IS POWER, 80 kW, not a current: Pack.PowerLimit. The current
% limit is then sized so it never binds first: the current 80 kW needs at
% the pack's lowest voltage (144 cells at the 2.5 V floor, 360 V).
C = par(C,'Pack.PowerLimit',         80000, 'W','DESIGN IFS-09: 80 kW limit on the accumulator (2026-10-20), the FS one');
C = par(C,'Pack.CurrentLimit',      222.2, 'A','DERIVED so the 80 kW limit binds, never the current: 80 kW at the pack''s floor, 144 x 2.5 V = 360 V. At nominal (518 V) 80 kW is 154 A, 38.6 A a cell.');

% POWERTRAIN: Drivetrain Innovation F-SiC 2WD kit -- one F-SiC inverter block
% (two inverter modules) and two F-SiC motors, ONE PER REAR WHEEL, through a
% 12.5:1 in-wheel reduction. Figures from DTI's F-SiC product page.
% Not on the page and not given, so still inherited: the reduction's
% efficiency (DrivetrainEfficiency 0.92) and the rotor inertia (not modelled).
% The motors' 3.7 kg each sit in the rear corners; the car's total mass is
% still the IFS-08's.
C = par(C,'Motor.Count',          2,     '-',  'DESIGN IFS-09: F-SiC 2WD kit, one motor per rear wheel (2026-10-20)');
C = par(C,'GearRatio',            12.5,  '-',  'DESIGN IFS-09: 12.5:1 in-wheel reduction (2026-10-20)');
C = par(C,'MotorMaxTorque',       31.6,  'N*m','DATASHEET F-SiC motor peak torque, per motor (11 Nm nominal)');
C = par(C,'MotorMaxPower',        35000, 'W',  'DATASHEET F-SiC motor peak power, per motor (15.4 kW nominal). Two of them, 70 kW, sit under the 80 kW accumulator limit.');
C = par(C,'Motor.MaxRpm',         20000, 'rpm','DATASHEET F-SiC motor maximum speed (13 250 rpm nominal). At 12.5:1 on a 0.202 m wheel: 33.8 m/s.');
C = par(C,'Inverter.VMin',        36,    'V',  'DATASHEET F-SiC inverter operating voltage 36-620 VDC');
C = par(C,'Inverter.VMax',        620,   'V',  'DATASHEET F-SiC inverter operating voltage 36-620 VDC');
C = par(C,'MaxRegenTorque',       31.6,  'N*m','ASSUMED mirrors the drive torque, per motor (the IFS-08''s rule)');
C = par(C,'MaxRegenPower',        18662, 'W',  'DERIVED from the P30B''s 9 A maximum charge current (datasheet): 4p x 9 A = 36 A at the 518 V nominal pack. The IFS-08''s 6 kW was an assumption about the VTC6.');
end
