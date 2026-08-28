function C = car_spec()
%CAR_SPEC  THE car. Every number the simulator runs on, and where it came from.
%
%   THIS IS THE FILE YOU EDIT. Nothing else. settings.json is generated from
%   it, the Simulink plant is built from it, and the FMU the UE5 simulator
%   loads is exported from that. Change a number here and run BUILD_CAR, and
%   every one of those follows.
%
%   Each entry is  par(name, value, unit, source)  and the SOURCE IS NOT
%   OPTIONAL. It is the whole point of this file. Three of the numbers below
%   were wrong for years because nobody could say where they came from, and
%   the only way that gets caught is if every value has to answer the
%   question in writing.
%
%   Write one of these in the source, honestly:
%
%     MEASURED   somebody put an instrument on this car. Say what and when.
%     GEOMETRY   read off the IFS-08 hardpoint table. Say which points.
%     DERIVED    computed from other entries here. Say the formula.
%     ASSUMED    a typical value for a car like this. Nobody measured it.
%     DISPUTED   sources disagree. Say who says what.
%     UNKNOWN    nobody knows. This is a legitimate answer and a useful one.
%
%   See also BUILD_CAR, CHECK_CAR.

C = struct('Name','IFS-08','Fields',struct(),'Order',{{}});

%% ---- chassis geometry -------------------------------------------------
% The wheelbase and track are read straight off the IFS-08 hardpoint table:
% front wheel centre F15 at x=160 mm, rear R15 at x=1730 mm, both at y=600.
% They are not design intent -- they are where the wheels are.
C = par(C,'Wheelbase',      1.570, 'm',  'GEOMETRY Susp_Geometry F15/R15 wheel centres: (1730-160)/1000');
C = par(C,'TrackFront',     1.200, 'm',  'GEOMETRY Susp_Geometry F15 wheel centre y=600 mm, both sides');
C = par(C,'TrackRear',      1.200, 'm',  'GEOMETRY Susp_Geometry R15 wheel centre y=600 mm, both sides');

% 0.438 is the as-built car on scales, and it beats the drawing. The IFS-08
% hardpoint CoG x of 865.7 mm implies 0.4486; a 2% gap between design intent
% and the car that got built is normal, and the scales win.
C = par(C,'WeightDistFront',0.438, '-',  'MEASURED corner weights FL 43.5 FR 48.5 RL 62 RR 56 kg -> 92/210');

% DISPUTED, and it matters: CoG height sets how much load moves under braking.
C = par(C,'CoGHeight',      0.300, 'm',  'DISPUTED 0.300 is from the MONO block labelled IFS-06/07; the VD dept file and the C++ default both say 0.3441. Nobody has measured the IFS-08.');
C = par(C,'Mass',           275,   'kg', 'DISPUTED 275 here; corner weights total 210 kg for the car, and the VD dept file says 290 kg with driver. 210+65 = 275 is consistent but nobody wrote that down.');

%% ---- wheel and tyre --------------------------------------------------
C = par(C,'WheelRadius',    0.202, 'm',  'MEASURED-ish calibrated in PR #54/#56 and consumed by the pipeline odometry. Workbook says 203.2 mm unloaded, VD dept says 200 mm dynamic. 202 sits between and must not move without changing kRpmToMs in the pipeline.');
C = par(C,'WheelWidth',     0.190, 'm',  'GEOMETRY workbook tyre block: 19.05 cm');
C = par(C,'TireMu',         1.40,  '-',  'DISPUTED 1.40 comes from an unvalidated curve fit; the tyres dept says 1.65 at 1000 N for the Hoosier 16.0x7.5-10. Nobody has put this tyre on a rig.');
C = par(C,'RollingResistance',0.020,'-', 'ASSUMED typical for a warm slick. Tyres dept says 0.015. Settled by decision, not measurement.');

%% ---- suspension ------------------------------------------------------
% UNKNOWN and demonstrably suspect: this implies a 4.95 Hz ride frequency,
% where the workbook records 2.9-3.0 Hz beside a 24-25 N/mm wheel rate. That
% wheel rate is the IFS-06/07 car, so it cannot simply be adopted -- but ours
% agrees with nothing at all. CHECK_CAR warns about this every build.
C = par(C,'HeaveStiffness', 227600.0,'N/m','UNKNOWN no source anywhere. Implies 4.95 Hz ride frequency; the only recorded figure is 2.9-3.0 Hz, for IFS-06/07.');
C = par(C,'PitchStiffness', 155600.0,'N/m','UNKNOWN no source anywhere');
C = par(C,'RollStiffnessFront',27000.0,'N*m/rad','UNKNOWN no source anywhere');
C = par(C,'RollStiffnessRear', 22000.0,'N*m/rad','UNKNOWN no source anywhere');
C = par(C,'RollCenterFront', 0.040, 'm','ASSUMED Susp_Geometry has the hardpoints these should be computed from');
C = par(C,'RollCenterRear',  0.060, 'm','ASSUMED Susp_Geometry has the hardpoints these should be computed from');
C = par(C,'SuspensionDamping',1.5,  '-','ASSUMED damping ratio, never measured');

%% ---- steering --------------------------------------------------------
% Four sources have disagreed about maximum lock; see issue #462. This is the
% tyre's peak-grip slip angle rather than a mechanical limit, chosen because
% steering past peak grip only ever loses grip.
C = par(C,'MaxSteerAngle',   22.4, 'deg','DERIVED peak-grip slip angle of the tyre curve, via characterise_plant. See #462: uDV says 18.2 deg, the workbook implies 19.25 deg from an IFS-07 wheelbase.');

%% ---- powertrain ------------------------------------------------------
C = par(C,'GearRatio',       2.909,'-',  'MEASURED workbook MONO ratio 2.909, matches motor/wheel speeds quoted beside it');
C = par(C,'MotorMaxTorque',  230,  'N*m','DISPUTED workbook says 220 N*m peak; 230 is what the sim has always run');
C = par(C,'MotorMaxPower',   80000,'W',  'MEASURED rules limit 80 kW, workbook agrees');
C = par(C,'MaxRegenTorque',  230,  'N*m','ASSUMED mirrors drive torque; regen is the only service brake on this car');
C = par(C,'MaxRegenPower',   6000, 'W',  'ASSUMED battery cell input-current limit');
C = par(C,'DrivetrainEfficiency',0.92,'-','MEASURED workbook quotes 92-98%; the low end taken');

%% ---- aerodynamics ----------------------------------------------------
C = par(C,'CdA',             0.90, 'm^2','UNKNOWN no CFD or tunnel run is recorded');
C = par(C,'ClA',             1.80, 'm^2','UNKNOWN no CFD or tunnel run is recorded');
C = par(C,'AeroBalanceFront',0.45, '-',  'ASSUMED slightly rearward of the weight distribution');

%% ---- accumulator: the cell -------------------------------------------
% Sony/Murata US18650VTC6. Datasheet figures, with the caveats stated:
% the continuous-current rating is genuinely disputed (Sony quote 30 A with
% a 80 C cut-off, which is not a rating you can design to), and DC internal
% resistance is not the 1 kHz AC figure usually printed.
C = par(C,'Cell.Name',            0,      '-',  'DATASHEET Sony/Murata US18650VTC6 -- the value here is a placeholder; the name is the point');
C = par(C,'Cell.CapacityAh',      3.000,  'A*h','DATASHEET 3000 mAh nominal (2800 minimum)');
C = par(C,'Cell.VMax',            4.20,   'V',  'DATASHEET charge cut-off');
C = par(C,'Cell.VNom',            3.60,   'V',  'DATASHEET nominal');
C = par(C,'Cell.VMin',            2.50,   'V',  'ASSUMED datasheet says 2.0 V; 2.5 is the usual design floor and is kinder to cycle life');
C = par(C,'Cell.Rint',            0.020,  'ohm','ASSUMED DC internal resistance. The 13 mOhm usually quoted is the 1 kHz AC figure and is NOT what sags under load.');
C = par(C,'Cell.IMaxCont',        20,     'A',  'DISPUTED Sony quote 30 A with an 80 C cut-off, which is not a designable rating. 20 A is the figure the pack should be sized on.');
C = par(C,'Cell.IMaxPulse',       30,     'A',  'DATASHEET absolute maximum, seconds only');
C = par(C,'Cell.Mass',            0.0467, 'kg', 'DATASHEET 46.7 g');

%% ---- accumulator: the topology ---------------------------------------
% THIS IS THE PART THAT IS MEANT TO CHANGE. Every pack quantity the plant
% uses -- voltage, capacity, resistance, current and power limits -- is
% DERIVED from these four numbers and the cell above, so trying a different
% arrangement is editing four integers, not hunting for pack values that
% were computed once by hand and pasted somewhere.
%
% A different cell is the block above. A different arrangement is here.
C = par(C,'Pack.CellsSeriesPerModule',   19, '-','MEASURED accumulator team: each module is 19s6p');
C = par(C,'Pack.CellsParallelPerModule',  6, '-','MEASURED accumulator team: each module is 19s6p');
C = par(C,'Pack.ModulesInSeries',         5, '-','MEASURED 5 modules in series -> 95s overall');
C = par(C,'Pack.ModulesInParallel',       1, '-','MEASURED modules are in series only');

% WHAT THE CAR ACTUALLY DRAWS, which is not the same as what the cells are
% rated for and must not be confused with it. Logged on the car at up to
% 200 A. Across six parallel that is 33 A per cell, ABOVE the VTC6's 30 A
% maximum -- which is what a short burst looks like, because a cell does not
% refuse current. The rating is a statement about heat and cycle life, not a
% wall the current cannot cross.
%
% This is the number the torque envelope uses. The cell ratings above are
% what check_car compares it against, so exceeding them is visible rather
% than silently modelled as impossible.
C = par(C,'Pack.CurrentLimit',         200, 'A','MEASURED logged on the car, bursts to 200 A');

%% ---- tyre curve ------------------------------------------------------
% Shape factors, not measurements. LonC sets how much grip survives at full
% slide: sin(C*pi/2), so 1.46 -> 75%. It was 1.7 (45%) until the Magic
% Formula swap showed the car could neither launch nor stop properly.
C = par(C,'Pacejka.LatB',   10.0,  '-',  'ASSUMED curve fit, never validated against tyre data');
C = par(C,'Pacejka.LatC',    1.40, '-',  'ASSUMED gives an 81% sliding tail, inside the 70-85% a slick is quoted at');
C = par(C,'Pacejka.LatE',   -0.30, '-',  'ASSUMED curve fit, never validated against tyre data');
C = par(C,'Pacejka.LonB',   13.973,'-',  'DERIVED holds slip stiffness at 19262 N when LonC moved: B = Ck/(C*mu*Fz)');
C = par(C,'Pacejka.LonC',    1.46, '-',  'DERIVED chosen so sin(C*pi/2) = 0.75, a 75% sliding tail');
C = par(C,'Pacejka.LonE',   -0.50, '-',  'ASSUMED curve fit, never validated against tyre data');
end

%% =======================================================================
function C = par(C, name, value, unit, source)
%PAR  One parameter, with the source it came from. Source is mandatory.
if isempty(strtrim(source))
    error('car_spec:noSource','%s has no source. Write UNKNOWN if that is the truth.', name);
end
key = strrep(name,'.','_');
C.Fields.(key) = struct('name',name,'value',value,'unit',unit,'source',source);
C.Order{end+1} = key;
end
