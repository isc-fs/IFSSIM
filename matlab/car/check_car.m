function ok = check_car(C)
%CHECK_CAR  Cross-check the car against itself, and say what is not known.
%
%   Every error found in the August 2026 audit was a CONSISTENCY failure that
%   nothing was looking for: a wheelbase that disagreed with the hardpoints, a
%   suspension whose stiffness implied a ride frequency nobody had ever
%   recorded, a tyre curve whose sliding tail was half what a slick holds.
%   None of them were typos. Each was a number that made sense alone and did
%   not survive being compared with another number.
%
%   So this compares them. It cannot tell you a value is RIGHT -- only the car
%   can do that -- but it can tell you two values cannot both be right.
%
%   ok = CHECK_CAR(C) returns false if anything failed. Warnings do not fail
%   the build; they are things worth knowing every time you build.

if nargin < 1, C = car_spec(); end
v = @(n) C.Fields.(strrep(n,'.','_')).value;   % dotted names are stored flattened
ok = true; nwarn = 0;

fprintf('\n=== CHECK_CAR: %s ===\n', C.Name);

%% ---- consistency ------------------------------------------------------
% Ride frequency. This is the check that would have caught HeaveStiffness.
sprung  = v('Mass') * 0.85 / 4;              % ~15% unsprung, per corner
wheelHz = sqrt(v('HeaveStiffness')/4/sprung)/(2*pi);
[ok,nwarn] = band(ok,nwarn,'ride frequency', wheelHz,'Hz', 1.5, 4.0, ...
    'A Formula Student car sits around 2.5-3.5 Hz. Outside that band either the stiffness or the mass is wrong.');

% Sliding tail. sin(C*pi/2) is the fraction of peak grip a Magic Formula
% curve keeps once the tyre is properly sliding.
for ax = {'Lat','Lon'}
    Cs = v(['Pacejka_' ax{1} 'C']);
    [ok,nwarn] = band(ok,nwarn,sprintf('%s sliding tail',ax{1}), 100*sin(Cs*pi/2),'%', 60, 90, ...
        'A slick is usually quoted holding 70-85% of peak at full slide.');
end

% Weight distribution against the geometry it implies.
aF = v('Wheelbase') * (1 - v('WeightDistFront'));
[ok,nwarn] = band(ok,nwarn,'CoG behind front axle', aF*1000,'mm', 700, 950, ...
    'The IFS-08 hardpoint table puts it at 865.7 mm.');

% Track against wheelbase: a very wide or very narrow car is a units error.
[ok,nwarn] = band(ok,nwarn,'track / wheelbase', v('TrackFront')/v('Wheelbase'),'-', 0.6, 0.9, ...
    'Outside this, one of the two is probably in the wrong units or from a different car.');

% Aero balance should sit near the weight distribution, or the car changes
% balance with speed in a way nobody intended.
[ok,nwarn] = band(ok,nwarn,'aero balance - weight dist', ...
    abs(v('AeroBalanceFront')-v('WeightDistFront')),'-', 0, 0.10, ...
    'A large gap means the car is deliberately speed-sensitive. Deliberate is fine; accidental is not.');

% Maximum lock, as a plausibility bound only.
%
% NOT compared against the tyre's peak slip angle, though it is tempting and
% this check did exactly that at first. They are different quantities: lock is
% a ROAD WHEEL angle the driver commands, slip angle is the angle the tyre
% actually runs at, and it depends on how the car is moving. A car can sit at
% 22 deg of lock with the tyre at 8 deg of slip. There is no invariant tying
% them, and asserting one flagged a settled decision as a fault.
[ok,nwarn] = band(ok,nwarn,'max steering lock', v('MaxSteerAngle'),'deg', 12, 32, ...
    'Outside the range an FS car can physically steer. See #462 for the four figures that have disagreed about this.');

% The tyre's peak slip angle is REPORTED, not asserted -- it is worth knowing
% every build, because the whole curve shape hangs off it.
B = v('Pacejka_LatB'); Cy = v('Pacejka_LatC'); E = v('Pacejka_LatE');
a  = linspace(0, 0.5, 4000);
mf = sin(Cy*atan(B*a - E*(B*a - atan(B*a))));
[~,i] = max(mf);
fprintf('  [info] %-28s %8.1f deg  (reported, not asserted)\n','tyre peak slip angle', a(i)*180/pi);

% THE ACCUMULATOR AGAINST THE MOTOR. The pack is built from a cell part
% number and an arrangement, so what it can actually deliver is derived, not
% asserted -- and it has to be at least what the motor is allowed to draw.
K = pack_from_cells(C);
fprintf('\n--- accumulator ---\n');
fprintf('  %d cells: %ds%dp in %d modules, %.0f V max, %.1f A*h, %.2f kWh, %.1f kg\n', ...
        K.NCells, K.Ns, K.Np, K.NModules, K.VMax, K.CapacityAh, K.EnergyWh/1000, K.Mass);
fprintf('  internal resistance %.3f ohm   (cell Rint * Ns / Np)\n', K.Rint);
fprintf('  cell ratings allow  %.1f kW continuous, %.1f kW pulse (after its own sag)\n', ...
        K.PMaxCont/1000, K.PMaxPulse/1000);
fprintf('  the car draws       %.0f A -> %.1f kW electrical, %.1f A per cell\n', ...
        K.IOperating, K.POperating/1000, K.CellAmpsAtOperating);
% FS ACCUMULATOR SEGMENT RULES. A segment may not exceed 120 V maximum or
% 6 MJ, which is usually what decides how the pack is split in the first
% place -- so if a proposed arrangement breaks them, it is not a pack.
Vseg = v('Pack.CellsSeriesPerModule') * v('Cell.VMax');
Eseg = v('Pack.CellsSeriesPerModule') * v('Pack.CellsParallelPerModule') * ...
       v('Cell.CapacityAh') * v('Cell.VNom') * 3600 / 1e6;      % MJ
[ok,nwarn] = band(ok,nwarn,'segment voltage', Vseg,'V', 0, 120, ...
    'FS rules cap an accumulator segment at 120 V maximum.');
[ok,nwarn] = band(ok,nwarn,'segment energy',  Eseg,'MJ', 0, 6, ...
    'FS rules cap an accumulator segment at 6 MJ.');

% THE OPERATING CURRENT AGAINST THE CELLS' OWN RATING. Being over it is not
% an error -- cells deliver what is asked and the rating is about heat and
% life -- but it should never be invisible.
[ok,nwarn] = band(ok,nwarn,'cell amps at operating limit', K.CellAmpsAtOperating,'A', 0, v('Cell.IMaxPulse'), ...
    sprintf(['The car draws %.0f A, which is %.1f A per cell against a %.0f A ' ...
             'maximum. Fine in bursts, not something to hold.'], ...
            K.IOperating, K.CellAmpsAtOperating, v('Cell.IMaxPulse')));

[ok,nwarn] = band(ok,nwarn,'pack pulse power / motor power', K.POperating/v('MotorMaxPower'), '-', 1.0, 4.0, ...
    sprintf(['The motor is allowed %.0f kW and the accumulator can deliver %.1f kW. ' ...
             'A pack that cannot feed the motor means the motor figure is fiction, ' ...
             'or the pack is bigger than car_spec says.'], v('MotorMaxPower')/1000, K.POperating/1000));

%% ---- what nobody knows ------------------------------------------------
lvl = struct('UNKNOWN',{{}},'DISPUTED',{{}},'ASSUMED',{{}});
for i = 1:numel(C.Order)
    f = C.Fields.(C.Order{i});
    tok = regexp(f.source,'^(UNKNOWN|DISPUTED|ASSUMED)','match','once');
    if ~isempty(tok), lvl.(tok){end+1} = f.name; end
end
fprintf('\n--- provenance ---\n');
fprintf('  %d parameters: %d unknown, %d disputed, %d assumed, %d sourced\n', ...
    numel(C.Order), numel(lvl.UNKNOWN), numel(lvl.DISPUTED), numel(lvl.ASSUMED), ...
    numel(C.Order)-numel(lvl.UNKNOWN)-numel(lvl.DISPUTED)-numel(lvl.ASSUMED));
for t = {'UNKNOWN','DISPUTED'}
    if ~isempty(lvl.(t{1}))
        fprintf('  %-9s %s\n', t{1}, strjoin(lvl.(t{1}), ', '));
    end
end

%% ---- things that reach outside this repo -------------------------------
fprintf('\n--- couplings outside this repo ---\n');
fprintf('  WheelRadius %.3f m is used by the PIPELINE to turn motor rpm into\n', v('WheelRadius'));
fprintf('  road speed (kRpmToMs). Changing it here and not there puts a silent\n');
fprintf('  bias into /odom that no test in this repo would catch.\n');

fprintf('\n%s\n', ternary(ok, sprintf('CHECK_CAR PASSED (%d warning(s))', nwarn), 'CHECK_CAR FAILED'));
end

%% =======================================================================
function [ok,nwarn] = band(ok, nwarn, name, value, unit, lo, hi, why)
if value >= lo && value <= hi
    fprintf('  [ok  ] %-28s %8.3f %-4s in [%g, %g]\n', name, value, unit, lo, hi);
else
    nwarn = nwarn + 1;
    fprintf('  [WARN] %-28s %8.3f %-4s OUTSIDE [%g, %g]\n', name, value, unit, lo, hi);
    fprintf('         %s\n', why);
end
end

function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
