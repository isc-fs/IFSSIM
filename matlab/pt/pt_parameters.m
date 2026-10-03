function T = pt_parameters()
%PT_PARAMETERS  Every parameter the powertrain depends on, and whether it MATTERS.
%
%       cd matlab/pt
%       pt_parameters
%
%   Two computed columns, because "wired" and "matters" are different claims
%   and a department needs both:
%
%   REACHES (spec_reach) -- is anything in the code wired to it? A static
%   trace, the same one vd_parameters uses.
%
%   BINDS (pt_model, perturbed) -- on the car as it is TODAY, does moving it
%   10% move any number this department is judged on? A parameter can be fully
%   wired and still never bind, because a min() somewhere picks a different
%   limit every time. That is not a bug, it is a design fact -- and it is the
%   most useful thing this table says. MotorMaxPower is the example: the motor
%   is rated 80 kW and the pack at its current limit gives 59, so a bigger
%   motor buys nothing and the accumulator is where the next kilowatt is.
%
%   The binding answer is for THIS car. Change the pack and MotorMaxPower may
%   start to bind; that is what pt_study is for.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
C = car_spec();

% name, group, what it changes
SPEC = {
'GearRatio'            'owned'      'trades launch torque against motor speed at the top'
'MotorMaxTorque'       'owned'      'the torque ceiling; only binds once traction stops binding'
'MotorMaxPower'        'owned'      'the motor''s power ceiling, against the pack''s'
'DrivetrainEfficiency' 'owned'      'every drive newton and every regen newton, both ways'
'MaxRegenTorque'       'owned'      'regen ceiling at low speed'
'MaxRegenPower'        'owned'      'regen ceiling at speed -- binds first, by a lot'
'Pack.CurrentLimit'    'accumulator' 'with the pack voltage, THE power limit of this car'
'Cell.Rint'            'accumulator' 'voltage sag at the current limit, so power'
'Pack.CellsSeriesPerModule' 'accumulator' 'pack voltage, so power at a fixed current'
'Mass'                 'chassis'    'every acceleration on this page'
'WheelRadius'          'chassis'    'the overall ratio, together with GearRatio'
'CoGHeight'            'chassis'    'rear load under acceleration, so launch traction'
'WeightDistFront'      'chassis'    'static rear load, so launch traction'
'TireMu'               'tyres'      'launch traction'
'CdA'                  'aero'       'top speed and the pack-limited end of the run'
'ClA'                  'aero'       'traction at speed, and rolling resistance'
'RollingResistance'    'tyres'      'a constant drag; matters at top speed'
};
n = size(SPEC,1);

E0 = pt_model();
KPI = {'t_accel','75 m'; 'v_top','v top'; 'launch_g','launch'; 'regen10','regen@10'; 'regen20','regen@20'};
k0 = kpis(E0);

R = spec_reach(SPEC(:,1));
binds = dept_moves(SPEC(:,1), @(P) kpis(pt_model(P)), KPI, k0);

fprintf('\n========================= POWERTRAIN =========================\n');
fprintf('  %-26s %9s %-6s %-11s %-12s %s\n','PARAMETER','VALUE','UNIT','PROVENANCE','GROUP','+10% MOVES');
for i = 1:n
    f = C.Fields.(strrep(SPEC{i,1},'.','_'));
    fprintf('  %-26s %9.4g %-6s %-11s %-12s %s\n', SPEC{i,1}, f.value, f.unit, ...
            provclass(f.source), SPEC{i,2}, binds{i});
end

fprintf('\n---- where each one reaches (spec_reach) ----------------------\n');
for i = 1:n
    fprintf('  %-26s %s\n', SPEC{i,1}, plant_reach(R(i,:)));
end

fprintf('\n---- READ THIS BEFORE TRUSTING A RESULT -----------------------\n');
dead = SPEC(strcmp(binds,'DOES NOT BIND'),1);
if ~isempty(dead)
    fprintf('  ON THIS CAR, THESE DO NOT BIND (computed just now):\n    %s\n', strjoin(dead, ', '));
    fprintf('  Wired, read, and still nothing moves, because a min() picks a\n');
    fprintf('  different limit at every speed. Changing them changes nothing until\n');
    fprintf('  whatever IS binding is moved out of the way.\n\n');
end
if any(strcmp(dead,'GearRatio'))
    fprintf('  GearRatio does not bind because the PLANT HAS NO MOTOR SPEED LIMIT,\n');
    fprintf('  not because the ratio is free. Its real trade -- launch torque against\n');
    fprintf('  hitting the rev limit -- is invisible until one is added. This car\n');
    fprintf('  reaches %.0f rpm at its %.1f m/s top speed in the model.\n\n', E0.top_rpm, E0.v_top);
end
fprintf('  DrivetrainEfficiency is charged ONCE, at the gears: road power is\n');
fprintf('  eta = %.2f of pack power. The motor and inverter are LOSSLESS in the\n', ...
        C.Fields.DrivetrainEfficiency.value);
fprintf('  plant -- an assumption; a sourced motor efficiency would be a\n');
fprintf('  separate parameter. (Until 2026-10 it was charged twice, eta^2.)\n\n');
fprintf('  The power limit is the PACK: %.1f kW at the shaft at %.0f%% SoC\n', E0.shaft_kW, 100*E0.soc);
fprintf('  (%.0f V under %.0f A), against a %.0f kW motor.\n', E0.Vterm, ...
        C.Fields.Pack_CurrentLimit.value, C.Fields.MotorMaxPower.value/1000);
fprintf('  The accumulator parameters bind here, in the design model. They do\n');
fprintf('  NOT reach the plant through a study override: build_battery_pack reads\n');
fprintf('  car_spec directly. pt_study runs them; plant runs refuse them.\n\n');
fprintf('  The launch assumes PERFECT traction control (tyre held at its peak).\n');
fprintf('  The plant at full throttle has none; pt_report shows what that costs.\n');
fprintf('==============================================================\n');
fprintf('  pt_report       what the powertrain does\n');
fprintf('  pt_plots        the same, as figures\n');
fprintf('  pt_study(...)   try a change\n\n');

T = table(SPEC(:,1), SPEC(:,2), binds, R.Reaches, ...
          'VariableNames', {'Parameter','Group','Binds','Reaches'});
end

% -----------------------------------------------------------------------
function k = kpis(E)
k.t_accel  = E.t_accel;
k.v_top    = E.v_top;
k.launch_g = E.launch_g;
k.regen10  = E.regen_g_at(10);
k.regen20  = E.regen_g_at(20);
end

function s = plant_reach(r)
if startsWith(r.Reaches{1}, 'NOT STUDYABLE')
    s = sprintf('plant: %s -- wired, but no study override reaches it', r.Plant{1});
elseif strcmp(r.Plant{1}, '-')
    s = ['plant: nothing. ' r.Reaches{1}];
else
    s = sprintf('plant: %s', r.Plant{1});
end
end

function c = provclass(src)
w = upper(strtok(src));
known = {'MEASURED','MEASURED-ISH','GEOMETRY','DERIVED','DATASHEET', ...
         'SECONDARY','ASSUMED','DISPUTED','ZEROED','UNKNOWN'};
if any(strcmp(w, known)), c = w; else, c = 'UNKNOWN'; end
if strcmp(c,'MEASURED-ISH'), c = 'MEASURED~'; end
end
