function T = aero_parameters()
%AERO_PARAMETERS  The aero department's parameters, and what each one moves.
%
%       cd matlab/aero
%       aero_parameters
%
%   Same two computed columns as pt_parameters: REACHES (spec_reach, is it
%   wired) and MOVES (aero_kpis, +10%: what changes, and by how much). The
%   neighbours -- weight distribution, CoG height, grip -- are listed because
%   aero numbers only mean something against them: a 45% front aero balance
%   is rearward on a 50% car and forward on a 44% one.
%
%   Every aero number on this car is UNKNOWN or ASSUMED. No CFD run or tunnel
%   test is recorded. So this table is about SENSITIVITY -- which unknown is
%   worth measuring first -- not about the car's actual aero.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
C = car_spec();

SPEC = {
'ClA'              'owned'     'downforce: corner grip, and rolling resistance'
'CdA'              'owned'     'drag: straights, top speed, energy per lap'
'AeroBalanceFront' 'owned'     'which axle the downforce loads: LIMIT BALANCE with speed'
'WeightDistFront'  'chassis'   'what the aero balance is measured against'
'CoGHeight'        'chassis'   'load transfer, which downforce partly offsets'
'Mass'             'chassis'   'downforce per kg is what grip sees'
'TireMu'           'tyres'     'grip per newton of downforce'
};
n = size(SPEC,1);

K0 = aero_kpis();
KPI = {'lap','lap'; 'aylim_22','ay@22'; 'ratio_22','bal@22'; 't75','75 m'; ...
       'v_top','v top'; 'lap_E_kJ','E/lap'};
R = spec_reach(SPEC(:,1));
moves = dept_moves(SPEC(:,1), @aero_kpis, KPI, K0);

fprintf('\n============================ AERO ============================\n');
fprintf('  %-17s %8s %-5s %-10s %-8s %s\n','PARAMETER','VALUE','UNIT','PROVENANCE','GROUP','+10% MOVES');
for i = 1:n
    f = C.Fields.(strrep(SPEC{i,1},'.','_'));
    fprintf('  %-17s %8.4g %-5s %-10s %-8s %s\n', SPEC{i,1}, f.value, f.unit, ...
            provclass(f.source), SPEC{i,2}, moves{i});
end
fprintf('\n  lap = %.2f s on lap_track;  ay@22 = yaw-balanced limit at 22 m/s;\n', K0.lap);
fprintf('  bal@22 = front/rear axle capacity ratio there (>1: the rear goes first)\n');

fprintf('\n---- where each one reaches (spec_reach) ----------------------\n');
for i = 1:n
    fprintf('  %-17s design model: %-4s plant: %s\n', SPEC{i,1}, R.DesignModel{i}, R.Plant{i});
end

fprintf('\n---- READ THIS BEFORE TRUSTING A RESULT -----------------------\n');
fprintf('  NO AERO NUMBER ON THIS CAR IS MEASURED. CdA and ClA are UNKNOWN,\n');
fprintf('  the balance is ASSUMED. Read the table as "which one matters most",\n');
fprintf('  which is what decides what to measure first.\n\n');
fprintf('  The maps are CONSTANT: no ride-height, pitch, roll or yaw\n');
fprintf('  sensitivity. A real package moves its balance as the car pitches\n');
fprintf('  under braking, which is exactly when the balance matters.\n\n');
fprintf('  Aero balance %.1f%% front against %.1f%% front weight: downforce loads\n', ...
        100*K0.aero_front, 100*K0.weight_front);
fprintf('  the front relatively MORE, so the rear runs out first and earlier as\n');
fprintf('  speed rises (bal@8 %.3f, bal@22 %.3f). Limit oversteer, growing.\n', K0.ratio_8, K0.ratio_22);
fprintf('==============================================================\n');
fprintf('  aero_report      the numbers, and the drag-for-downforce trade\n');
fprintf('  aero_plots       the same, as figures\n');
fprintf('  aero_study(...)  try a change\n\n');

T = table(SPEC(:,1), SPEC(:,2), moves, R.Reaches, ...
          'VariableNames', {'Parameter','Group','Moves','Reaches'});
end

function c = provclass(src)
w = upper(strtok(src));
known = {'MEASURED','MEASURED-ISH','GEOMETRY','DERIVED','DATASHEET', ...
         'SECONDARY','ASSUMED','DISPUTED','ZEROED','UNKNOWN'};
if any(strcmp(w, known)), c = w; else, c = 'UNKNOWN'; end
if strcmp(c,'MEASURED-ISH'), c = 'MEASURED~'; end
end
