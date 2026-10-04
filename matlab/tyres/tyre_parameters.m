function T = tyre_parameters()
%TYRE_PARAMETERS  The tyre department's parameters, and what each one moves.
%
%       cd matlab/tyres
%       tyre_parameters
%
%   Same two computed columns as the other departments: REACHES (spec_reach)
%   and +10% MOVES (tyre_kpis). Read the MOVES column against the provenance
%   column. Nobody has put this tyre on a rig, so every row is a guess, and
%   the order of the moves is the order in which to stop guessing.
%
%   Three parameters are inert ON PURPOSE -- Tyre.Pressure, Tyre.RimWidth and
%   Tyre.RefVelocity reach only terms that are zeroed or switched off -- and
%   this table should say DOES NOT BIND for them. If one of them ever starts
%   to move something, a zeroed term has been switched back on.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), fullfile(here,'..','vd'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
C = car_spec();

SPEC = {
'TireMu'                    'peak'     'peak friction at nominal load: everything'
'Tyre.LoadSensitivity'      'peak'     'what load transfer COSTS. -0.15 here; a slick is likelier -0.3 to -0.6'
'Tyre.NominalLoad'          'peak'     'the load the coefficients belong to'
'Tyre.StiffnessPeakLoadRatio' 'stiffness' 'how cornering stiffness grows with load'
'Pacejka.LatB'              'stiffness' 'with LatC and mu: cornering stiffness'
'Pacejka.LatC'              'shape'    'peak slip angle and the sliding tail'
'Pacejka.LatE'              'shape'    'how sharp the peak is'
'Pacejka.LonB'              'stiffness' 'longitudinal slip stiffness'
'Pacejka.LonC'              'shape'    'longitudinal tail'
'Pacejka.LonE'              'shape'    'longitudinal peak sharpness'
'Susp.CamberGripSensitivity' 'camber'  'grip lost per degree of camber away from static'
'RollingResistance'         'losses'   'a constant drag'
'WheelRadius'               'geometry' 'overall gearing, and the tyre size'
'Tyre.Pressure'             'inert'    'zeroed pressure terms: should NOT bind'
'Tyre.RimWidth'             'inert'    'turn slip is off: should NOT bind'
'Tyre.RefVelocity'          'inert'    'slip-speed friction decay disabled: should NOT bind'
};
n = size(SPEC,1);

KPI = {'lap','lap'; 'skidpad','skid'; 'aylim_22','ay@22'; 'understeer','K'; ...
       'launch_g','launch'; 't75','75 m'; 'mu_1p5','mu@1.5Fz0'; 'Ca_nom','Ca'; 'alpha_peak','a_pk'};
K0 = tyre_kpis();
R  = spec_reach(SPEC(:,1));
moves = dept_moves(SPEC(:,1), @tyre_kpis, KPI, K0);

fprintf('\n============================ TYRES ===========================\n');
fprintf('  car: %s\n', ifssim_car());
fprintf('  %-27s %9s %-8s %-10s %-9s %s\n','PARAMETER','VALUE','UNIT','PROVENANCE','ROLE','+10% MOVES');
for i = 1:n
    f = C.Fields.(strrep(SPEC{i,1},'.','_'));
    fprintf('  %-27s %9.4g %-8s %-10s %-9s %s\n', SPEC{i,1}, f.value, f.unit, ...
            prov_class(f.source), SPEC{i,2}, moves{i});
end
fprintf(['\n  lap %.2f s;  skid = skid pad lap, %.3f s;  ay@22 = yaw-balanced limit;\n' ...
         '  K = understeer gradient at low lateral, %+.3f deg/g (negative: oversteer);\n' ...
         '  Ca = cornering stiffness at nominal load; a_pk = peak slip angle\n'], ...
        K0.lap, K0.skidpad, K0.understeer);

fprintf('\n---- where each one reaches (spec_reach) ----------------------\n');
for i = 1:n
    fprintf('  %-27s design model: %-4s plant: %s\n', SPEC{i,1}, R.DesignModel{i}, R.Plant{i});
end

fprintf('\n---- READ THIS BEFORE TRUSTING A RESULT -----------------------\n');
inert = SPEC(strcmp(SPEC(:,2),'inert'),1);
woke = inert(~strcmp(moves(strcmp(SPEC(:,2),'inert')), 'DOES NOT BIND'));
if isempty(woke)
    fprintf('  The three inert parameters do not bind, as intended.\n\n');
else
    fprintf('  WARNING: %s should be inert and MOVED something. A zeroed\n', strjoin(woke, ', '));
    fprintf('  term has been switched back on; find it before trusting this table.\n\n');
end
fprintf('  NOBODY HAS PUT THIS TYRE ON A RIG. TireMu 1.40 is an unvalidated fit\n');
fprintf('  (the tyres department says 1.65); every shape coefficient is ASSUMED.\n');
fprintf('  Load sensitivity is the dangerous one: -0.15 is mild for a slick, and\n');
fprintf('  mild FLATTERS the car -- it under-charges every newton of transfer.\n\n');
fprintf('  CAMBER reaches the lap through body roll: the share the geometry does\n');
fprintf('  not recover costs peak grip (Susp.CamberGripSensitivity). K is taken at\n');
fprintf('  low lateral, where the car barely rolls, so it sees little of it.\n\n');
fprintf('  The longitudinal shape (Lon*) reaches the car through slip on the\n');
fprintf('  power limit (pt_model), so it moves the 75 m time and little else.\n\n');
fprintf('  The lap engine sees only the PEAK of the curve. Shape coefficients\n');
fprintf('  (C, E) reach the car through K, the understeer gradient, which the\n');
fprintf('  design model computes on the full curve. A shape change that moves K\n');
fprintf('  and not the lap is a handling change, not a speed change.\n');
fprintf('==============================================================\n');
fprintf('  tyre_report      the curves and the car on them\n');
fprintf('  tyre_plots       the same, as figures\n');
fprintf('  tyre_study(...)  try a change\n\n');

T = table(SPEC(:,1), SPEC(:,2), moves, R.Reaches, ...
          'VariableNames', {'Parameter','Role','Moves','Reaches'});
end
