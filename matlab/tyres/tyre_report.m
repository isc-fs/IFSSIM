function K = tyre_report()
%TYRE_REPORT  The tyre, and the car on it, as numbers.
%
%       cd matlab/tyres
%       tyre_report
%
%   (plant/tyre_report is a different tool: it sweeps the plant's actual
%   Simulink tyre BLOCK and checks it against the file. This one is the
%   department's view -- what the curve is, and what it does to the car.)
%
%   The headline is the dispute. The car runs TireMu 1.40, from an
%   unvalidated curve fit; the tyres department says 1.65 for this Hoosier at
%   1000 N. Both are printed, through the same car, so the argument is about
%   a number of seconds rather than a number of decimal places.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
P = ifssim_params();
K = tyre_kpis(P);
M = K.M;  Fz0 = M.Fz0;

fprintf('\n============================ TYRES ===========================\n');
fprintf('  nominal load %.0f N (Tyre.NominalLoad)\n\n', Fz0);
fprintf('  %-8s %10s %14s\n', 'load', 'peak mu', 'Ca [N/deg]');
Kya = @(Fz) -M.PKY1*Fz0*sin(M.PKY4*atan(Fz./(M.PKY2*Fz0))) * pi/180;
for f = [0.25 0.5 1 1.5 2]
    Fz = f*Fz0;
    fprintf('  %4.2f Fz0 %10.3f %14.1f\n', f, M.PDY1 + M.PDY2*(f-1), Kya(Fz));
end
fprintf('\n  peak slip angle at nominal load   %.1f deg\n', K.alpha_peak);
fprintf('  grip left once sliding            %.0f%% lateral, %.0f%% longitudinal\n', ...
        100*K.tail_lat, 100*K.tail_lon);
fprintf('  longitudinal slip stiffness       %.0f N per unit slip\n', K.Kx_nom);

fprintf('\n  THE CAR ON IT\n');
fprintf('    lap                %.2f s\n', K.lap);
fprintf('    skid pad           %.3f s   (vd_skidpad: the full design model, slowly)\n', K.skidpad);
fprintf('    cornering limit    %.3f g at 8 m/s, %.3f g at 22 m/s\n', K.aylim_8, K.aylim_22);
fprintf('    understeer K       %+.3f deg/g at low lateral\n', K.understeer);
fprintf('    launch             %.3f g;  75 m %.3f s\n', K.launch_g, K.t75);

% ---- the dispute -------------------------------------------------------
Kd = tyre_kpis(ifssim_params({'TireMu', 1.65}));
fprintf('\n  THE DISPUTE: TireMu 1.40 (car) against 1.65 (tyres department)\n');
fprintf('  %-22s %10s %10s %9s\n', '', '1.40', '1.65', 'change');
row('lap [s]',            K.lap,       Kd.lap);
row('skid pad [s]',       K.skidpad,   Kd.skidpad);
row('limit at 22 m/s [g]', K.aylim_22, Kd.aylim_22);
row('launch [g]',         K.launch_g,  Kd.launch_g);
row('75 m [s]',           K.t75,       Kd.t75);
fprintf('  A rig test settles a %.1f s-a-lap question.\n', K.lap - Kd.lap);
fprintf('==============================================================\n');
end

function row(name, a, b)
fprintf('  %-22s %10.3f %10.3f %+8.2f%%\n', name, a, b, 100*(b-a)/abs(a));
end
