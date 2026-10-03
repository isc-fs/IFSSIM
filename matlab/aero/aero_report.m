function R = aero_report()
%AERO_REPORT  What the aero does, and what it is worth.
%
%       cd matlab/aero
%       aero_report
%
%   The headline is the TRADE: how much drag a unit of downforce may cost
%   before the lap gets slower. Every aero concept is an argument about that
%   ratio, and it is a property of the CAR and the TRACK, not of the wing --
%   a heavier car, a tighter track or a weaker motor each move it. It is
%   computed here by differencing the lap time in ClA and in CdA.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
P = ifssim_params();
K = aero_kpis(P);

fprintf('\n============================ AERO ============================\n');
fprintf('  ClA %.2f  CdA %.2f  L/D %.2f  balance %.1f%% front (weight %.1f%%)\n', ...
        P.ClA, P.CdA, K.LD, 100*K.aero_front, 100*K.weight_front);
fprintf('  at 15 m/s: %.0f N downforce (%.1f%% of the car''s weight), %.0f N drag\n', ...
        K.downforce15, 100*K.downforce15/(P.Mass*9.81), K.drag15);

B = lap_balance(P, [5 10 15 20 25 30]');
G = K.G;
fprintf('\n  cornering limit, steady state\n');
fprintf('  %6s %10s %12s %10s %9s\n', 'v m/s', 'pooled g', 'balanced g', 'goes first', 'F/R ratio');
for k = 1:numel(B.v)
    fprintf('  %6.0f %10.3f %12.3f %10s %9.3f\n', B.v(k), interp1(G.v, G.ay0_pooled, B.v(k))/9.81, ...
            B.ay_lim(k)/9.81, B.limits(k), B.ratio(k));
end
fprintf('  pooled = all four tyres; balanced = what yaw balance lets the car use,\n');
fprintf('  and what the lap uses. The gap is grip a balance change would free.\n');

fprintf('\n  lap (lap_track, %.0f m)       %.2f s   mean %.1f km/h\n', K.L.length, K.lap, K.lap_mean_kmh);
fprintf('    pack energy              %.0f kJ   of which drag %.0f kJ (%.0f%%)\n', ...
        K.lap_E_kJ, K.lap_drag_kJ, 100*K.lap_drag_kJ/max(K.lap_E_kJ,eps));
fprintf('    corner / accel / brake   %.0f / %.0f / %.0f %% of the distance\n', ...
        100*mean(K.L.limit=="corner"), 100*mean(K.L.limit=="accel"), 100*mean(K.L.limit=="brake"));
fprintf('  75 m %.3f s, top speed %.1f m/s (pt_model)\n', K.t75, K.v_top);

% ---- the trade --------------------------------------------------------
h = 0.05;
dT_dCl = (aero_kpis(ifssim_params({'ClA', P.ClA*(1+h)})).lap - ...
          aero_kpis(ifssim_params({'ClA', P.ClA*(1-h)})).lap) / (2*h*P.ClA);
Kp = aero_kpis(ifssim_params({'CdA', P.CdA*(1+h)}));
Km = aero_kpis(ifssim_params({'CdA', P.CdA*(1-h)}));
dT_dCd = (Kp.lap - Km.lap) / (2*h*P.CdA);
dE_dCd = (Kp.lap_E_kJ - Km.lap_E_kJ) / (2*h*P.CdA);
breakeven = -dT_dCl / dT_dCd;

fprintf('\n  THE TRADE, on this car and this track\n');
fprintf('    +0.1 m^2 ClA            %+.3f s a lap\n', 0.1*dT_dCl);
fprintf('    +0.1 m^2 CdA            %+.3f s a lap, %+.1f kJ a lap\n', 0.1*dT_dCd, 0.1*dE_dCd);
fprintf('    break-even              %.2f m^2 of drag per m^2 of downforce\n', breakeven);
fprintf('  A package change is FASTER if it adds less than %.2f of CdA per unit of\n', breakeven);
fprintf('  ClA -- an incremental L/D above %.2f. The current package''s average L/D\n', 1/breakeven);
fprintf('  is %.2f; it is the INCREMENT that has to clear the bar, not the average.\n', K.LD);
fprintf('  Endurance is scored on energy too: drag costs %.1f kJ per 0.1 m^2 per lap.\n', 0.1*dE_dCd);
fprintf('==============================================================\n');

R = K;
R.dT_dClA = dT_dCl;  R.dT_dCdA = dT_dCd;  R.dE_dCdA = dE_dCd;  R.breakeven = breakeven;
end
