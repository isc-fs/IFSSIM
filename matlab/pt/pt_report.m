function R = pt_report(withPlant)
%PT_REPORT  What the powertrain does, as numbers.
%
%       cd matlab/pt
%       pt_report            design model only, instant
%       pt_report('plant')   also runs the acceleration event on the full
%                            Simulink plant (~1 min) and sets the two side by side
%
%   Everything here comes from pt_model, which is the plant's own envelope
%   (see its header). The plant run is the check on that claim, and it is also
%   the only way to see what the quasi-steady model leaves out on purpose:
%   wheelspin. The plant at full throttle has no traction control, so the
%   difference in the 75 m time is what one would be worth.

if nargin < 1, withPlant = ''; end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'), fullfile(here,'..','vd'));

P  = ifssim_params();
PK = pack_from_cells(P);
E  = pt_model(P);
El = pt_model(P, struct('soc', 0.25));    % end of an endurance, roughly

fprintf('\n========================= POWERTRAIN =========================\n');
fprintf('  car: %s\n', ifssim_car());
fprintf('  pack %d s%d p, %.0f V open circuit at %.0f%% SoC, %.2f ohm, limit %.0f A\n', ...
        PK.Ns, PK.Np, PK.OCV(E.soc), 100*E.soc, PK.Rint, PK.IOperating);
fprintf('\n  %-36s %12s %12s\n', '', sprintf('SoC %.0f%%',100*E.soc), sprintf('SoC %.0f%%',100*El.soc));
row('power at the shaft [kW]',            E.shaft_kW,          El.shaft_kW);
row('terminal voltage at the limit [V]',  E.Vterm,             El.Vterm);
row('launch [g]',                         E.launch_g,          El.launch_g);
row('traction-limited up to [m/s]',       E.v_traction_end,    El.v_traction_end);
row(sprintf('%.0f m, perfect traction control [s]', E.s_accel), E.t_accel, El.t_accel);
row('  speed there [km/h]',               3.6*E.v_accel_end,   3.6*El.v_accel_end);
row('top speed [m/s]',                    E.v_top,             El.v_top);
row('  motor speed there [rpm]',          E.top_rpm,           El.top_rpm);
row('regen + drag at 10 m/s [g]',         E.regen_g_at(10),    El.regen_g_at(10));
row('regen + drag at 20 m/s [g]',         E.regen_g_at(20),    El.regen_g_at(20));

fprintf('\n  what binds, by speed (SoC %.0f%%)\n', 100*E.soc);
fprintf('  %6s  %-12s %9s %8s %9s %7s\n', 'v m/s', 'LIMIT', 'F drive N', 'ax g', 'motor rpm', 'slip');
for vq = [0 5 10 15 20 25 30 35 40]
    i = find(E.v >= vq, 1);
    fprintf('  %6.0f  %-12s %9.0f %8.3f %9.0f %7.3f\n', vq, E.limit(i), E.F_drive(i), ...
            E.ax(i)/9.81, E.motor_rpm(i), E.kappa(i));
end

R = struct('model', E, 'model_low_soc', El);

if strcmpi(withPlant, 'plant')
    A = accel_run(E.s_accel);
    fprintf('\n  %-36s %12s %12s\n', sprintf('%.0f m acceleration', E.s_accel), 'pt_model', 'plant');
    row('time [s]',          E.t_accel,           A.t_target);
    row('speed there [km/h]', 3.6*E.v_accel_end,  A.v_end_kmh);
    fprintf(['\n  The plant is %.2f s slower. Most of that is the launch: full throttle\n' ...
             '  and no traction control, so the rear tyres spin (peak slip %.1f) and\n' ...
             '  sit past their peak. pt_model holds them AT the peak. That gap is the\n' ...
             '  upper bound on what a traction controller is worth on this event.\n'], ...
            A.t_target - E.t_accel, max(A.hist(:,6)));
    R.plant = A;
end

fprintf('==============================================================\n');
fprintf('  pt_parameters   what binds, and what does not\n');
fprintf('  pt_plots        the same, as figures\n');
fprintf('  pt_study(...)   try a change\n\n');
end

function row(name, a, b)
fprintf('  %-36s %12.3f %12.3f\n', name, a, b);
end
