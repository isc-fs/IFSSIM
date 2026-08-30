function out = vmaxSolve(car)
% vmax es el máximo v donde F_trac(v) >= F_res(v), y rpm <= redline

v_grid = linspace(0, 90, 4000); % m/s (ajusta si hace falta)
ok = false(size(v_grid));

for i = 1:numel(v_grid)
    v = v_grid(i);
    wf = wheelForceRWD_EV(car, v);
    [Fres, ~] = resistForces(car, v);
    ok(i) = (wf.F_trac >= Fres) && (wf.rpm <= car.rpm_redline);
end

if any(ok)
    vmax = max(v_grid(ok));
else
    vmax = 0;
end

out.vmax_ms = vmax;
out.vmax_kmh = vmax * 3.6;
out.rpm_at_vmax = speedToRPM(car, vmax);
end
