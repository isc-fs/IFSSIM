function plotAccelCurve(car)
v = linspace(0, 35, 250); % m/s
a = zeros(size(v));
lim = strings(size(v));

for i=1:numel(v)
    wf = wheelForceRWD_EV(car, v(i));
    [Fres,~] = resistForces(car, v(i));
    a(i) = (wf.F_trac - Fres)/car.m;
    lim(i) = wf.limitedBy;
end

figure; plot(v*3.6, a);
xlabel("Speed [km/h]"); ylabel("Acceleration [m/s^2]");
title("a(v) longitudinal");
grid on;
end
