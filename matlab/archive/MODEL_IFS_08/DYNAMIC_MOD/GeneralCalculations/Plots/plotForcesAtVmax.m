function plotForcesAtVmax(car)
v = linspace(0, 90, 600);
Ftr = zeros(size(v)); Fres = zeros(size(v));
for i=1:numel(v)
    wf = wheelForceRWD_EV(car, v(i));
    Ftr(i) = wf.F_trac;
    [Fr,~] = resistForces(car, v(i));
    Fres(i) = Fr;
end

figure; plot(v*3.6, Ftr, v*3.6, Fres);
xlabel("Speed [km/h]"); ylabel("Force [N]");
legend("F_{trac}", "F_{res}");
title("Tractive vs Resistive forces");
grid on;
end
