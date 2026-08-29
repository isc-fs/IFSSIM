function sim = accelSimDistance(car, s_target)
if nargin < 2, s_target = 75; end

dt = 0.001;
t = 0; v = 0; s = 0;

hist = zeros(0,6); % [t s v ax rpm Ftrac]

while s < s_target && t < 30
    wf = wheelForceRWD_EV(car, v);
    [Fres, ~] = resistForces(car, v);

    ax = (wf.F_trac - Fres) / car.m;
    ax = max(ax, -30);

    v = max(0, v + ax*dt);
    s = s + v*dt;
    t = t + dt;

    hist(end+1,:) = [t s v ax wf.rpm wf.F_trac]; %#ok<AGROW>
end

sim.t = t;
sim.v_end = v;
sim.v_end_kmh = v*3.6;
sim.hist = hist;
end
