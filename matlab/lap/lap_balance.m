function B = lap_balance(P, v)
%LAP_BALANCE  Which axle runs out of grip first in a steady corner, at speed v.
%
%   B = LAP_BALANCE(P, v)   P from ifssim_params(overrides), v in m/s (vector ok)
%
%   In a steady corner the front axle must supply m*ay*b/L of the lateral
%   force and the rear m*ay*a/L -- moment balance about the CoG, nothing to do
%   with the tyres. Each axle's CAPACITY depends on its load, which aero adds
%   to in the ratio AeroBalanceFront, and on the load transfer the corner
%   itself causes. Whichever axle reaches its requirement first sets the
%   limit, and the car goes there as understeer (front) or oversteer (rear).
%
%   That is why aero balance is a HANDLING number and not just a grip number:
%   downforce split differently from the weight shifts the limit balance with
%   speed. A car can be neutral in the slow hairpins and loose in the fast
%   sweepers, or the reverse, from aero alone.
%
%   B.ay_front   lateral the front axle can hold          [m/s^2]
%   B.ay_rear    lateral the rear axle can hold           [m/s^2]
%   B.ay_lim     the lower: the yaw-balanced cornering limit
%   B.limits     'front' (limit understeer) | 'rear' (limit oversteer)
%   B.ratio      ay_front / ay_rear; > 1 means the rear goes first
%
%   ay_lim is <= the POOLED limit (all four tyres together, no yaw balance).
%   lap_ggv caps its envelope at ay_lim, so the lap time sees the balance;
%   the gap between the two is grip a balance change would free.
%
%   Steady state, ax = 0, rear drive holding speed against drag. Same load
%   transfer and tyre law as lap_ggv.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','vd'), fullfile(here,'..','plant'), fullfile(here,'..','spec'));
M = dualtrack_build(P);
g = 9.81;  m = M.m;
rho = P.Assumed.AirDensity;
mf = m*M.wdF;  mr = m*(1-M.wdF);
Ksum    = M.KrF + M.KrR;
elastic = mf*(M.h - M.hrcF) + mr*(M.h - M.hrcR);
latF = mf*M.hrcF/M.tF + (M.KrF/Ksum)*elastic/M.tF;
latR = mr*M.hrcR/M.tR + (M.KrR/Ksum)*elastic/M.tR;

n = numel(v);
B.v = v(:);  B.ay_front = zeros(n,1);  B.ay_rear = zeros(n,1);
for k = 1:n
    Fl   = 0.5*rho*P.ClA*v(k)^2;
    drag = 0.5*rho*P.CdA*v(k)^2 + P.RollingResistance*(m*g + Fl);
    FzF = mf*g + Fl*M.aeroF;
    FzR = mr*g + Fl*(1-M.aeroF);
    B.ay_front(k) = bisect(@(ay) axle_ok(ay, FzF, latF, 0,    M, m*M.b/M.L));
    B.ay_rear(k)  = bisect(@(ay) axle_ok(ay, FzR, latR, drag, M, m*M.a/M.L));
end
B.ay_lim = min(B.ay_front, B.ay_rear);
B.ratio  = B.ay_front ./ B.ay_rear;
B.limits = repmat("front", n, 1);
B.limits(B.ay_rear < B.ay_front) = "rear";
end

% -------------------------------------------------------------------------
function ok = axle_ok(ay, Fz_axle, lat, Fx_axle, M, m_share)
%AXLE_OK  Can this axle make its share of ay, with Fx_axle on it as well?
dW = ay*lat;
Fz = max([Fz_axle/2 - dW; Fz_axle/2 + dW], 0);
Fx = [Fx_axle/2; Fx_axle/2];
mu  = M.PDY1 + M.PDY2*(Fz - M.Fz0)/M.Fz0;
cap = max(mu .* Fz, 0);
ok = all(abs(Fx) <= cap) && sum(sqrt(max(cap.^2 - Fx.^2, 0))) >= m_share*ay;
end

function x = bisect(f)
lo = 0;  hi = 40;
for it = 1:50
    mid = 0.5*(lo + hi);
    if f(mid), lo = mid; else, hi = mid; end
end
x = lo;
end
