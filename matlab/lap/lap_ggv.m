function G = lap_ggv(P, soc)
%LAP_GGV  The car's g-g-V envelope: lateral grip left at every speed and ax.
%
%   G = LAP_GGV()    the car as specified
%   G = LAP_GGV(P)   a parameter set from ifssim_params(overrides)
%   G = LAP_GGV(P, soc)  with the pack at this state of charge (default: the
%                    pack's start, PK.SoC0). Lower charge, lower voltage,
%                    less drive power -- the endurance runs on this.
%
%   What a quasi-steady lap simulation drives on. Built from the two
%   department models rather than a third copy of the car:
%
%     grip      the design model's (dualtrack_build): per-wheel loads with
%               longitudinal, lateral (geometric + elastic, split by the roll
%               stiffnesses) and aero load transfer, the Magic Formula peak
%               (PDY1 + PDY2*dfz)*Fz, and a circular combined-slip envelope
%               -- which is how build_tyre_paramset fits combined slip.
%     drive     pt_model's envelope: motor, pack, slip. Rear axle only.
%     braking   FOUR tyres, ideal brake balance. This is the MANUAL car: a
%               driver has a friction brake pedal. (The driverless car brakes
%               on regen alone; vd_gg's 'car' envelope is that car.)
%
%   G.v        speeds                                  [m/s]   (nv x 1)
%   G.ax       longitudinal accelerations              [m/s^2] (1 x na)
%   G.ay       lateral grip available, NaN = infeasible [m/s^2] (nv x na)
%   G.ay0      lateral grip at ax = 0, the cornering limit       (nv x 1)
%   G.ay0_pooled  the same, all four tyres pooled (no yaw balance)
%
%   Yaw balance enters once, as a cap: each speed's envelope is scaled to the
%   axle that saturates first in a steady corner (lap_balance). Otherwise
%   quasi-static: no transient, no yaw-moment control. A lap time from it is
%   the car's potential, not what a driver gets.

if nargin < 1 || isempty(P), P = ifssim_params(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','vd'), fullfile(here,'..','pt'), fullfile(here,'..','spec'));

M = dualtrack_build(P);
if nargin < 2 || isempty(soc), E = pt_model(P);
else, E = pt_model(P, struct('soc', soc)); end
g = 9.81;  m = M.m;  m_eff = E.m_eff;
rho = P.Assumed.AirDensity;

v  = (1:1:50)';
ax = (-3:0.02:2.2) * g;
ay = nan(numel(v), numel(ax));

mf = m*M.wdF;  mr = m*(1-M.wdF);
Ksum    = M.KrF + M.KrR;
elastic = mf*(M.h - M.hrcF) + mr*(M.h - M.hrcR);
latF = mf*M.hrcF/M.tF + (M.KrF/Ksum)*elastic/M.tF;     % front transfer per m/s^2
latR = mr*M.hrcR/M.tR + (M.KrR/Ksum)*elastic/M.tR;

for i = 1:numel(v)
    vi = v(i);
    Fl   = 0.5*rho*P.ClA*vi^2;
    drag = 0.5*rho*P.CdA*vi^2 + P.RollingResistance*(m*g + Fl);
    Fmot = interp1(E.v, E.F_motor, vi);

    Fx_total = m_eff*ax + drag;                  % at the contact patches
    driving  = Fx_total >= 0;
    ok = ~driving | Fx_total <= Fmot;            % the motor can make it

    dWlon = m*ax*M.h / M.L;
    FzF = mf*g - dWlon + Fl*M.aeroF;
    FzR = mr*g + dWlon + Fl*(1-M.aeroF);

    % The most lateral this ax leaves is the largest ay at which BOTH hold:
    %   every wheel can still carry its longitudinal share, |Fx| <= mu*Fz;
    %   the four tyres can make that ay, sum(Fy)/m >= ay.
    % Bisected, not iterated to the friction-circle fixed point. The fixed
    % point is where the tyres' lateral runs out, but long before that the
    % lateral transfer can unload the INNER driven wheel below its 50% drive
    % share -- an open differential gives both rear wheels the same torque,
    % so the inner one limits. The first version iterated, landed past that
    % point, and then threw the whole ax away as infeasible: 0.54 g of
    % straight-line acceleration where pt_model has 0.89.
    lo = zeros(size(ax));  hi = 3*g*ones(size(ax));
    cond = @(ayq) wheel_ok(ayq, ax, FzF, FzR, latF, latR, Fx_total, driving, M, m);
    feas0 = cond(0*ax);
    for it = 1:40
        mid = 0.5*(lo + hi);
        c = cond(mid);
        lo(c) = mid(c);  hi(~c) = mid(~c);
    end
    a = lo;
    a(~(ok & feas0)) = NaN;
    ay(i,:) = a;
end

% YAW BALANCE. The pooled envelope above lets all four tyres share the load
% as if the car could always be put in trim; it cannot, and whichever axle
% saturates first sets the corner. Scale each speed's envelope down to the
% yaw-balanced limit at ax = 0 (lap_balance). Without this a lap time cannot
% see aero balance, weight distribution or roll-stiffness split at all --
% the first aero_parameters table showed AeroBalanceFront moving nothing.
pooled0 = interp1(ax', ay', 0)';
B = lap_balance(P, v);
scale = min(1, B.ay_lim ./ max(pooled0, eps));
ay = ay .* scale;

G.v = v;  G.ax = ax;  G.ay = ay;
G.ay0 = interp1(ax', ay', 0)';
G.ay0_pooled = pooled0;
G.balance = B;
G.m_eff = m_eff;
G.P = P;
G.E = E;
end

function c = wheel_ok(ayq, ax, FzF, FzR, latF, latR, Fx_total, driving, M, m)
%WHEEL_OK  At lateral ayq (per ax column): can every wheel carry its Fx,
%   and do the four tyres make at least ayq?
dWf = ayq*latF;  dWr = ayq*latR;
Fz = max([FzF/2 - dWf; FzF/2 + dWf; FzR/2 - dWr; FzR/2 + dWr], 0);
share = Fz ./ max(sum(Fz,1), eps);
share(:, driving) = repmat([0; 0; 0.5; 0.5], 1, nnz(driving));
Fx  = share .* Fx_total;
mu  = M.PDY1 + M.PDY2*(Fz - M.Fz0)/M.Fz0;
cap = max(mu .* Fz, 0);
Fy  = sqrt(max(cap.^2 - Fx.^2, 0));
c = all(abs(Fx) <= cap + 1e-9, 1) & (sum(Fy,1)/m >= ayq - 1e-12);
end
