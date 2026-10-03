function K = aero_kpis(P)
%AERO_KPIS  The numbers the aero department is judged on, for one car.
%
%   K = AERO_KPIS()     the car as specified
%   K = AERO_KPIS(P)    P from ifssim_params(overrides)
%
%   One function so the report, the parameter table and the study all measure
%   the same things the same way. About a second: it builds the g-g-V
%   envelope and drives a lap on it.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(P), P = ifssim_params(); end

rho = P.Assumed.AirDensity;  g = 9.81;
q = @(v) 0.5*rho*v.^2;

K.downforce15 = q(15)*P.ClA;
K.drag15      = q(15)*P.CdA;
K.LD          = P.ClA / max(P.CdA, eps);
K.aero_front  = P.AeroBalanceFront;
K.weight_front= P.WeightDistFront;

G = lap_ggv(P);
L = lap_sim(G);
E = G.E;
B = lap_balance(P, [8 15 22]);

K.ay0_8   = interp1(G.v, G.ay0_pooled, 8)  / g;
K.ay0_22  = interp1(G.v, G.ay0_pooled, 22) / g;
K.aylim_8  = B.ay_lim(1) / g;
K.aylim_22 = B.ay_lim(3) / g;
K.limits_8  = B.limits(1);
K.limits_22 = B.limits(3);
K.ratio_8   = B.ratio(1);
K.ratio_22  = B.ratio(3);
K.lap       = L.time;
K.lap_mean_kmh = 3.6*L.v_mean;
K.lap_E_kJ  = L.E_pack;
K.lap_drag_kJ = L.E_drag;
K.t75       = E.t_accel;
K.v_top     = E.v_top;
K.L = L;  K.G = G;  K.B = B;
end
