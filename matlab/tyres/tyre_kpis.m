function K = tyre_kpis(P)
%TYRE_KPIS  The numbers the tyre department is judged on, for one car.
%
%   K = TYRE_KPIS()    the car as specified
%   K = TYRE_KPIS(P)   P from ifssim_params(overrides)
%
%   Two levels, because a tyre is judged twice:
%
%   THE TYRE ITSELF, from the Magic Formula coefficients exactly as
%   dualtrack_build and build_tyre_paramset derive them: peak friction
%   against load, cornering stiffness, the slip angle the peak sits at, how
%   much grip is left once it slides.
%
%   THE CAR ON IT. The lap engine only sees the PEAK of the curve, so it is
%   blind to the shape (B, C, E). The understeer gradient is not: it comes
%   from the design model's steady-state trim, which runs the whole curve.
%   Both are needed, or half the tyre's parameters look inert.
%
%   About 1.5 s.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), fullfile(here,'..','vd'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(P), P = ifssim_params(); end
g = 9.81;

M = dualtrack_build(P);
Fz0 = M.Fz0;

% ---- the tyre ------------------------------------------------------------
mu   = @(Fz) M.PDY1 + M.PDY2*(Fz - Fz0)/Fz0;                       % lateral peak
Kya  = @(Fz) -M.PKY1*Fz0*sin(M.PKY4*atan(Fz./(M.PKY2*Fz0)));        % N/rad
K.mu_half = mu(0.5*Fz0);
K.mu_nom  = mu(Fz0);
K.mu_1p5  = mu(1.5*Fz0);
K.Ca_nom  = Kya(Fz0) * pi/180;                                        % N/deg
K.Ca_1p5  = Kya(1.5*Fz0) * pi/180;
% Peak slip angle at nominal load, on the lateral curve the plant runs.
C = M.PCY1;  E = M.PEY1;  D = mu(Fz0)*Fz0;  B = Kya(Fz0)/(C*D);
shape = @(a) D*sin(C*atan(B*a - E*(B*a - atan(B*a))));
a = linspace(0, 0.6, 6001);
[~, i] = max(shape(a));
K.alpha_peak = a(i) * 180/pi;                                         % deg
K.tail_lat  = sin(C*pi/2);                                            % grip left sliding
K.tail_lon  = sin(P.Pacejka.LonC*pi/2);
K.Kx_nom    = P.Derived.LongSlipStiffness;                            % N per unit slip

% ---- the car: understeer gradient, from two steady-state trims -----------
% delta = L*ay/v^2 + K*ay/g holds at fixed speed as well as fixed radius, so
% the slope between two small steer angles gives K without a radius sweep
% (vd_constant_radius: 50 s; this: 50 ms). Low lateral, so it is the LINEAR
% gradient -- what the driver feels on turn-in, not at the limit.
v = 12;
S1 = dualtrack_trim(v, 0.010, M);
S2 = dualtrack_trim(v, 0.020, M, [S1.vy; S1.r]);
d_ay = (S2.ay - S1.ay) / g;
K.understeer = ((S2.delta - S1.delta) - M.L*(S2.ay - S1.ay)/v^2) * 180/pi / d_ay;   % deg/g

% ---- the car: lap, limit, launch ------------------------------------------
G = lap_ggv(P);
L = lap_sim(G);
K.lap      = L.time;
K.aylim_8  = interp1(G.v, G.ay0, 8)  / g;
K.aylim_22 = interp1(G.v, G.ay0, 22) / g;
K.ratio_22 = interp1(G.balance.v, G.balance.ratio, 22);
R = 9.125;                                        % FS skid pad path radius
vsk = fzero(@(vv) interp1(G.v, G.ay0, vv) - vv^2/R, [2 25]);
K.skidpad  = 2*pi*R / vsk;
K.launch_g = G.E.launch_g;
K.t75      = G.E.t_accel;
K.G = G;  K.L = L;  K.M = M;
end
