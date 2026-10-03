function E = pt_model(P, opts)
%PT_MODEL  What the powertrain can put on the road, at every speed.
%
%   E = PT_MODEL()          the car as specified
%   E = PT_MODEL(P)         a parameter set from ifssim_params(overrides)
%   E = PT_MODEL(P, opts)   opts.soc      state of charge   (default: pack start)
%                           opts.s_accel  event distance, m (default 75)
%
%   The powertrain department's DESIGN model -- what dualtrack_build is to the
%   suspension department. It answers in milliseconds, so a study can sweep it.
%
%   IT IS THE PLANT'S OWN ENVELOPE, not a second one. The drive limit below is
%   the line in build_powertrain's MATLAB Function block,
%       T = min(T_cmd, min(Tmax, min(Pmax, v_pack*Ipk) / w)),
%   evaluated at the voltage the pack settles to while passing Ipk, and the
%   regen limit and the directional efficiency -- charged ONCE, at the
%   gears -- are that block's too. The
%   tyre's longitudinal peak is the plant's Magic Formula peak,
%   (PDX1 + PDX2*dfz)*Fz. If this file and the block disagree, one of them
%   is wrong; test_pt_model checks the envelope against the plant.
%
%   WHAT IT LEAVES OUT, so its numbers are not over-trusted:
%     - wheelspin. The quasi-steady run holds the tyre AT its peak when it is
%       traction-limited, never past it --
%       perfect traction control. The plant at full throttle has none, so
%       accel_run on the plant is SLOWER off the line, and the gap between the
%       two is what a traction controller is worth.
%     - pack dynamics. Voltage is taken at a fixed state of charge; over 75 m
%       the SoC moves by well under 1%.
%     - motor speed limit. The plant has none, so neither does this.
%
%   E.v            speed grid                                   [m/s]
%   E.F_motor      what the motor can put through the wheels    [N]
%   E.F_traction   what the driven tyres can transmit           [N]
%   E.F_drive      the lesser of the two                        [N]
%   E.F_resist     drag + rolling resistance                    [N]
%   E.F_regen      regen braking force at the wheels            [N]
%   E.kappa        driven-tyre slip ratio needed to make F_drive
%   E.limit        what binds at each speed: traction | torque | pack | motor power
%   E.t_accel      quasi-steady time over s_accel               [s]
%   E.v_top        where drive force meets resistance           [m/s]
%   E.pack_kW      mechanical power the pack can supply         [kW]

if nargin < 1 || isempty(P)
    P = ifssim_params();
end
if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','spec'));

PK = pack_from_cells(P);
if ~isfield(opts,'soc'),     opts.soc = PK.SoC0;  end
if ~isfield(opts,'s_accel'), opts.s_accel = 75;   end

g   = 9.81;
m   = P.Mass;
r   = P.WheelRadius;
gr  = P.GearRatio;
eta = P.DrivetrainEfficiency;
rho = P.Assumed.AirDensity;
L   = P.Wheelbase;
h   = P.CoGHeight;
Iw  = P.Assumed.WheelInertia;
m_eff = m + 4*Iw/r^2;          % the wheels have to be spun up too

% ---- the pack ----------------------------------------------------------
% Terminal voltage while passing the operating current. The block computes
% P_pack = v_pack * Ipk with v_pack the sagged terminal voltage, so at the
% limit the pack delivers Ipk at Voc - Ipk*R.
Voc    = PK.OCV(opts.soc);
Vterm  = Voc - PK.IOperating * PK.Rint;
P_pack = Vterm * PK.IOperating;                % electrical = shaft: lossless motor
P_shaft = min(P.MotorMaxPower, P_pack);

% ---- drive envelope ----------------------------------------------------
v  = (0:0.05:80)';

% Resistance: the plant's aero block is CdA and ClA at a fixed density;
% rolling resistance acts on the total normal load, downforce included.
D        = 0.5 * rho * P.ClA * v.^2;
F_resist = 0.5 * rho * P.CdA * v.^2 + P.RollingResistance * (m*g + D);

% Traction: the driven axle is the rear (the block's split is [0 0 .5 .5]).
% Its load grows with the acceleration it produces, so solve for ax.
mu0 = P.TireMu;  dfz = P.Assumed.TyreLoadSensitivity;  Fz0 = P.Derived.NominalWheelLoad;
FzR_static = m*g*(1 - P.WeightDistFront) + D*(1 - P.AeroBalanceFront);
% The plant's MF 6.x peak, Dx = (PDX1 + PDX2*dfz)*Fz with PDX1 = TireMu and
% PDX2 = TyreLoadSensitivity (build_tyre_paramset). NOT mu*(1 + PDX2*dfz),
% which an earlier version of this file used: that scales the sensitivity by
% mu and cost 2% of launch traction at the rear's launch load.
peak = @(Fz) (mu0 + dfz .* (Fz - Fz0) ./ Fz0) .* Fz;          % one tyre

% SLIP COSTS POWER, and leaving it out was worth 3-5% of drive force. To push,
% the tyre must turn faster than the road -- 5% faster at 20 m/s on this car,
% read off the plant -- so the motor spins 5% faster than road speed implies,
% and on the power limit 5% more speed is 5% less torque. Solved on the plant's
% own longitudinal Magic Formula: Kx = PKX1*Fz, C = LonC, E = LonE, and the
% same load-sensitive peak as above (build_tyre_paramset).
mf = mf_long(P);
kappa = zeros(size(v));
ax = zeros(size(v));
for it = 1:80
    wm = v .* (1 + kappa) / r * gr;
    T_motor = min(P.MotorMaxTorque, P_shaft ./ max(wm, 1e-3));
    F_motor = T_motor * gr * eta / r;
    FzR = FzR_static + m .* ax .* h ./ L;
    F_traction = 2 * peak(FzR/2);
    F_drive = min(F_motor, F_traction);
    ax_new    = (F_drive - F_resist) / m_eff;
    kappa_new = mf.slip(F_drive/2, FzR/2, peak(FzR/2));
    done = max(abs(ax_new - ax)) < 1e-9 && max(abs(kappa_new - kappa)) < 1e-9;
    ax    = 0.5*ax    + 0.5*ax_new;            % damped: transfer and slip feed back
    kappa = 0.5*kappa + 0.5*kappa_new;
    if done, break; end
end
wa = max(v / r * gr, 1e-3);                    % road-speed motor speed, for regen

limit = repmat("traction", size(v));
mot = F_motor <= F_traction;
limit(mot & T_motor >= P.MotorMaxTorque - 1e-9) = "torque";
limit(mot & T_motor <  P.MotorMaxTorque - 1e-9 & P_pack <  P.MotorMaxPower) = "pack";
limit(mot & T_motor <  P.MotorMaxTorque - 1e-9 & P_pack >= P.MotorMaxPower) = "motor power";

% ---- regen -------------------------------------------------------------
% The block: |T| <= min(Treg, Preg/w), faded by tanh(w/wregen), and the wheel
% must also brake the losses, so it is divided by eta, not multiplied.
T_regen = min(P.MaxRegenTorque, P.MaxRegenPower ./ wa) .* tanh(wa / P.Assumed.RegenFadeSpeed);
F_regen = T_regen * gr / eta / r;

% ---- the events --------------------------------------------------------
% Quasi-steady straight line: t = integral dv/a, s = integral v dv/a.
a  = max(ax, 0);
ok = a > 1e-6;
vv = v(ok);  aa = a(ok);
s_of_v = cumtrapz(vv, vv ./ aa);
t_of_v = cumtrapz(vv, 1 ./ aa);
if s_of_v(end) >= opts.s_accel
    E.v_accel_end = interp1(s_of_v, vv, opts.s_accel);
    E.t_accel     = interp1(s_of_v, t_of_v, opts.s_accel);
else
    E.v_accel_end = NaN;  E.t_accel = NaN;
end
iTop = find(F_drive - F_resist <= 0, 1, 'first');
if isempty(iTop), E.v_top = v(end); else, E.v_top = v(iTop); end

% Where the binding limit changes, as speeds a driver would recognise.
E.v_traction_end = last_speed(v, limit == "traction");
E.v_base         = v(find(T_motor < P.MotorMaxTorque - 1e-9, 1, 'first'));

E.v = v;  E.ax = ax;  E.limit = limit;  E.kappa = kappa;
E.F_motor = F_motor;  E.F_traction = F_traction;  E.F_drive = F_drive;
E.F_resist = F_resist;  E.F_regen = F_regen;  E.T_motor = T_motor;
E.motor_rpm = wm * 60/(2*pi);              % includes the slip
E.pack_kW   = P_pack / 1000;
E.shaft_kW  = P_shaft / 1000;
E.Vterm     = Vterm;
E.soc       = opts.soc;
E.s_accel   = opts.s_accel;
E.m_eff     = m_eff;
E.launch_g  = ax(1) / g;
E.regen_g_at = @(vq) (interp1(v, F_regen, vq) + interp1(v, F_resist, vq)) / m_eff / g;
E.top_rpm   = interp1(v, wm, E.v_top) * 60/(2*pi);
E.binding_top = limit(max(1, min(numel(v), iTop)));
end

function vs = last_speed(v, mask)
i = find(mask, 1, 'last');
if isempty(i), vs = 0; elseif i == numel(v), vs = v(end); else, vs = v(i); end
end

function mf = mf_long(P)
%MF_LONG  The plant's pure longitudinal Magic Formula, inverted for slip.
%   Fx = Dx sin(C atan(Bk - E(Bk - atan Bk))), B = Kx/(C Dx), Kx = PKX1*Fz.
%   In x = B*kappa the shape depends only on C and E, so x(Fx/Dx) is
%   tabulated once up to the peak and kappa = x/B.
C = P.Pacejka.LonC;  E = P.Pacejka.LonE;
PKX1 = P.Derived.LongSlipStiffness / P.Derived.NominalWheelLoad;
shape = @(x) sin(C * atan(x - E*(x - atan(x))));
xp = fzero(@(x) C*atan(x - E*(x - atan(x))) - pi/2, [1e-6 50]);
xs = linspace(0, xp, 2000)';
fs = shape(xs);
mf.slip = @(F, Fz, Dx) interp1(fs, xs, min(max(F./Dx, 0), fs(end))) ./ (PKX1*Fz ./ (C*Dx));
end
