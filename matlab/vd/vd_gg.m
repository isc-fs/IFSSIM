function G = vd_gg(M, v)
%VD_GG  The g-g envelope: how much lateral is left at a given longitudinal.
%
%   G = VD_GG(M, v) returns the combined-acceleration envelope at speed v.
%
%   This is the standard concept-comparison artefact and the one picture that
%   says what a car can do, rather than what it does in one manoeuvre. Every
%   point on the boundary is a corner of the performance envelope somebody has
%   to design for.
%
%   TWO ENVELOPES ARE RETURNED, and the gap between them is the point.
%
%     G.tyre  what the four contact patches could deliver, if every wheel
%             could be given exactly the longitudinal force it wanted.
%     G.car   what THIS car can actually use. It is rear-wheel drive, so
%             traction comes from two tyres and is capped by motor torque and
%             the 80 kW rule; and it has no modulated service brake, so
%             deceleration is regen on the rear axle plus drag.
%
%   Quasi-static, and honest about it: at each point the load transfer is
%   solved consistently with the accelerations it produces (longitudinal from
%   ax, lateral from ay, aero from v), the per-wheel friction limit is applied
%   with load sensitivity, and the lateral capacity left over is
%   sqrt((mu*Fz)^2 - Fx^2). No transient, no yaw balance -- this asks what
%   grip EXISTS, not whether the car is in trim while using it.

if nargin < 1 || isempty(M)
    here = fileparts(mfilename('fullpath'));
    addpath(here); addpath(fullfile(here,'..','plant'));
    M = dualtrack_build();
end
if nargin < 2 || isempty(v), v = 12; end

P = ifssim_params();
g = 9.81;

% Longitudinal capability of the car, as opposed to of the tyres.
Fdrive_max = P.MotorMaxTorque * P.GearRatio * P.DrivetrainEfficiency / P.WheelRadius;
Fpower_max = P.MotorMaxPower * P.DrivetrainEfficiency / max(v, 1);
Fregen_max = P.MaxRegenTorque * P.GearRatio / P.WheelRadius;
Fdrag      = 0.5*P.Assumed.AirDensity*P.CdA*v^2 + P.RollingResistance*M.m*g;

axs = linspace(-2.2*g, 2.2*g, 121);
G = struct('v',v,'ax',axs/g,'ay_tyre',nan(size(axs)),'ay_car',nan(size(axs)));

for i = 1:numel(axs)
    ax = axs(i);
    G.ay_tyre(i) = solve_ay(ax, M, v, P, 'tyre',  Fdrive_max, Fpower_max, Fregen_max, Fdrag) / g;
    G.ay_car(i)  = solve_ay(ax, M, v, P, 'car',   Fdrive_max, Fpower_max, Fregen_max, Fdrag) / g;
end

G.ay_max_tyre = max(G.ay_tyre);
G.ay_max_car  = max(G.ay_car);
G.ax_max_car  = max(G.ax(G.ay_car > 0));
G.ax_min_car  = min(G.ax(G.ay_car > 0));
end

% -----------------------------------------------------------------------
function ay = solve_ay(ax, M, v, P, mode, Fdrive, Fpower, Fregen, Fdrag)
%SOLVE_AY  Most lateral acceleration available while holding this ax.
g = 9.81;
m = M.m;

% Longitudinal force the car must produce at the contact patches.
Fx_total = m*ax + Fdrag*sign(max(v,0.1));
if strcmp(mode,'car')
    if ax >= 0
        if Fx_total > min(Fdrive, Fpower), ay = 0; return; end   % not enough motor
    else
        if -Fx_total > Fregen, ay = 0; return; end               % regen only, no brakes
    end
end

Fl = 0.5*P.Assumed.AirDensity*P.ClA*v^2;
mf = m*M.wdF;  mr = m*(1-M.wdF);
Ksum = M.KrF + M.KrR;
elastic = mf*(M.h - M.hrcF) + mr*(M.h - M.hrcR);

ay = 0;
for it = 1:60
    dWlon = m*ax*M.h / M.L;
    FzF = mf*g - dWlon + Fl*M.aeroF;
    FzR = mr*g + dWlon + Fl*(1-M.aeroF);
    dWf = ay*( mf*M.hrcF/M.tF + (M.KrF/Ksum)*elastic/M.tF );
    dWr = ay*( mr*M.hrcR/M.tR + (M.KrR/Ksum)*elastic/M.tR );
    Fz  = max([FzF/2 - dWf; FzF/2 + dWf; FzR/2 - dWr; FzR/2 + dWr], 0);

    % Longitudinal force split. The tyre envelope may put it wherever it is
    % most useful; the car must put drive on the rear axle only.
    if strcmp(mode,'car') && ax >= 0
        share = [0; 0; 0.5; 0.5];
    elseif strcmp(mode,'car')
        share = [0; 0; 0.5; 0.5];            % regen brakes the driven axle
    else
        share = Fz / max(sum(Fz), eps);       % by available load
    end
    Fx = share * Fx_total;

    mu = M.PDY1 + M.PDY2*(Fz - M.Fz0)/M.Fz0;
    cap = max(mu .* Fz, 0);
    Fy = sqrt(max(cap.^2 - Fx.^2, 0));
    ayNew = sum(Fy)/m;
    if abs(ayNew - ay) < 1e-6, ay = ayNew; break; end
    ay = ay + 0.5*(ayNew - ay);               % damped, the loop is a fixed point
end
end
