function out = wheelForceRWD_EV(car, v)
% Devuelve fuerza tracción disponible y estados
% 1) calcula rpm
% 2) calcula Tmotor y cap por potencia
% 3) pasa a rueda
% 4) limita por neumático con transferencia (iteración simple)

rpm = speedToRPM(car, v);

% Si supera redline, no hay tracción
if rpm > car.rpm_redline
    out.F_trac = 0;
    out.rpm = rpm;
    out.Tmotor = 0;
    out.Twheel = 0;
    out.limitedBy = "redline";
    return
end

% Fuerza por powertrain (sin límite neumático)
[Tmot, ~] = engineMotorModel(car, rpm);
GR = car.gear_ratio * car.final_drive;

Twheel = Tmot * GR * car.eta_drivetrain; % [Nm]
Twheel = min(Twheel, car.T_wheel_cap);
Fpt = Twheel / car.r_wheel;              % [N]

% Límite neumático con transferencia: resuelve ax = (min(Fpt, mu*FzR) - Fres)/m
% Hacemos una iteración fija (suficiente para 1D).
ax_guess = 0;
for k = 1:8
    FzR = normalLoadsRWD(car, ax_guess, v);

    mu = car.mu_long_ref * (1 + car.mu_k * log(max(FzR,1)/car.Fz_ref)); % simple load sens.
    mu = max(mu, 0.5); % no dejar mu absurda (seguridad)

    Ftire = mu * FzR;
    [Fres, ~] = resistForces(car, v);

    Ftrac = min(Fpt, Ftire);
    ax_new = (Ftrac - Fres) / car.m;
    ax_guess = 0.6*ax_guess + 0.4*ax_new; % relajación
end

out.F_trac = max(Ftrac, 0);
out.rpm = rpm;
out.Tmotor = Tmot;
out.Twheel = Twheel;
out.F_pt = Fpt;
out.F_tire_limit = Ftire;

if Fpt <= Ftire
    out.limitedBy = "powertrain";
else
    out.limitedBy = "tire";
end
end
