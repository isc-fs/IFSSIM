function [Tmot, Pmot] = engineMotorModel(car, rpm)
% Par motor interpolado + caps:
% - cap por T_motor_max
% - cap por potencia P_max_motor => T <= P/omega

rpm = max(rpm, 0);
Traw = interp1(car.rpm_vec, car.T_motor_vec, rpm, "linear", "extrap");
Traw = min(Traw, car.T_motor_max);

omega = rpm * 2*pi/60;              % [rad/s]
Praw = Traw .* omega;

% Cap potencia: si omega=0, evitar división
TcapP = Traw;
idx = omega > 1e-6;
TcapP(idx) = min(Traw(idx), car.P_max_motor ./ omega(idx));

Tmot = TcapP;
Pmot = Tmot .* omega;
end
