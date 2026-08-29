function rpm = speedToRPM(car, v)
% v [m/s]
GR = car.gear_ratio * car.final_drive;
wheel_omega = v / car.r_wheel;
rpm = (wheel_omega * GR) * 60/(2*pi);
end
