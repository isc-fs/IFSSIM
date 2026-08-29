function v = rpmToSpeed(car, rpm)
GR = car.gear_ratio * car.final_drive;
wheel_omega = (rpm * 2*pi/60) / GR;
v = wheel_omega * car.r_wheel;
end
