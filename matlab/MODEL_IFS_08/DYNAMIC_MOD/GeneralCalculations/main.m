clear; clc;

addpath("data"); addpath("src"); addpath("plots");

car = assembleCar();
validateCar(car);

% Métricas motor
[~, idxT] = max(car.T_motor_vec);
rpm_Tmax = car.rpm_vec(idxT);
Tmax = car.T_motor_vec(idxT);

% Potencia pico (con el modelo: par capado por potencia ya)
rpm_scan = linspace(0, car.rpm_redline, 2000);
P = zeros(size(rpm_scan));
for i=1:numel(rpm_scan)
    [~, P(i)] = engineMotorModel(car, rpm_scan(i));
end
[Pmax, idxP] = max(P);
rpm_Pmax = rpm_scan(idxP);

% Vmax
vmax = vmaxSolve(car);

% 0-75 m
sim75 = accelSimDistance(car, 75);

fprintf("=========== FS EV RWD PERFORMANCE TOOLKIT ===========\n");
fprintf("Motor Tmax (input curve):  %.1f Nm @ %.0f rpm\n", Tmax, rpm_Tmax);
fprintf("Motor Pmax (with cap):     %.1f kW @ %.0f rpm\n", Pmax/1000, rpm_Pmax);
fprintf("Final drive:               %.2f\n", car.final_drive);
fprintf("v_max:                     %.1f km/h (rpm=%.0f)\n", vmax.vmax_kmh, vmax.rpm_at_vmax);
fprintf("0-75 m:                    %.3f s | v_end=%.1f km/h\n", sim75.t, sim75.v_end_kmh);
fprintf("====================================================\n");

% Plots rápidos
plotAccelCurve(car);
plotForcesAtVmax(car);
