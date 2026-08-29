function validateCar(car)
% Validaciones rápidas para evitar errores típicos de unidades/inputs.

mustBePositive(car.m)
mustBePositive(car.r_wheel)
mustBePositive(car.wheelbase)
mustBePositive(car.g)

assert(car.wdf > 0 && car.wdf < 1, "weight_dist_front debe estar en (0,1)")
assert(car.rpm_redline > 1000, "rpm_redline parece demasiado bajo (¿unidades?)")
assert(car.P_max_motor > 1e3, "P_max_motor parece demasiado bajo (¿W vs kW?)")

% Signos aero
% ClA negativo = downforce
% CdA positivo
assert(car.CdA > 0, "CdA debe ser > 0")
% ClA puede ser 0 si sin aero
end
