function [Fres, parts] = resistForces(car, v)
% v [m/s]
Faero = 0.5 * car.rho * car.CdA .* v.^2;
Froll = car.Crr * car.m * car.g * ones(size(v));
Fgrade = car.m * car.g * sin(car.grade_rad) * ones(size(v));

Fres = Faero + Froll + Fgrade;

parts.Faero = Faero;
parts.Froll = Froll;
parts.Fgrade = Fgrade;
end
