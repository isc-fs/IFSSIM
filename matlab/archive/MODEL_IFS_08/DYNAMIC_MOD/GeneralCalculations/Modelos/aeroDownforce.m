function Fz_aero = aeroDownforce(car, v)
% Downforce total (N). ClA negativo => Fz positivo.
Fz_aero = -0.5 * car.rho * car.ClA .* v.^2;
end
