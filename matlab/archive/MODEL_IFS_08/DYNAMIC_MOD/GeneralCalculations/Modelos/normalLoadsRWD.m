function FzR = normalLoadsRWD(car, ax, v)
% Aproximación 2 ejes:
% Carga total = m*g + downforce
% Reparto estático + transferencia longitudinal: dF = m*ax*cg_h/wheelbase
% En aceleración (ax>0) aumenta carga trasera.

Fz_total = car.m*car.g + aeroDownforce(car, v);
FzF0 = car.wdf * Fz_total;
FzR0 = (1 - car.wdf) * Fz_total;

dF = car.m * ax * car.cg_h / car.wheelbase;
FzR = FzR0 + dF;

% Evitar cosas raras numéricas
FzR = max(FzR, 0);
end
