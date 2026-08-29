function vd = inputs_vehicle_dynamics()
% ======================= VEHICLE DYNAMICS DEPT ==============================
% Unidades: SI (m, kg, s, N, Nm)

vd.m_total      = 290;          % [kg] masa total con piloto
vd.wheelbase    = 1.627;        % [m]
vd.cg_h         = 344.1/1000;   % [m] altura CG

rear_right  = 56;
rear_left   = 62;
front_right = 48.5;
front_left  = 43.5;

front_axle = front_right + front_left;
rear_axle  = rear_right + rear_left;
total_mass = front_axle + rear_axle;

wfront = front_axle / total_mass;   % 0.438
wrear  = rear_axle  / total_mass;

vd.weight_dist_front = 0.438; % [-] reparto estático delantero (0..1)

vd.r_wheel_dyn  = 0.200;   % [m] radio dinámico rueda (clave)
vd.drive        = "RWD";   % fijo para este toolkit

% Opcionales (para futuro)
vd.Izz = NaN;             % [kg m^2] si no se usa, dejar NaN

% ============================================================================
end
