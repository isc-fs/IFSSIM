function car = assembleCar()
% Junta todos los inputs en una sola struct "car"

car.vd   = inputs_vehicle_dynamics();
car.pt   = inputs_powertrain_ev();
car.ti   = inputs_tires();
car.aero = inputs_aero();
car.env  = inputs_environment();

% Campos "planos" para comodidad
car.m   = car.vd.m_total;
car.g   = car.env.g;
car.rho = car.env.rho;

car.r_wheel = car.vd.r_wheel_dyn;
car.wheelbase = car.vd.wheelbase;
car.cg_h = car.vd.cg_h;
car.wdf = car.vd.weight_dist_front; % front static fraction

car.CdA = car.aero.CdA;
car.ClA = car.aero.ClA;

car.mu_long_ref = car.ti.mu_long_ref;
car.mu_k        = car.ti.mu_k;
car.Fz_ref      = car.ti.Fz_ref;
car.Crr         = car.ti.Crr;

car.eta_drivetrain = car.pt.eta_drivetrain;
car.final_drive    = car.pt.final_drive;
car.gear_ratio     = car.pt.gear_ratios(1); % 1-speed
car.rpm_redline    = car.pt.rpm_redline;
car.P_max_motor    = car.pt.P_max_motor;
car.T_motor_max    = car.pt.T_motor_max;
car.T_wheel_cap    = car.pt.T_wheel_cap;

car.rpm_vec     = car.pt.rpm_vec;
car.T_motor_vec = car.pt.T_motor_vec;

car.grade_rad = car.env.grade_rad;

end
