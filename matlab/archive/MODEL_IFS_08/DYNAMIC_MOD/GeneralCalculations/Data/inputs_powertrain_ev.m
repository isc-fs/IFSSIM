function pt = inputs_powertrain_ev()
% ========================= POWERTRAIN EV DEPT ===============================
% Modelo mínimo EV:
% - Curva de par motor vs rpm (límite por control)
% - Límite potencia inversor/batería
% - Eficiencia drivetrain (simplificada)

% EMRAX 228

pt.rpm_vec = [0 1000 2000 3000 4000 5000 6000];
pt.T_motor_vec = [230 240 240 240 240 240 200]; % Nm (estimado de la gráfica EMRAX 228HV LC)

pt.rpm_redline    = 6500;    % [rpm]
pt.T_motor_max    = 220;     % [Nm] cap duro (seguridad)

% Límites eléctricos (cap potencia en el eje motor o en rueda)
pt.P_max_motor    = 124e3;    % [W] potencia máxima disponible en motor (aprox)
pt.eta_drivetrain = 0.92;     % [-] eficiencia global motor->rueda

% Transmisión (EV 1 velocidad)
pt.gear_ratios    = 1.0;      % No tocar, al ser EV hay solo una marcha
pt.final_drive    = 32/11;    % [-] relación final

% Opcional: limitación por "traction control" (si queréis cap en rueda)
pt.T_wheel_cap = Inf;         % [Nm] Inf si no cap

% ============================================================================
end
