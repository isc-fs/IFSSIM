function aero = inputs_aero()
% ============================= AERO DEPT ====================================
% Convención: ClA negativo => downforce (genera Fz positivo hacia el suelo)
aero.CdA = 0.95;     % [m^2]
aero.ClA = -3.0;     % [m^2] (más negativo => más downforce)

% Reparto downforce: % delante (solo útil si luego se modela por ejes)
aero.balance_front = 0.45; % [-] 0..1

% ============================================================================
end
