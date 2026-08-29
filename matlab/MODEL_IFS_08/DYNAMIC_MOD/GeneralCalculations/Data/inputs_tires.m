function ti = inputs_tires()
% ============================ TIRES DEPT ====================================
% Modelo longitudinal simple con opción de sensibilidad a carga.

% Hoosier 16.0x7.5-10 R20 (43075 R20)

ti.mu_long_ref = 1.65;   % [-] seco, valor utilizable (starter realista)
ti.Fz_ref      = 1000;   % [N] referencia por rueda trasera
ti.mu_k        = -0.10;  % [-] sensibilidad a carga (más negativo => peor con carga)

ti.Crr         = 0.015;  % [-] resistencia a rodadura (starter)

% ============================================================================
end
