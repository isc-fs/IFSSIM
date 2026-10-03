function K = acc_report()
%ACC_REPORT  The pack, the endurance, and the ratings it is held to.
%
%       cd matlab/accumulator
%       acc_report
%
%   Three questions, in the order a scrutineer and then a race engineer ask
%   them: what IS the pack; does it finish the endurance; and do the cells
%   stay inside what their datasheet actually characterises.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
K = acc_kpis();
R = K.R;  P = K.P;

fprintf('\n========================= ACCUMULATOR ========================\n');
fprintf('  %d cells in series, %d in parallel (%d cells)\n', K.Ns, K.Np, K.Ns*K.Np);
fprintf('  %.0f V full, %.0f V nominal, %.1f Ah, %.2f kWh, %.3f ohm\n', ...
        K.V_max, K.V_nom, K.cap_Ah, K.E_kWh, K.R_pack);
fprintf('  %.1f kg of cells (car %.0f kg)\n', K.m_cells, K.m_car);
fprintf('  power at the %.0f A limit: %.1f kW at the shaft fresh, %.1f kW at 25%% SoC\n', ...
        K.I_limit, K.kW_fresh, K.kW_tired);

fprintf('\n  ENDURANCE, %.0f km, flat out every lap, no cooling\n', R.laps_needed*R.lap_length/1000);
fprintf('    %s: %.1f of %.1f laps, %.0f%% charge left\n', ...
        tern(R.finishes, 'FINISHES', 'DOES NOT FINISH'), R.laps, R.laps_needed, 100*R.soc_end);
fprintf('    energy drawn       %.2f kWh of %.2f in the pack\n', R.E_used_kWh, R.E_pack_kWh);
fprintf('    first lap %.2f s, last lap %.2f s (the pack sags as it empties)\n', R.lap_first, R.lap_tired);
fprintf('    %-26s %8s %8s %8s\n', 'per lap, at SoC', 'charge', 'energy', 'cell Vmin');
for k = 1:numel(R.per_lap)
    L = R.per_lap(k);
    fprintf('    %-26.2f %6.3f Ah %5.3f kWh %6.3f V\n', L.soc, L.Q_Ah, L.E_kWh, L.Vmin);
end
lossKWh = sum([R.per_lap.heat_J]) / numel(R.per_lap) * K.Ns*K.Np * R.laps / 3.6e6;
fprintf('    lost as heat in the cells: about %.2f kWh\n', lossKWh);

fprintf('\n  THE CELLS AGAINST THEIR RATINGS\n');
fprintf('    %-30s %8s %10s\n', '', 'event', 'rating');
fprintf('    %-30s %6.1f A %8.1f A   %s\n', 'peak cell current', R.I_cell_peak, P.Cell.ICharacterised, ...
        tern(R.I_cell_peak <= P.Cell.ICharacterised, 'inside', 'ABOVE the highest characterised rate'));
fprintf('    %-30s %6.1f A %8.1f A   %s\n', 'rms cell current', R.I_cell_rms, P.Cell.ISustained, ...
        tern(R.I_cell_rms <= P.Cell.ISustained, 'inside', 'ABOVE the sustained figure'));
fprintf('    %-30s %6.2f V %8.2f V   %s\n', 'lowest cell voltage', R.V_cell_min, P.Cell.VMin, ...
        tern(R.V_cell_min >= P.Cell.VMin, 'above the floor', 'BELOW the floor'));
fprintf('    %-30s %6.0f K\n', 'cell heating, no cooling', R.dT_cell);
fprintf(['    The no-cooling rise is what the cooling has to take out over the event.\n' ...
         '    It is not a temperature the cells reach.\n']);
fprintf('==============================================================\n');
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
