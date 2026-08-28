function K = pack_from_cells(C)
%PACK_FROM_CELLS  Derive every pack quantity from the cell and the topology.
%
%   The plant needs pack voltage, capacity, resistance and current limits.
%   None of those are typed anywhere: they come from the cell part number and
%   how many of them are arranged which way, so changing the arrangement
%   cannot leave a stale pack value behind somewhere else.
%
%   K = PACK_FROM_CELLS(CAR_SPEC).

if nargin < 1, C = car_spec(); end
v = @(n) C.Fields.(strrep(n,'.','_')).value;

K.Ns = v('Pack.CellsSeriesPerModule')   * v('Pack.ModulesInSeries');    % cells in series
K.Np = v('Pack.CellsParallelPerModule') * v('Pack.ModulesInParallel');  % strings in parallel
K.NModules = v('Pack.ModulesInSeries') * v('Pack.ModulesInParallel');
K.NCells   = K.Ns * K.Np;

K.VMax = K.Ns * v('Cell.VMax');
K.VNom = K.Ns * v('Cell.VNom');
K.VMin = K.Ns * v('Cell.VMin');

K.CapacityAh = K.Np * v('Cell.CapacityAh');
K.EnergyWh   = K.CapacityAh * K.VNom;

% Series adds resistance, parallel divides it.
K.Rint = v('Cell.Rint') * K.Ns / K.Np;

% Current limits are set by the PARALLEL count; voltage by the series count.
K.IMaxCont  = K.Np * v('Cell.ISustained');       % what it can hold
K.IMaxPulse = K.Np * v('Cell.ICharacterised');   % highest rate characterised

% Power the pack can actually deliver, at nominal voltage and allowing for
% the sag its own resistance causes at that current. This is the number the
% torque envelope should be respecting and currently is not.
K.PMaxCont  = K.IMaxCont  * (K.VNom - K.IMaxCont *K.Rint);
K.PMaxPulse = K.IMaxPulse * (K.VNom - K.IMaxPulse*K.Rint);

% What the car is actually allowed to pull, and what that delivers. This is
% the operating point, as distinct from K.IMaxPulse which is the sum of the
% cells' own ratings.
K.IOperating = v('Pack.CurrentLimit');
K.POperating = K.IOperating * (K.VNom - K.IOperating*K.Rint);
K.CellAmpsAtOperating = K.IOperating / K.Np;

% Self-heating per cell at the operating current. This, not a rating, is what
% says whether a current is survivable -- and it only means anything against
% a duration, so it is returned as a rate.
K.CellWatts   = K.CellAmpsAtOperating^2 * v('Cell.Rint');
K.CellThermal = v('Cell.Mass') * v('Cell.SpecificHeat');    % J/K
K.CellKperSec = K.CellWatts / K.CellThermal;

K.Mass = K.NCells * v('Cell.Mass');
end
