function B = battery_from_spec(P)
%BATTERY_FROM_SPEC  The accumulator as Simscape Battery objects, from the spec.
%
%   B = BATTERY_FROM_SPEC(P)   P from ifssim_params(overrides)
%
%   Turns car_spec's pack description -- the cell, and four integers for how
%   cells are arranged -- into the Simscape Battery Builder hierarchy:
%
%       batteryCell             one cell: geometry; electrical data are set
%                               on the generated blocks (see below)
%       batteryParallelAssembly CellsParallelPerModule cells in parallel
%       batteryModule           CellsSeriesPerModule assemblies in series
%       batteryModuleAssembly   ModulesInSeries modules in series: a string
%       batteryPack             ModulesInParallel strings in parallel
%
%   ONLY THE TOPOLOGY LIVES IN THE OBJECTS. The cell's electrical data --
%   capacity, the open-circuit curve, resistance, the starting charge -- are
%   NOT baked into the generated library. build_battery_pack points them at
%   IFSSIM_* workspace variables instead, so a different cell, or a study of
%   one, reaches the plant without regenerating anything. Regeneration (~50 s)
%   is needed only when the ARRANGEMENT changes, and the library is named after
%   the arrangement (B.LibraryName) so each one is generated once.
%
%   MODEL RESOLUTION: LUMPED, per module. Each module is one cell model scaled
%   by its series and parallel counts -- five electrical states for the IFS-08,
%   the same as the hand-built pack this replaced. 'Detailed' would give a
%   state per cell (570), and is the switch to throw when cell-to-cell spread,
%   balancing or a per-cell thermal model become the question.
%
%   B.Pack          the batteryPack object
%   B.LibraryName   valid identifier, unique to the arrangement
%   B.Np, B.Ns      cells in parallel per assembly, assemblies in series per module
%   B.NModSeries, B.NModParallel, B.NModules
%   B.SeriesCells   cells in series across the whole pack

needs = {'Pack.CellsParallelPerModule','Pack.CellsSeriesPerModule', ...
         'Pack.ModulesInSeries','Pack.ModulesInParallel'};
for k = 1:numel(needs)
    x = getdot(P, needs{k});
    if ~(isscalar(x) && x >= 1 && x == round(x))
        error('battery_from_spec:topology', ...
              '%s must be a whole number of at least 1, not %g: it counts cells or modules.', needs{k}, x);
    end
end
% ONE CELL, ONE TOP VOLTAGE. Cell.VMax and the last entry of the open-circuit
% table describe the same thing. The plant used to rescale the table to
% VMax when they differed, so editing either one silently moved the other's
% meaning; now they must agree, and the build says so if they do not.
if abs(P.Cell.OCV_V(end) - P.Cell.VMax) > 1e-9
    error('battery_from_spec:vmax', ...
          ['Cell.VMax (%.3f V) and the top of Cell.OCV_V (%.3f V) disagree. They are\n' ...
           'the same quantity typed twice; change both, in car_spec.'], ...
          P.Cell.VMax, P.Cell.OCV_V(end));
end

B.Np = P.Pack.CellsParallelPerModule;
B.Ns = P.Pack.CellsSeriesPerModule;
B.NModSeries   = P.Pack.ModulesInSeries;
B.NModParallel = P.Pack.ModulesInParallel;
B.NModules     = B.NModSeries * B.NModParallel;
B.SeriesCells  = B.Ns * B.NModSeries;
B.Resolution   = "Lumped";
B.LibraryName  = sprintf('IFSSIM_Battery_%dp%ds%dm%dx', B.Np, B.Ns, B.NModSeries, B.NModParallel);

geo  = batteryCylindricalGeometry(simscape.Value(P.Cell.Height, "m"), ...
                                  simscape.Value(P.Cell.Diameter/2, "m"));
cell = batteryCell(geo);
pa   = batteryParallelAssembly(cell, B.Np);
mod  = batteryModule(pa, B.Ns, ModelResolution=B.Resolution);
str  = batteryModuleAssembly(repmat(mod, 1, B.NModSeries));
if B.NModParallel == 1
    B.Pack = batteryPack(str);
else
    B.Pack = batteryPack(repmat(str, 1, B.NModParallel), CircuitConnection="Parallel");
end
end

function x = getdot(S, name)
x = S;
for k = strsplit(name, '.'), x = x.(k{1}); end
end
