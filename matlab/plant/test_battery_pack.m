function ok = test_battery_pack()
%TEST_BATTERY_PACK  The generated Simscape Battery pack against the analytic answer.
%
%   A lumped table-based pack under a constant current I has a closed form:
%       V(0+)   = Nseries * OCV(soc0) - I * R_pack
%       dSoC    = I * t / (Ah_pack * 3600)
%   with R_pack = R_cell * Nseries / Nparallel and Ah_pack = Ah_cell * Nparallel.
%   The pack built by build_battery_pack is held to it in three cases, each
%   chosen to catch a different way the generated pack can be wrong:
%
%   1. THE IFS-08, as car_spec has it. Polarity included: a discharge demand
%      must give a positive voltage and a FALLING charge (wired by the
%      generated port names, it gives -416 V and charges).
%   2. A DIFFERENT CELL, by override, with NO rebuild: the cell's data are
%      workspace variables, and this proves they are read from there.
%   3. A DIFFERENT ARRANGEMENT -- strings in parallel, which the hand-wired
%      pack this replaced could not represent at all. 4p x 24s modules, four to
%      a string, two strings in parallel: the shape of change a new prototype
%      brings. Generates a second library (~1 min, cached after the first run).

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'models'), fullfile(here,'..','spec'));
ok = true;
fprintf('\n=== Simscape Battery pack ===\n');
I = 100;  T = 10;

% ---- 1. the IFS-08 ----------------------------------------------------
P = ifssim_load_workspace();
R = run_pack(P, I, T);
ok = expect(ok, 'IFS-08', P, R, I, T);
ok = check(ok, 'IFS-08: discharge gives a positive voltage', R.v0 > 0, true, 0);

% ---- 2. a different cell, same build ----------------------------------
ov = {'Cell.CapacityAh', 3.5, 'Cell.Rint', 0.015};
P2 = ifssim_load_workspace(ov);          % workspace only; harness NOT rebuilt
R2 = run_pack(P2, I, T, R.model);
ok = expect(ok, 'cell override, no rebuild', P2, R2, I, T);

% ---- 3. a different arrangement --------------------------------------
ov3 = {'Pack.CellsParallelPerModule', 4, 'Pack.CellsSeriesPerModule', 24, ...
       'Pack.ModulesInSeries', 4, 'Pack.ModulesInParallel', 2};
P3 = ifssim_load_workspace(ov3);
R3 = run_pack(P3, I, T);
ok = expect(ok, '2 strings x 4 modules x 24s4p', P3, R3, I, T);
ok = check(ok, '2 strings: one state of charge per module (8)', numel(R3.soc0), 8, 0);

ifssim_load_workspace();                 % leave the car as specified
fprintf('\n%s\n', tern(ok, 'battery pack PASS.', 'BATTERY PACK FAILED.'));
end

% -------------------------------------------------------------------------
function ok = expect(ok, tag, P, R, I, T)
PK = pack_from_cells(P);
soc0 = P.Assumed.BatteryInitialSoC;
Np = PK.Np;  Ns = PK.Ns;
Rp = P.Cell.Rint * Ns / Np;
Ah = P.Cell.CapacityAh * Np;
ocv = @(s) Ns * interp1(P.Cell.OCV_SoC, P.Cell.OCV_V, s);
dsoc = I*T/(Ah*3600);
ok = check(ok, sprintf('%s: V(0+) [V]', tag),  R.v0,   ocv(soc0) - I*Rp, 1e-3);
ok = check(ok, sprintf('%s: V(end) [V]', tag), R.vend, ocv(soc0 - dsoc) - I*Rp, 1e-3);
ok = check(ok, sprintf('%s: charge drop', tag), mean(R.soc0 - R.socend), dsoc, 1e-6);
ok = check(ok, sprintf('%s: modules stay equal', tag), max(R.socend) - min(R.socend), 0, 1e-9);
end

function R = run_pack(P, I, T, model)
STEP = '0.0010416666666666671';
if nargin < 4
    model = 'test_battery_pack_harness';
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    set_param(model,'SolverType','Fixed-step','Solver','ode1','FixedStep',STEP);
    add_block('simulink/Sources/Constant',[model '/I'],'Value','TBP_I','Position',[30 60 60 80]);
    info = build_battery_pack(model, 'Pack', [150 40 300 200], P);
    add_line(model, 'I/1', 'Pack/1');
    add_block('simulink/Sinks/To Workspace',[model '/v'],'VariableName','tbp_v', ...
              'SaveFormat','Timeseries','Position',[400 40 460 60]);
    add_line(model, 'Pack/1', 'v/1');
    add_block('simulink/Signal Routing/Mux',[model '/mux'],'Inputs',num2str(info.NumModules), ...
              'Position',[380 90 385 90+20*info.NumModules]);
    for m = 1:info.NumModules, add_line(model, sprintf('Pack/%d', m+1), sprintf('mux/%d', m)); end
    add_block('simulink/Sinks/To Workspace',[model '/s'],'VariableName','tbp_s', ...
              'SaveFormat','Timeseries','Position',[420 100 480 120]);
    add_line(model, 'mux/1', 's/1');
end
set_param(model, 'StopTime', num2str(T));
assignin('base', 'TBP_I', I);
r = sim(model);
v = r.get('tbp_v');  s = r.get('tbp_s');
d = squeeze(s.Data);  if size(d,1) ~= numel(s.Time), d = d'; end
R.v0 = v.Data(2);  R.vend = v.Data(end);
R.soc0 = d(1,:);   R.socend = d(end,:);
R.model = model;
end

function ok = check(ok, name, got, want, tol)
if islogical(got) || islogical(want)
    pass = isequal(logical(got), logical(want));
    fprintf('  [%s] %s\n', tern(pass,'ok  ','FAIL'), name);
else
    pass = abs(got - want) <= tol;
    fprintf('  [%s] %-46s got %.6g  want %.6g\n', tern(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
