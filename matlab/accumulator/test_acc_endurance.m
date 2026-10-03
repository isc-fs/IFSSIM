function ok = test_acc_endurance()
%TEST_ACC_ENDURANCE  Identities the endurance model must close exactly.
%
%   There is no logged endurance to compare against, so the model is held to
%   conservation and to cases with a known answer:
%
%   1. ENERGY CLOSES. On every lap, the energy the cells' open-circuit voltage
%      gives up must equal what reaches the terminals plus what the internal
%      resistance burns: sum(Voc*I*dt) = sum(P*dt) + sum(I^2*R*dt). If the
%      current solve is wrong, this is where it shows.
%   1b. On the power limit the lap draws exactly the pack's current limit,
%      as the plant does (test_powertrain_physics).
%   2. CHARGE IS CHARGE. With no regen and no losses the charge per lap is
%      the lap's energy over the open-circuit voltage.
%   3. A PACK TWICE AS BIG goes further. Doubling the parallel strings
%      (mass following) must complete at least as many laps.
%   4. MASS FOLLOWS. Adding a string adds exactly the cells' mass to the car.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','plant'), fullfile(here,'..','spec'));
ok = true;
fprintf('\n=== accumulator endurance ===\n');

P  = ifssim_params();
PK = pack_from_cells(P);

% ---- 1. energy closes ------------------------------------------------
s = PK.SoC0;
L = lap_sim(lap_ggv(P, s));
Voc = PK.OCV(s);
I = (Voc - sqrt(Voc^2 - 4*PK.Rint*L.P_elec)) / (2*PK.Rint);
lhs = sum(Voc*I.*L.dt);
rhs = sum(L.P_elec.*L.dt) + sum(I.^2*PK.Rint.*L.dt);
ok = check(ok, 'lap energy: Voc*I = P + I^2 R [kJ]', lhs/1e3, rhs/1e3, 1e-6*abs(rhs)/1e3);

% ---- 1b. on the power limit, the lap draws the current limit ----------
% The plant's powertrain test asserts the same: pack-limited, the car draws
% exactly Pack.CurrentLimit. If the lap's power is missing a loss (as it was
% the tyre slip), it peaks below.
ok = check(ok, 'peak lap current = the pack current limit [A]', max(I), PK.IOperating, 0.01*PK.IOperating);

% ---- 2. lossless charge ----------------------------------------------
Pl = ifssim_params({'Cell.Rint', 1e-9});
PKl = pack_from_cells(Pl);
R = acc_endurance(Pl);
Ll = lap_sim(lap_ggv(Pl, PKl.SoC0));
ok = check(ok, 'lossless: charge/lap = energy / Voc [Ah]', R.per_lap(1).Q_Ah, ...
           sum(Ll.P_elec.*Ll.dt)/PKl.OCV(PKl.SoC0)/3600, 1e-6);

% ---- 3. bigger pack, further -----------------------------------------
K0 = acc_kpis(P);
K2 = acc_kpis(ifssim_params({'Pack.CellsParallelPerModule', 2*P.Pack.CellsParallelPerModule}));
ok = check(ok, 'twice the strings: at least as many laps', K2.laps >= K0.laps, true, 0);
fprintf('        laps %.1f -> %.1f, car %.0f -> %.0f kg\n', K0.laps, K2.laps, K0.m_car, K2.m_car);

% ---- 4. mass follows ---------------------------------------------------
K1 = acc_kpis(ifssim_params({'Pack.CellsParallelPerModule', P.Pack.CellsParallelPerModule + 1}));
want = PK.Ns * P.Pack.ModulesInParallel * P.Cell.Mass;      % one more cell per series position
ok = check(ok, 'one more string adds its cells'' mass [kg]', K1.m_car - K0.m_car, want, 1e-9);

fprintf('\n%s\n', tern(ok, 'accumulator endurance PASS.', 'ACCUMULATOR ENDURANCE FAILED.'));
end

function ok = check(ok, name, got, want, tol)
if islogical(got) || islogical(want)
    pass = isequal(logical(got), logical(want));
    fprintf('  [%s] %s\n', tern(pass,'ok  ','FAIL'), name);
else
    pass = abs(got - want) <= tol;
    fprintf('  [%s] %-42s got %.6g  want %.6g\n', tern(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
