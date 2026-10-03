function ok = test_pt_model()
%TEST_PT_MODEL  Hold the powertrain design model to the plant it claims to be.
%
%   pt_model says it is build_powertrain's envelope, evaluated quasi-steadily.
%   This runs the acceleration event on the full Simulink plant (~1 min) and
%   checks that claim where it can be checked.
%
%   WHERE: above the traction limit, where the pack binds. Below it the plant
%   has no traction control and sits past the tyre's peak, which pt_model by
%   design does not -- the launch is a DIFFERENCE between them, not an error,
%   and is checked only as a bound.
%
%   THE THRESHOLDS WERE SET AFTER THE FIRST COMPARISON, not before, so they
%   prove agreement at that level and nothing about how it was reached. What
%   that comparison found, so the gap is not rediscovered:
%     - shaft power matched to 0.3%: same pack, same envelope.
%     - acceleration was 7% low until tyre SLIP went in (2-6% at 20-35 m/s;
%       on the power limit the motor spins that much faster than the road).
%     - what is left is ~25-40 N the plant body loses that tyre forces minus
%       drag minus m*a does not explain. The wheel side balances exactly
%       (drive torque = fx*r + Iw*dw/dt + Crr*Fz*r to 0.01 N.m), so it is on
%       the body side, and it is UNEXPLAINED. As acceleration falls it grows
%       as a fraction: 1% at 20 m/s, 4.5% at 35. Hence the check stops at 30.
%
%   DISCRIMINATION. The gate has to be able to fail. Charging drivetrain
%   efficiency once instead of twice is the live question about this plant
%   (see pt_parameters), and a model that did so must fall outside it.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'), fullfile(here,'..','vd'));
ok = true;
fprintf('\n=== pt_model against the plant ===\n');

P = ifssim_params();
E = pt_model(P);
A = accel_run(E.s_accel);
H = A.hist;  t = H(:,1);  Q = A.ptrain;
vw  = gradient(H(:,2), t);                       % world-frame speed
axp = gradient(movmean(vw, 48), t);
wm  = mean(Q.omega(:,3:4), 2) * P.GearRatio;     % driven wheels, at the motor

% ---- 1. the pack: same envelope ----------------------------------------
band = vw > 22 & vw < 35;
Pshaft = median(Q.motor_torque(band) .* wm(band)) / 1000;
ok = check(ok, 'shaft power on the pack limit [kW]', E.shaft_kW, Pshaft, 0.01*Pshaft);

% ---- 2. acceleration where the pack binds ------------------------------
for vq = [20 25 30]
    i = find(vw >= vq, 1);
    ok = check(ok, sprintf('ax at %d m/s [g], within 5%%', vq), ...
               interp1(E.v, E.ax, vq)/9.81, axp(i)/9.81, 0.05*axp(i)/9.81);
    ok = check(ok, sprintf('driven slip at %d m/s, within 0.015', vq), ...
               interp1(E.v, E.kappa, vq), H(i,6), 0.015);
end

% ---- 3. the launch: a bound, not an agreement --------------------------
ok = check(ok, 'perfect-TC time is a lower bound on the plant''s', ...
           E.t_accel <= A.t_target, true, 0);

% ---- 4. the gate can fail ----------------------------------------------
% Efficiency charged ONCE: the same road power from sqrt(eta) per stage.
Ponce = ifssim_params({'DrivetrainEfficiency', sqrt(P.DrivetrainEfficiency)});
Eonce = pt_model(Ponce);
i = find(vw >= 25, 1);
err = abs(interp1(Eonce.v, Eonce.ax, 25) - axp(i)) / axp(i);
fprintf('        eta charged once: ax at 25 m/s off by %.1f%%\n', 100*err);
ok = check(ok, 'discriminates: eta charged once falls outside 5%', err > 0.05, true, 0);

fprintf('\n%s\n', tern(ok, 'pt_model PASS.', 'pt_model FAILED.'));
end

% -------------------------------------------------------------------------
function ok = check(ok, name, got, want, tol)
if islogical(got) || islogical(want)
    pass = isequal(logical(got), logical(want));
    fprintf('  [%s] %s\n', tern(pass,'ok  ','FAIL'), name);
else
    pass = abs(got - want) <= tol;
    fprintf('  [%s] %-46s model %.4g  plant %.4g\n', tern(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
