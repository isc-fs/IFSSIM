function ok = test_lap_sim()
%TEST_LAP_SIM  Identities the lap engine must reproduce exactly.
%
%   A lap time has no ground truth here -- there is no recorded fast lap -- so
%   the engine is held to cases whose answer is known without it:
%
%   1. A CIRCLE, on a car with no load sensitivity, no aero, no rolling
%      resistance: total grip is mu*m*g whatever the load transfer does, so
%      the speed must be sqrt(mu*g*R). Any error in the corner-speed solve or
%      the load-transfer loop shows up here.
%   1b. BALANCE. The same flat car must also be exactly yaw-balanced.
%       And on the real, rear-limited car, a rearward aero balance must
%       raise the limit.
%   2. A STRAIGHT from rest: the forward pass on the envelope must reproduce
%      pt_model's 75 m time -- same drive limit, same traction law, reached by
%      a different route (a g-g-V table and a path march, against an ODE in
%      speed). Disagreement means one of the two is integrating wrong.
%   3. DIRECTION. Downforce at no drag cost must not make a lap slower; drag
%      at no downforce must not make it faster. Weak, but a sign error passes
%      the two identities above and fails this.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'), fullfile(here,'..','pt'));
ok = true;
fprintf('\n=== lap engine ===\n');

% ---- 1. circle --------------------------------------------------------
flat = {'Tyre.LoadSensitivity',0, 'ClA',0, 'CdA',0, 'RollingResistance',0};
Pf = ifssim_params(flat);
G  = lap_ggv(Pf);
for R = [9 15 25]
    L = lap_sim(G, [R 2*pi*R]);
    want = sqrt(Pf.TireMu * 9.81 * R);
    ok = check(ok, sprintf('circle R=%2d m: v = sqrt(mu g R)', R), median(L.v), want, 0.005*want);
end

% ---- 1b. yaw balance --------------------------------------------------
% The same flat car is exactly balanced: each axle's capacity and its
% requirement are both proportional to its static load, so neither goes
% first and the balanced limit IS mu*g.
B = lap_balance(Pf, 15);
ok = check(ok, 'flat car: front/rear capacity ratio = 1', B.ratio, 1, 1e-6);
ok = check(ok, 'flat car: balanced limit = mu g', B.ay_lim, Pf.TireMu*9.81, 1e-3);
% And the cap moves the right way: on this rear-limited car, aero balance
% moved REARWARD loads the axle that runs out, so the limit must rise.
P  = ifssim_params();
Bb = lap_balance(P, 22);
Br = lap_balance(ifssim_params({'AeroBalanceFront', P.AeroBalanceFront - 0.05}), 22);
ok = check(ok, 'rear-limited car: rearward aero balance raises the limit', ...
           Bb.limits == "rear" && Br.ay_lim > Bb.ay_lim, true, 0);

% ---- 2. straight from rest -------------------------------------------
G = lap_ggv(P);
E = pt_model(P);
L = lap_sim(G, [Inf 75], struct('closed', false, 'v0', 0, 'ds', 0.05));
ok = check(ok, '75 m from rest = pt_model [s]', L.time, E.t_accel, 0.01*E.t_accel);
ok = check(ok, 'speed at 75 m = pt_model [m/s]', L.v(end), E.v_accel_end, 0.01*E.v_accel_end);

% ---- 3. direction ----------------------------------------------------
L0 = lap_sim(G);
Ld = lap_sim(lap_ggv(ifssim_params({'ClA', 1.2*P.ClA})));
Lx = lap_sim(lap_ggv(ifssim_params({'CdA', 1.2*P.CdA})));
ok = check(ok, 'more downforce, same drag: not slower', Ld.time <= L0.time, true, 0);
ok = check(ok, 'more drag, same downforce: not faster', Lx.time >= L0.time, true, 0);
fprintf('        lap %.3f s; +20%% ClA %.3f s; +20%% CdA %.3f s\n', L0.time, Ld.time, Lx.time);

fprintf('\n%s\n', tern(ok, 'lap engine PASS.', 'LAP ENGINE FAILED.'));
end

function ok = check(ok, name, got, want, tol)
if islogical(got) || islogical(want)
    pass = isequal(logical(got), logical(want));
    fprintf('  [%s] %s\n', tern(pass,'ok  ','FAIL'), name);
else
    pass = abs(got - want) <= tol;
    fprintf('  [%s] %-40s got %.4f  want %.4f\n', tern(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
