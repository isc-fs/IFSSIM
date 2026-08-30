function ok = test_dualtrack()
%TEST_DUALTRACK  Physics checks on the dual-track design model.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'..','plant'));
M = dualtrack_build();
ok = true;
fprintf('\n=== dual-track design model ===\n');

t = (0:0.002:5)';

%% 1. Straight ahead is straight.
Y = dualtrack_sim(t, 0, 10, M);
ok = chk(ok,'straight: no yaw rate',  Y.r(end),  0, 1e-12);
ok = chk(ok,'straight: no sideslip',  Y.vy(end), 0, 1e-12);
ok = chk(ok,'straight: left and right loads equal', ...
         Y.Fz(end,1)-Y.Fz(end,2), 0, 1e-9);

%% 2. The car is symmetric.
Yl = dualtrack_sim(t,  4*pi/180, 10, M);
Yr = dualtrack_sim(t, -4*pi/180, 10, M);
ok = chk(ok,'symmetry: mirrored steer mirrors yaw', Yl.r(end)+Yr.r(end), 0, 1e-9);
ok = chk(ok,'symmetry: mirrored steer mirrors load', ...
         (Yl.Fz(end,1)-Yl.Fz(end,2)) + (Yr.Fz(end,1)-Yr.Fz(end,2)), 0, 1e-9);

%% 3. Vertical load is conserved: the road holds the car up, whatever it does.
Y = dualtrack_sim(t, 6*pi/180, 12, M);
Fl = 0.5*M.rho*M.ClA*12^2;
ok = chk(ok,'total Fz = weight + downforce', sum(Y.Fz(end,:)), M.m*M.g + Fl, 1e-6);

%% 4. Lateral transfer is m*ay*h/t, however it is split.
% This is a free-body result and does not care about roll centres or roll
% stiffness -- those decide the SPLIT, never the total. A model that gets the
% total wrong is wrong about the car; one that gets the split wrong is only
% wrong about the balance.
dW = ((Y.Fz(end,2)-Y.Fz(end,1)) + (Y.Fz(end,4)-Y.Fz(end,3)))/2;
want = M.m*Y.ay(end)*M.h/((M.tF+M.tR)/2);
ok = chk(ok,'lateral transfer = m*ay*h/t', dW, want, 0.02*abs(want));

%% 5. The geometric term exists, and is the share the roll centres imply.
% The reference plant cannot do this at all -- its roll centre is on the
% ground -- so this check is also the standing note of what the plant owes.
geoF = M.m*M.wdF*M.hrcF/M.tF;
elas = (M.m*M.wdF*(M.h-M.hrcF) + M.m*(1-M.wdF)*(M.h-M.hrcR))*(M.KrF/(M.KrF+M.KrR))/M.tF;
ok = chk(ok,'geometric share of front transfer is 10-25%%', ...
         geoF/(geoF+elas) > 0.10 && geoF/(geoF+elas) < 0.25, true, 0);
fprintf('        front transfer: %.0f%% geometric (links), %.0f%% elastic (springs)\n', ...
        100*geoF/(geoF+elas), 100*elas/(geoF+elas));

%% 6. Grip is bounded, and the bound is the tyre's.
% Swept rather than asserted at one point: the peak has to be found, and a
% ramp is how the literature finds it.
tr = (0:0.002:12)';
peak = 0;
for v = [6 8 10 12]
    Yr2 = dualtrack_sim(tr, M.maxSteer*tr/12, v, M);
    peak = max(peak, max(abs(Yr2.ay)));
end
Flim = 0.5*M.rho*M.ClA*12^2;
cap  = M.mu*(M.m*M.g + Flim)/M.m;
ok = chk(ok,'peak ay never exceeds mu*(mg+downforce)/m', peak <= cap*1.02, true, 0);
ok = chk(ok,'peak ay actually reaches the limit (>85%%)', peak > 0.85*M.mu*M.g, true, 0);
fprintf('        peak ay %.2f m/s^2, tyre cap %.2f\n', peak, cap);

%% 7. The front axle gives up first AT THE LIMIT.
% Note this is NOT the same statement as "the car understeers". The understeer
% gradient K, measured in the linear range by vd_constant_radius, is NEGATIVE
% for this car -- it oversteers. What happens at the limit is a different
% question from what happens at 0.3 g, and a car can do one of each.
%
% The front carries more of the lateral transfer (55.1%% roll stiffness) and so
% loses more to load sensitivity as g builds; the rear starts with more static
% weight (56.2%%). Which one runs out first depends on where you look.
Yl2 = dualtrack_sim(tr, M.maxSteer*tr/12, 10, M);
[~,i] = max(abs(Yl2.ay));
af = mean(Yl2.alpha(i,1:2));  ar = mean(Yl2.alpha(i,3:4));
ok = chk(ok,'front slip angle exceeds rear at the limit', af > ar, true, 0);
fprintf('        at peak ay: front %.2f deg, rear %.2f deg\n', af*180/pi, ar*180/pi);

%% 8. The design model and the plant have the SAME tyre.
% Derived here, written there, from the same parameters -- so this asserts the
% two derivations agree rather than assuming they do. If build_tyre_paramset
% ever changes how a coefficient is formed, this fails instead of the two
% models quietly drifting apart.
T = load(M.tyreSource);  T = T.ifssim_tyre;
ok = chk(ok,'tyre matches the plant''s Magic Formula set', ...
         [M.PCY1 M.PDY1 M.PDY2 M.PEY1 M.PKY1 M.PKY2 M.PKY4 M.Fz0], ...
         [T.PCY1 T.PDY1 T.PDY2 T.PEY1 T.PKY1 T.PKY2 T.PKY4 T.FNOMIN], 1e-9);

%% 9. Low speed, small steer: the model must agree with geometry.
Yk = dualtrack_sim(t, 2*pi/180, 4, M);
kin = 4*tan(2*pi/180)/M.L;
ok = chk(ok,'low speed approaches the kinematic yaw rate', ...
         abs(Yk.r(end)/kin - 1) < 0.05, true, 0);
fprintf('        4 m/s, 2 deg: r %.4f vs kinematic %.4f\n', Yk.r(end), kin);

if ok, fprintf('\ndual-track model PASS.\n');
else,  fprintf('\nDUAL-TRACK MODEL FAILED.\n'); end
end

function ok = chk(ok,name,got,want,tol)
if islogical(want) || islogical(got)
    pass = isequal(logical(got),logical(want));
    fprintf('  [%s] %-46s %s\n', tern(pass,'ok  ','FAIL'), name, tern(logical(got),'true','false'));
else
    pass = all(abs(got-want) <= tol);
    fprintf('  [%s] %-46s got %+.6g  want %+.6g\n', tern(pass,'ok  ','FAIL'), name, got(1), want(1));
end
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
