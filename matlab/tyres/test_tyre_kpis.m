function ok = test_tyre_kpis()
%TEST_TYRE_KPIS  The fast tyre numbers against the slow design-model tests.
%
%   tyre_kpis takes two shortcuts to stay at ~1.5 s, and each is held to the
%   design-model test it replaces (those take ~50 s each, so this test does):
%
%   UNDERSTEER GRADIENT from two steady-state trims at fixed speed, against
%   vd_constant_radius's sweep on a large circle, where the lateral is low
%   enough for both to be measuring the linear gradient.
%
%   SKID PAD from the lap engine's yaw-balanced limit, against vd_skidpad,
%   which trims the full design model on the circle.
%
%   And one identity on the curve itself: at nominal load the cornering
%   stiffness tyre_kpis reports must be exactly the one car_spec declares
%   (Derived.CorneringStiffness), because PKY1 is solved to put it there.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','vd'), fullfile(here,'..','plant'), fullfile(here,'..','spec'));
ok = true;
fprintf('\n=== tyre_kpis ===\n');
P = ifssim_params();
K = tyre_kpis(P);
M = K.M;

C = vd_constant_radius(60, M, 8:1:16);
ok = check(ok, 'K: two trims = R=60 sweep [deg/g]', K.understeer, C.K, 0.02);

S = vd_skidpad(M);
ok = check(ok, 'skid pad: lap engine = vd_skidpad [s]', K.skidpad, S.lap_time, 0.01*S.lap_time);

ok = check(ok, 'Ca at nominal = declared [N/rad]', K.Ca_nom*180/pi, P.Derived.CorneringStiffness, 1e-6*P.Derived.CorneringStiffness);

fprintf('\n%s\n', tern(ok, 'tyre_kpis PASS.', 'tyre_kpis FAILED.'));
end

function ok = check(ok, name, got, want, tol)
pass = abs(got - want) <= tol;
fprintf('  [%s] %-40s got %.4f  want %.4f\n', tern(pass,'ok  ','FAIL'), name, got, want);
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
