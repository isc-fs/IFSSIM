function ok = test_kinematics_feed(car)
%TEST_KINEMATICS_FEED  The car's kinematic numbers are its hardpoints' numbers.
%
%   ok = TEST_KINEMATICS_FEED()          the active car
%   ok = TEST_KINEMATICS_FEED('IFS-09')
%
%   Motion ratio, camber gain, bump steer, roll centres and static camber are
%   DERIVED in the car's spec from its hardpoints. This re-derives them -- the
%   multibody sweep for the first three, the static geometry for the rest --
%   and fails if the spec has drifted from its own geometry. A hardpoint edit
%   that is not carried through to the numbers the models read is the exact
%   failure that left the plant on assumed values for a year.
%
%   Two more things the feed must not break:
%     WHEEL RATE STAYS ANCHORED. The spring is re-derived as Kw/MR^2, so the
%     wheel rate -- the quantity tied to the recorded ride frequency -- must
%     still be HeaveStiffness/4.
%     BUMP STEER HAS THE RIGHT SIGN IN THE DESIGN MODEL. Front toe-in in bump
%     steers both front wheels INTO the turn as the body rolls (outer in bump,
%     inner in droop): roll oversteer, so the understeer gradient must FALL.
%     The design model had the opposite sign until this was written, harmless
%     while bump steer was zero.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'), fullfile(here,'..','vd'));
if nargin < 1 || isempty(car), car = ifssim_car(); end
was = ifssim_car();  restore = onCleanup(@() ifssim_car(was));
ifssim_car(car);
P = ifssim_params([], car);
ok = true;
fprintf('\n=== kinematics in the %s spec, against its hardpoints ===\n', car);

for ax = {'front','rear'}
    a = ax{1};  sfx = [upper(a(1)) a(2:end)];
    evalc('T = sm_corner_sweep(a);');
    v = T(abs(T.travel_mm) <= 30.5 & ~isnan(T.travel_mm), :);
    pc = polyfit(v.travel_mm/1000, v.camber_deg, 1);
    pt = polyfit(v.travel_mm/1000, v.toe_deg, 1);
    H = sm_hardpoints(a, struct('car', car));
    gain = -pc(1) * (H.track/2) * pi/180;
    toein = -pt(1);                               % sweep toe is left-wheel steer
    G = susp_geometry(H);
    ok = c(ok, sprintf('%s camber gain', a),            P.Susp.(['CamberGain' sfx]),  gain,  0.002);
    ok = c(ok, sprintf('%s bump steer [deg/m toe-in]', a), P.Susp.(['BumpSteer' sfx]), toein, 0.02);
    ok = c(ok, sprintf('%s motion ratio', a),           P.Susp.(['MotionRatio' sfx]), T.Properties.UserData.motionRatio, 0.002);
    ok = c(ok, sprintf('%s roll centre [mm]', a),       P.(['RollCenter' sfx])*1e3, G.rc_m*1e3, 0.2);
    ok = c(ok, sprintf('%s static camber [deg]', a),    P.Susp.(['StaticCamber' sfx]), G.camber_deg, 0.01);
    ok = c(ok, sprintf('%s model assembles on the hardpoints [mm]', a), ...
           T.Properties.UserData.assemblyError_m*1e3, 0, 1e-3);
    ok = c(ok, sprintf('%s wheel rate = HeaveStiffness/4 [N/m]', a), ...
           P.Susp.(['SpringRate' sfx]) * P.Susp.(['MotionRatio' sfx])^2, P.HeaveStiffness/4, 0.002*P.HeaveStiffness/4);
end

% ---- bump steer sign, by its physics --------------------------------------
K0 = understeer(ifssim_params({'Susp.BumpSteerFront', 0}, car));
K1 = understeer(ifssim_params({'Susp.BumpSteerFront', 10}, car));
fprintf('        understeer K: %+.4f deg/g with no bump steer, %+.4f with 10 deg/m front toe-in\n', K0, K1);
ok = c(ok, 'front toe-in in bump is roll OVERSTEER (K falls)', double(K1 < K0), 1, 0);

fprintf('\n%s\n', tern(ok, sprintf('kinematics feed PASS: the %s spec is its geometry.', car), ...
                           'KINEMATICS FEED FAILED.'));
end

% -------------------------------------------------------------------------
function K = understeer(P)
% Two steady-state trims at 12 m/s, as tyre_kpis does it, at a lateral where
% the body rolls enough for roll steer to register.
M = dualtrack_build(P);
v = 12;
S1 = dualtrack_trim(v, 0.02, M);
S2 = dualtrack_trim(v, 0.04, M, [S1.vy; S1.r]);
K = ((S2.delta - S1.delta) - M.L*(S2.ay - S1.ay)/v^2) * 180/pi / ((S2.ay - S1.ay)/9.81);
end

function ok = c(ok, name, got, want, tol)
pass = abs(got - want) <= tol;
fprintf('  [%s] %-46s spec %10.4f  geometry %10.4f\n', tern(pass,'ok  ','FAIL'), name, got, want);
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
