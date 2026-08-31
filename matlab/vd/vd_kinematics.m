function K = vd_kinematics(M)
%VD_KINEMATICS  What the suspension geometry does to the tyres.
%
%   The camber curve, the wheel travel, the spring the motion ratio implies,
%   and what any of it is worth. This is the suspension designer's page.
%
%   WHAT IS GEOMETRY AND WHAT IS A GUESS. The camber angles below are
%   geometry: given a roll angle and a camber gain they are arithmetic, and
%   they are as trustworthy as the gain you put in. What a degree of
%   inclination COSTS is not -- that needs a tyre on a rig, and
%   Susp.CamberGripSensitivity is one assumed number standing in for it.
%   Read the angles; treat the percentages as a sensitivity, not a result.

if nargin < 1 || isempty(M)
    here = fileparts(mfilename('fullpath'));
    addpath(here); addpath(fullfile(here,'..','plant'));
    M = dualtrack_build();
end
P = ifssim_params();
K = struct();

fprintf('\n================= SUSPENSION KINEMATICS =================\n');
fprintf('  SPRINGS AND MOTION RATIO\n');
fprintf('    front  %6.0f N/m spring at MR %.2f  ->  %6.0f N/m at the wheel\n', ...
        P.Susp.SpringRateFront, P.Susp.MotionRatioFront, P.Derived.WheelRateFront);
fprintf('    rear   %6.0f N/m spring at MR %.2f  ->  %6.0f N/m at the wheel\n', ...
        P.Susp.SpringRateRear, P.Susp.MotionRatioRear, P.Derived.WheelRateRear);
fprintf('    ride frequency %.2f Hz          (FS cars run 2.5-3.5)\n', P.Derived.RideFreqHz);
fprintf('    wheel rate goes as the SQUARE of motion ratio: a 10%% pickup move\n');
fprintf('    is a 21%% wheel-rate change, which is a bigger lever than the spring.\n');

fprintf('\n  CAMBER\n');
fprintf('    static     front %+.2f deg   rear %+.2f deg\n', ...
        P.Susp.StaticCamberFront, P.Susp.StaticCamberRear);
fprintf('    gain       front %.2f        rear %.2f   (1.0 cancels roll exactly)\n', ...
        P.Susp.CamberGainFront, P.Susp.CamberGainRear);

C = vd_constant_radius(9.125, M, 4:0.5:20);
ok = logical(C.settled);
ay = C.ay(ok);  vv = C.v(ok);  dd = C.delta(ok);
n = numel(vv);
cam = zeros(n,4); roll = zeros(n,1);
for i = 1:n
    S = dualtrack_trim(vv(i), dd(i), M);
    [~, d] = dualtrack_rhs([S.vy; S.r], [dd(i); vv(i); 0], M);
    cam(i,:) = d.camber*180/pi;  roll(i) = d.roll*180/pi;
end
K.ay = ay/9.81;  K.camber = cam;  K.roll = roll;

fprintf('\n    on a 9.125 m circle, at the limit (%.2f g, %.2f deg of roll):\n', ...
        ay(end)/9.81, roll(end));
nm = {'front inner','front outer','rear inner','rear outer'};
ix = [1 2 3 4];
for j = 1:4
    fprintf('      %-12s %+.2f deg   (moved %+.2f from static)\n', nm{j}, cam(end,ix(j)), ...
            cam(end,ix(j)) - cam(1,ix(j)));
end

fprintf('\n  WHAT THE GEOMETRY IS WORTH\n');
lap0 = 2*pi*9.125/C.v(end);
best = lap0;  worst = lap0;
for g = [0 1]
    Mg = dualtrack_build(ifssim_params({'Susp.CamberGainFront', g, 'Susp.CamberGainRear', g}));
    Cg = vd_constant_radius(9.125, Mg, 4:0.5:20);
    t  = 2*pi*9.125/Cg.v(end);
    if g == 0, worst = t; else, best = t; end
end
fprintf('    camber gain 0 (wheel leans with the body)  skid pad %.3f s\n', worst);
fprintf('    camber gain 1 (wheel stays upright)        skid pad %.3f s\n', best);
fprintf('    so the whole camber-gain design space is worth %.0f ms here.\n', 1000*(worst-best));
if abs(worst-best) < 0.05
    fprintf('\n    DO NOT QUOTE THAT NUMBER. Its magnitude rests on three things\n');
    fprintf('    nobody has measured: CamberGripSensitivity (an assumed 1.5%%/deg\n');
    fprintf('    where a real slick is 1-3), and both roll stiffnesses, which\n');
    fprintf('    car_spec marks UNKNOWN. The honest span is more like 20-60 ms.\n');
    fprintf('\n    QUOTE THE CONCLUSION INSTEAD, which survives all of that:\n');
    fprintf('    the car rolls %.2f deg at the limit, and camber gain cannot be\n', roll(end));
    fprintf('    worth much on a car that does not roll. Before spending a week\n');
    fprintf('    on camber curves, go find out whether the roll stiffnesses are\n');
    fprintf('    real -- they gate this and everything else on the balance page.\n');
end
K.camber_gain_worth = worst - best;

fprintf('\n  BUMP STEER\n');
if P.Susp.BumpSteerFront == 0 && P.Susp.BumpSteerRear == 0
    fprintf('    zero, by design, on both axles. A non-zero number here is a\n');
    fprintf('    defect to be measured with string pots, not a setting.\n');
else
    fprintf('    front %+.2f deg/m   rear %+.2f deg/m -- MEASURE THIS\n', ...
            P.Susp.BumpSteerFront, P.Susp.BumpSteerRear);
end

fprintf('\n  WHAT IS MISSING, and it is a lot\n');
fprintf('    No hardpoints. These are RATES a solver would produce, not a\n');
fprintf('    solver. There is no roll centre migration, no caster or scrub,\n');
fprintf('    no anti-dive or anti-squat, and the rates do not change with\n');
fprintf('    travel -- a real camber curve is not a straight line.\n');
fprintf('    The plant has none of this at all; it is design-model only.\n');
fprintf('========================================================\n\n');
end
