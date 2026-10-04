function ok = test_susp_geometry()
%TEST_SUSP_GEOMETRY  The imported hardpoints reproduce the workbook's own analysis.
%
%   With the sheet's datum (ground at its Z = 0, CoG where its FASE 3 puts
%   it), every number the Susp_Geometry sheet computes must come back out of
%   sm_hardpoints + susp_geometry. That proves the import (every point read,
%   none shifted), the frame conversion (an x sign error flips caster and
%   trail, and the "front" pivot guard catches a mirror), and the geometry
%   code, against a calculation somebody else did by hand.
%
%   Only then are the car's own numbers worth anything: the same code with the
%   ground where the tyre puts it (sm_hardpoints' default).

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
ok = true;
fprintf('\n=== suspension geometry against the workbook ===\n');
sheet = struct('GroundZ_mm', 0, 'car', 'IFS-08');
P = ifssim_params([], 'IFS-08');

% ---- front: FASE 1, 2, 3 ----------------------------------------------
H = sm_hardpoints('front', sheet);
% The sheet's FASE 3 CoG: x = 865.7009 in SHEET coordinates, i.e. 705.7 mm
% behind the front wheel centre (sheet x 160).
cogF = struct('x', H.wheel_centre(1) - (865.7009 - 160)/1000, 'h', 0.300, 'brakeFront', 0.6);
G = susp_geometry(H, cogF);
ok = c(ok, 'front KPI [deg]                (row 68)',  G.kpi_deg,       7.125,     1e-3);
ok = c(ok, 'front caster [deg]             (row 69)',  G.caster_deg,    6.2065,    1e-3);
ok = c(ok, 'front scrub radius [mm]        (row 77)',  G.scrub_m*1e3,   21.25,     1e-3);
ok = c(ok, 'front mechanical trail [mm]    (row 108)', G.trail_m*1e3,   36.9125,   1e-3);
ok = c(ok, 'front kingpin offset [mm]      (row 79)',  G.kpOffset_m*1e3, 60,       1e-3);
ok = c(ok, 'front spindle length [mm]      (row 80)',  G.spindle_m*1e3, 38,        1e-3);
ok = c(ok, 'front IC y [mm]                (row 152)', G.icFV(1)*1e3,  -4292.1831, 1e-3);
ok = c(ok, 'front IC z [mm]                (row 153)', G.icFV(2)*1e3,   230,       1e-3);
ok = c(ok, 'front roll centre [mm]         (row 164)', G.rc_m*1e3,      28.2083,   1e-3);
ok = c(ok, 'front FVSA [mm]                (row 167)', G.fvsa_m*1e3,    4897.5867, 1e-3);
ok = c(ok, 'front anti-dive [%]            (row 246)', G.antiDive_pct,  26.6509,   1e-3);

% ---- rear ---------------------------------------------------------------
H = sm_hardpoints('rear', sheet);
cogR = struct('x', H.wheel_centre(1) + (1730 - 864.2991)/1000, 'h', 0.300, 'brakeFront', 0.6);
G = susp_geometry(H, cogR);
ok = c(ok, 'rear KPI [deg]                 (row 89)',  G.kpi_deg,       3.5763,    1e-3);
ok = c(ok, 'rear caster [deg]              (row 90)',  G.caster_deg,    0,         1e-3);
ok = c(ok, 'rear scrub radius [mm]         (row 97)',  G.scrub_m*1e3,   35.625,    1e-3);
ok = c(ok, 'rear kingpin offset [mm]       (row 99)',  G.kpOffset_m*1e3, 55,       1e-3);
ok = c(ok, 'rear spindle length [mm]       (row 100)', G.spindle_m*1e3, 50,        1e-3);
ok = c(ok, 'rear IC y [mm]                 (row 192)', G.icFV(1)*1e3,  -1778.656,  1e-3);
ok = c(ok, 'rear roll centre [mm]          (row 197)', G.rc_m*1e3,      58.016,    1e-3);
ok = c(ok, 'rear FVSA [mm]                 (row 200)', G.fvsa_m*1e3,    2389.7498, 1e-3);
ok = c(ok, 'rear "anti-squat" = anti-LIFT [%] (row 258)', G.antiLift_pct, 22.1884, 1e-3);

% ---- the frame is the plant's ------------------------------------------
Hf = sm_hardpoints('front');  Hr = sm_hardpoints('rear');
ok = c(ok, 'front wheel centre at the plant''s front axle [m]', Hf.wheel_centre(1),  P.Derived.aFront, 1e-12);
ok = c(ok, 'rear wheel centre at the plant''s rear axle [m]',   Hr.wheel_centre(1), -P.Derived.bRear,  1e-12);
ok = c(ok, 'front track = car_spec TrackFront [m]', Hf.track, P.TrackFront, 1e-12);

fprintf('\n%s\n', tern(ok, 'suspension geometry PASS: the import reproduces the workbook.', ...
                           'SUSPENSION GEOMETRY FAILED.'));
end

function ok = c(ok, name, got, want, tol)
pass = abs(got - want) <= tol;
fprintf('  [%s] %-50s got %11.4f  want %11.4f\n', tern(pass,'ok  ','FAIL'), name, got, want);
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
