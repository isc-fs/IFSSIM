function T = build_tyre_paramset(outdir, overrides)
%BUILD_TYRE_PARAMSET  Write the Magic Formula parameter set the tyre block loads.
%
%   The Combined Slip Wheel 2DOF block does NOT take its coefficients from its
%   own dialog fields -- those are display only, overwritten whenever the tyre
%   dropdown changes. With tireType = 'External file' it loads a struct from a
%   .mat, and that struct is the only thing that decides what the tyre does.
%   This builds ours, from settings.json, so the tyre and the car cannot drift
%   apart the way they would if the numbers were typed into a dialog once.
%
%   WHAT THIS IS NOT. Magic Formula is a better-STRUCTURED model, not better
%   tyre data. Every number here still comes from the same unvalidated B/C/E
%   fit we already had; nobody has put an IFS-08 tyre on a rig. What MF buys
%   is real combined slip instead of a friction ellipse, a curve whose tail
%   gives sliding grip its own value rather than a guessed ratio, and slots
%   with the right names for real data to go into when it exists.
%
%   THE BASE FILE SUPPLIES FIELD NAMES, NOTHING ELSE. An earlier version of
%   this file loaded the shipped passenger-car set and overwrote the
%   coefficients it thought mattered, claiming in this very comment that the
%   base was used "ONLY so that all 243 fields exist". That was not true: 64
%   of 238 numeric fields were overwritten and 174 were still a 235/45R18
%   road radial's -- the whole combined-slip family, the whole aligning- and
%   overturning-moment families, the effective-radius shape, and 27 scaling
%   factors that a fitting tool had left at 0.99-ish instead of 1.
%
%   So now the base is loaded, every numeric field is STRIPPED TO ZERO, and
%   nothing exists in the output that was not written deliberately below. A
%   guard at the bottom fails the build if a non-zero passenger-car value
%   ever survives again. Zero is the right default because it is MF's
%   no-effect value nearly everywhere -- the exceptions are listed and set
%   explicitly, and the one place where zero is actively WRONG (combined
%   slip) is derived instead, see fit_combined_slip.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
% Study overrides thread all the way down. Without this, plant_study would
% announce "a tyre parameter changed, so the tyre set has to be rebuilt",
% spend the minute, and rebuild the BASELINE car -- returning a table of
% +0.0% rows for the most disputed number in the spec.
if nargin < 2, overrides = struct(); end
P = ifssim_params(overrides);

base = fullfile(matlabroot,'toolbox','vdynblks','vdynblksutilities','vdynPassCar.mat');
S    = load(base);
T    = S.vdynPassCar;          % FITTYP 62 -- Magic Formula 6.2
BASE = T;                      % kept only so the guard can police inheritance

% ---- strip ------------------------------------------------------------
% Every numeric field goes to zero. The char fields (FILE_TYPE, TYRESIDE,
% FUNCTION_NAME and friends) are format, not physics, and stay.
fn = fieldnames(T);
for k = 1:numel(fn)
    if isnumeric(T.(fn{k})), T.(fn{k}) = 0; end
end

% ---- file format ------------------------------------------------------
% Not physics: these tell the block how to read the rest.
T.FILE_VERSION   = BASE.FILE_VERSION;
T.FITTYP         = 62;      % Magic Formula 6.2
T.N_TIRE_STATES  = BASE.N_TIRE_STATES;
T.USE_MODE       = 0;

% ---- what kind of tyre this is ---------------------------------------
T.UNLOADED_RADIUS = P.WheelRadius;
T.WIDTH           = P.WheelWidth;
T.RIM_RADIUS      = P.WheelRadius * 0.62;      % 10 in rim in a 16 in tyre
T.RIM_WIDTH       = P.Assumed.TyreRimWidth;
T.ASPECT_RATIO    = 0.45;
T.FNOMIN          = P.Derived.NominalWheelLoad;   % the TYRE's reference load
T.NOMPRES         = P.Assumed.TyrePressure;
T.INFLPRES        = P.Assumed.TyrePressure;       % run at nominal: dpi = 0
% See Tyre.RefVelocity in car_spec: this is not a cosmetic reference speed,
% it sets how fast friction decays with slip velocity, and at the inherited
% 16 m/s it is what stops a spinning wheel recovering.
T.LONGVL          = P.Assumed.TyreRefVelocity;
T.VXLOW           = P.Assumed.SlipRegularisationSpeed;

% Tyre mass and inertia. The block's own wheel inertia is written separately
% by configure_tyre_block from P.Assumed.WheelInertia, which is the whole
% rotating corner; these are the rubber's share of it and must not contradict
% it. IXX is taken as half of IYY, the thin-ring relation.
T.MASS = P.Assumed.TyreMass;
T.IYY  = P.Assumed.WheelInertia;
T.IXX  = P.Assumed.WheelInertia / 2;

% ---- lateral, pure slip ----------------------------------------------
% Straight from our Magic Formula fit: C is the shape factor, mu the peak,
% E the curvature. This is the SAME curve, restated in MF 6.2's names.
T.PCY1 = P.Pacejka.LatC;
T.PDY1 = P.TireMu;
% Load sensitivity of mu. Not zero, and the reasoning for that is in
% ifssim_params next to the number: a zero here makes an axle's peak force
% invariant to how load splits across its wheels, i.e. it makes the load
% transfer the moment-arm fix restores completely inconsequential.
T.PDY2 = P.Assumed.TyreLoadSensitivity;
% PDY3 (camber) stays at the stripped zero: the suspension has no camber DOF.
T.PEY1 = P.Pacejka.LatE;
% Cornering stiffness: Kya = PKY1*FNOMIN*sin(PKY4*atan(Fz/(PKY2*FNOMIN))),
% which peaks at Fz = PKY2*FNOMIN when PKY4 = 2. PKY2 used to be 1 so that the
% sine was exactly 1 at nominal load and PKY1 was just Ca/FNOMIN -- but that
% put the stiffness PEAK at the static load, so a wheel taking on load got
% LESS responsive, which is backwards. With the peak moved above the working
% load, PKY1 is solved for so the calibrated Ca still lands exactly at static
% load. Negative because Fy opposes slip.
T.PKY4 = 2;
T.PKY2 = P.Assumed.TyreStiffnessPeakLoadRatio;
T.PKY1 = -P.Derived.CorneringStiffness / ...
         (T.FNOMIN * sin(T.PKY4 * atan(1/T.PKY2)));
% PHY* (conicity) and PVY* (ply steer) stay zero: not in our fit.

% ---- longitudinal, pure slip -----------------------------------------
T.PCX1 = P.Pacejka.LonC;
T.PDX1 = P.TireMu;
T.PDX2 = P.Assumed.TyreLoadSensitivity;   % same rubber, same load sensitivity
T.PEX1 = P.Pacejka.LonE;
T.PKX1 = P.Derived.LongSlipStiffness / T.FNOMIN;

% ---- combined slip ----------------------------------------------------
% The one family where the stripped zero is not "no assumption" but a bad
% one: it would uncouple Fx from Fy entirely and let the resultant reach
% 1.4142*mu*Fz. Derived from our own pure-slip curves instead, by requiring
% the force envelope to be a circle. See fit_combined_slip for the argument
% and for what the passenger-car values were doing to it.
[RBX1, RBY1, Dcs] = fit_combined_slip(P);
T.RBX1 = RBX1;   T.RCX1 = 1;    % REX*, RHX1 stay zero: no curvature, no shift
T.RBY1 = RBY1;   T.RCY1 = 1;    % REY*, RHY*, RVY* stay zero, likewise
% RBX2/RBX3 and RBY2/RBY3/RBY4 (load and camber dependence of the coupling)
% stay zero. RVY1-6, the kappa-induced lateral force, stays zero: it is a
% ply-steer coupling and plySteer is switched off in the block anyway.

% ---- scaling factors --------------------------------------------------
% These exist so a user can stretch a fitted dataset without refitting it.
% Ours IS the fit, so every one of them is 1 by definition. The base file
% had them at 0.98-1.02, the residue of somebody else's fitting run, which
% is a silent few-percent error on every quantity they touch.
for f = {'LFZ0','LCX','LMUX','LEX','LKX','LHX','LVX','LCY','LMUY','LEY', ...
         'LKY','LHY','LVY','LTR','LRES','LXAL','LYKA','LVYKA','LS','LKYC', ...
         'LKZC','LVMX','LMX','LMY','LMP','LCZ','LGAX','LGAY','LGAZ','LGYR', ...
         'LSGKP','LSGAL','LMUY_star','LCM'}
    if isfield(T,f{1}), T.(f{1}) = 1; end
end
% LMUV is the exception and stays at the stripped zero: it scales the decay
% of friction with slip velocity, mu/(1 + LMUV*Vs/LONGVL), which is exactly
% the effect we have chosen not to model.
%
% Setting it is NOT sufficient, and this was measured rather than assumed.
% With LMUV = 0 in this file and LONGVL back at 16 m/s, the decay returns in
% full: 0-75 m at full throttle goes 5.085 -> 6.745 s and 45% throttle beats
% 100% again, the exact symptom LONGVL was raised to cure. The block is not
% reading LMUV out of the parameter file -- it keeps its own mask default of
% 1. So LONGVL is the only working handle on this effect, the zero here is a
% statement of intent rather than a control, and anyone who "tidies" LONGVL
% back to a realistic rig speed will silently reintroduce the bug.

% ---- pressure sensitivity: none --------------------------------------
% The tyre runs at NOMPRES and the pressure input is set equal to it, so the
% PP* and P**P* families would multiply zero anyway. Left at the stripped
% zero so that a future change to the pressure input cannot quietly
% introduce a passenger car's response.

% ---- rolling resistance: OFF -----------------------------------------
% Unlike the Fiala block there is no rollingType switch here; MF always
% computes My. The QSY set is at the stripped zero, which turns it off so
% the plant keeps its own Crr, a settled number applied as an axle torque.

% ---- moments we do not use -------------------------------------------
% Mx (overturning), My (rolling resistance) and Mz (self-aligning) are all
% TERMINATED in build_tiresuspension: the chassis takes a force and a moment
% arm from us, not per-wheel moments, and the steering is kinematic so no
% self-aligning torque comes back. The QSX, Q*Z, SSZ and turn-slip families
% therefore stay at the stripped zero rather than carrying a road radial's
% pneumatic trail into a model that throws it away.

% ---- vertical: rigid carcass -----------------------------------------
% See the note in build_tiresuspension: the effective rolling radius must
% stay at the calibrated wheel radius, because the PIPELINE converts motor
% rpm to speed with it. A deflecting carcass would put a quiet ~1.3% bias
% into /odom that nobody would look for in a tyre model.
T.VERTICAL_STIFFNESS = 1e9;
T.VERTICAL_DAMPING   = 0;

% Q_RE0 scales the free rolling radius: Romega = R0*(Q_RE0 + Q_V1*(...)^2).
% The shipped value is ~1.267, which is right for the tyre it was fitted to
% and badly wrong for ours -- left alone it made a free-rolling wheel turn at
% 39.0 rad/s instead of 49.5, because it was rolling on 0.202*1.267 = 0.256 m.
% That is the same wheel-radius trap as the carcass deflection above, and it
% matters for the same reason: the pipeline turns motor rpm into speed with
% this radius.
%
% (An earlier attempt to set these produced a tyre carrying zero vertical
% load, and they took the blame for it. They were innocent: the real cause
% was the block's configuration order resetting vertType back to its own
% vertical model. Recorded here so nobody re-learns it the same way.)
T.Q_RE0 = 1;
% Q_V1, Q_V2, BREFF, DREFF, FREFF, PFZ1, BOTTOM_*, Q_FZ2, Q_FC*, Q_CAM* and
% Q_FYS* stay at the stripped zero, which leaves Re = R0 exactly and the
% vertical model rigid -- which is what vertType = 'None' in the block asks
% for anyway. Q_RA*/Q_RB* (contact patch shape) are turn-slip inputs and
% turn slip is off.

% ---- relaxation ------------------------------------------------------
% Magic Formula does not take a relaxation LENGTH; it takes carcass
% stiffnesses and derives the length as sigma = K/C. So the lengths we already
% hold as parameters are converted rather than dropped.
%
% This is load-bearing, not cosmetic. Wheel spin is a state inside the tyre
% block now, integrated explicitly, and the semi-implicit solver that used to
% keep it stable is gone with the hand-written tyre. Relaxation is what damps
% the launch transient in its place: left at the shipped values sigma_kappa
% was 0.05 m, the wheel outran the curve peak before the tyre could respond,
% and the car left the line spinning its wheels at twenty times road speed.
T.LONGITUDINAL_STIFFNESS = P.Derived.LongSlipStiffness  / P.Assumed.RelaxLengthLong;
T.LATERAL_STIFFNESS      = P.Derived.CorneringStiffness / P.Assumed.RelaxLengthLat;
% The yaw carcass mode feeds Mz only, and Mz is terminated. It still has to
% be finite or the yaw relaxation length is infinite, so it is shaped from
% the lateral stiffness over a half-width rather than left at a road tyre's.
T.YAW_STIFFNESS = T.LATERAL_STIFFNESS * (T.WIDTH/2)^2;

% ---- operating envelope ----------------------------------------------
% Validity limits, not physics: they only have to bracket what the car does.
T.FZMIN   = 0;              % a lifted wheel must really carry nothing
T.FZMAX   = 10 * T.FNOMIN;
T.PRESMIN = T.NOMPRES/2;  T.PRESMAX = T.NOMPRES*2;
T.KPUMIN  = -1.5;  T.KPUMAX = 1.5;      % a locked wheel is -1, spin exceeds it
T.ALPMIN  = -pi/2; T.ALPMAX = pi/2;     % a spinning car reaches full lateral
T.CAMMIN  = -0.2;  T.CAMMAX = 0.2;      % gamma is identically 0: no camber DOF

% ---- guard ------------------------------------------------------------
% The failure this file exists to prevent: a non-zero passenger-car number
% surviving because nobody noticed the base still supplied it. A field that
% is zero in both is ours by choice, not inheritance -- zero is what the
% strip wrote. The allowlist is for the handful we deliberately set to the
% same value the base happened to hold.
allow = {'FILE_VERSION','FITTYP','N_TIRE_STATES','USE_MODE', ...
         'KPUMIN','KPUMAX', ...   % a locked wheel is -1 whoever fitted the tyre
         'CAMMIN','CAMMAX'};      % gamma is identically zero; any bracket does
stale = {};
fn = fieldnames(BASE);
for k = 1:numel(fn)
    f = fn{k};
    if ~isnumeric(BASE.(f)) || BASE.(f) == 0, continue; end
    if any(strcmp(f,allow)), continue; end
    % A scaling factor of exactly 1 is "no scaling" -- our choice, which the
    % base happens to share for the few it did not leave at a fit residue.
    if f(1) == 'L' && T.(f) == 1, continue; end
    if isequal(T.(f), BASE.(f)), stale{end+1} = f; end %#ok<AGROW>
end
if ~isempty(stale)
    error('build_tyre_paramset:inherited', ...
          ['%d field(s) still hold the passenger-car value: %s\n' ...
           'Either set them from car_spec or leave them at the stripped zero.'], ...
          numel(stale), strjoin(stale, ' '));
end

if ~isfolder(outdir), mkdir(outdir); end
fpath = fullfile(outdir,'ifssim_tyre.mat');
ifssim_tyre = T;                                     %#ok<NASGU>
save(fpath,'ifssim_tyre');
fprintf('wrote %s\n', fpath);
Ca_at_Fz0 = -T.PKY1*T.FNOMIN*sin(T.PKY4*atan(1/T.PKY2));
fprintf('  MF 6.2 from settings.json: mu %.2f, Ca %.0f N/rad, Ck %.0f N, Fz0 %.0f N, R %.3f m\n', ...
        T.PDY1, Ca_at_Fz0, T.PKX1*T.FNOMIN, T.FNOMIN, T.UNLOADED_RADIUS);
fprintf('  load sensitivity PDY2/PDX2 %.2f, stiffness peaks at %.1f x static load\n', ...
        T.PDY2, T.PKY2);
fprintf('  combined slip derived: RBX1 %.3f RBY1 %.3f, envelope %.4f-%.4f of mu*Fz\n', ...
        T.RBX1, T.RBY1, Dcs.EnvelopeMin, Dcs.EnvelopeMax);
end
