function T = build_tyre_paramset(outdir)
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
%   The base is the shipped passenger-car set, used ONLY so that all 243
%   fields exist and are self-consistent. Every coefficient that describes
%   what KIND of tyre this is gets overwritten below, and every effect we
%   have no data for is zeroed rather than left at a passenger car's value --
%   inheriting a road tyre's load sensitivity by omission would be worse than
%   admitting we do not model it.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
P = ifssim_params();

base = fullfile(matlabroot,'toolbox','vdynblks','vdynblksutilities','vdynPassCar.mat');
S = load(base);
T = S.vdynPassCar;          % FITTYP 62 -- Magic Formula 6.2

% ---- what kind of tyre this is ---------------------------------------
T.UNLOADED_RADIUS = P.WheelRadius;
T.WIDTH           = P.Assumed.TyreWidth;
T.RIM_RADIUS      = P.WheelRadius * 0.62;      % 10 in rim in a 16 in tyre
T.ASPECT_RATIO    = 0.45;
T.FNOMIN          = P.Derived.NominalWheelLoad;   % OUR static corner load
T.NOMPRES         = P.Assumed.TyrePressure;
T.LONGVL          = 16;                        % reference speed for the fit
T.VXLOW           = P.Assumed.SlipRegularisationSpeed;

% ---- lateral, pure slip ----------------------------------------------
% Straight from our Magic Formula fit: C is the shape factor, mu the peak,
% E the curvature. This is the SAME curve, restated in MF 6.2's names.
T.PCY1 = P.Pacejka.LatC;
T.PDY1 = P.TireMu;
T.PDY2 = 0;   % no load sensitivity of mu -- we have never measured it
T.PDY3 = 0;   % no camber sensitivity -- the suspension has no camber DOF
T.PEY1 = P.Pacejka.LatE;
T.PEY2 = 0; T.PEY3 = 0; T.PEY4 = 0; T.PEY5 = 0;
% Cornering stiffness. MF builds it as PKY1*FNOMIN*sin(PKY4*atan(Fz/(PKY2*FNOMIN))),
% so with PKY4 = 2 and PKY2 = 1 the sine is exactly 1 at Fz = FNOMIN and the
% whole thing collapses to PKY1*FNOMIN. Negative because Fy opposes slip.
T.PKY1 = -P.Derived.CorneringStiffness / T.FNOMIN;
T.PKY2 = 1; T.PKY3 = 0; T.PKY4 = 2; T.PKY5 = 0; T.PKY6 = 0; T.PKY7 = 0;
T.PHY1 = 0; T.PHY2 = 0;      % no horizontal shift: no conicity in our fit
T.PVY1 = 0; T.PVY2 = 0; T.PVY3 = 0; T.PVY4 = 0;   % no ply steer

% ---- longitudinal, pure slip -----------------------------------------
T.PCX1 = P.Pacejka.LonC;
T.PDX1 = P.TireMu;
T.PDX2 = 0; T.PDX3 = 0;
T.PEX1 = P.Pacejka.LonE;
T.PEX2 = 0; T.PEX3 = 0; T.PEX4 = 0;
T.PKX1 = P.Derived.LongSlipStiffness / T.FNOMIN;
T.PKX2 = 0; T.PKX3 = 0;
T.PHX1 = 0; T.PHX2 = 0;
T.PVX1 = 0; T.PVX2 = 0;

% ---- pressure sensitivity: none --------------------------------------
% The tyre runs at NOMPRES and the pressure input is set equal to it, so
% these would multiply zero anyway. Zeroed so that a future change to the
% pressure input cannot quietly introduce a passenger car's response.
for f = {'PPX1','PPX2','PPX3','PPX4','PPY1','PPY2','PPY3','PPY4','PPY5','PPMX1','PPZ1','PPZ2'}
    T.(f{1}) = 0;
end

% ---- rolling resistance: OFF -----------------------------------------
% Unlike the Fiala block there is no rollingType switch here; MF always
% computes My. Zeroing the QSY set turns it off so the plant keeps its own
% Crr, which is a settled number and is applied as an axle torque.
for i = 1:8, T.(sprintf('QSY%d',i)) = 0; end

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
T.Q_RE0 = 1;  T.Q_V1 = 0;  T.Q_V2 = 0;

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

% ---- operating envelope ----------------------------------------------
T.FZMIN = 0;    % a lifted wheel must really carry nothing
T.FZMAX = 10 * T.FNOMIN;
T.PRESMIN = T.NOMPRES/2;  T.PRESMAX = T.NOMPRES*2;

if ~isfolder(outdir), mkdir(outdir); end
f = fullfile(outdir,'ifssim_tyre.mat');
ifssim_tyre = T;                                     %#ok<NASGU>
save(f,'ifssim_tyre');
fprintf('wrote %s\n', f);
fprintf('  MF 6.2 from settings.json: mu %.2f, Ca %.0f N/rad, Ck %.0f N, Fz0 %.0f N, R %.3f m\n', ...
        T.PDY1, -T.PKY1*T.FNOMIN, T.PKX1*T.FNOMIN, T.FNOMIN, T.UNLOADED_RADIUS);
end
