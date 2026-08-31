function M = dualtrack_build(P, tyreMat)
%DUALTRACK_BUILD  Constants for the dual-track design model.
%
%   THIS IS THE SECOND MODEL, and the reason there are two is the whole point
%   of docs/vehicle_dynamics_alignment.md. The Simulink plant in matlab/plant
%   is the REFERENCE PLANT: rigid-body chassis, spring/damper suspension with
%   states, Magic Formula with relaxation, a Simscape accumulator. It plays
%   the role VI-CarRealTime plays in Escofet's thesis and CarSim plays in
%   Fricano's -- the thing you validate against.
%
%   This is the DESIGN model. It is what the literature actually builds a
%   controller and a state estimator on: two states, algebraic load transfer,
%   no suspension dynamics, speed prescribed rather than integrated. Escofet
%   §4.2, "Dual Track NTV".
%
%   Why it is worth having a deliberately simpler model:
%     - it runs in milliseconds, so it can be swept, fitted and inverted
%     - yaw rate is an OUTPUT of two states, so a yaw-rate reference and an
%       observer fall straight out of it; the plant cannot be inverted
%     - it is small enough to port to the pipeline, which today steers on a
%       KINEMATIC bicycle -- no tyres, no slip -- and has the vy drift to
%       show for it
%
%   It shares car_spec with the plant, and derives the SAME Magic Formula
%   coefficients the plant's tyre block is built from. A design model that
%   drifts from its plant is worse than no design model, so test_dualtrack
%   asserts the two derivations agree.

if nargin < 1 || isempty(P), P = ifssim_params(); end
if nargin < 2 || isempty(tyreMat)
    tyreMat = fullfile(fileparts(mfilename('fullpath')), '..', 'plant', ...
                       'models', 'ifssim_tyre.mat');
end

M = struct();
M.m   = P.Mass;
M.Izz = P.Assumed.Izz;
M.g   = 9.81;
M.L   = P.Wheelbase;
M.a   = P.Derived.aFront;      % CoG -> front axle
M.b   = P.Derived.bRear;       % CoG -> rear axle
M.tF  = P.TrackFront;
M.tR  = P.TrackRear;
M.h   = P.CoGHeight;
M.wdF = P.WeightDistFront;

% Roll centres and roll stiffness distribution. These are what split lateral
% load transfer into the part carried through the suspension LINKS (geometric,
% instant, no roll needed) and the part carried through the SPRINGS (elastic,
% distributed by roll stiffness). The reference plant models only the second
% -- its roll centre is at ground level, see build_tiresuspension -- so this
% is one place the design model is currently RICHER than the plant it is
% validated against, which is an odd state of affairs and is tracked in
% docs/vehicle_dynamics_alignment.md.
M.hrcF = P.RollCenterFront;
M.hrcR = P.RollCenterRear;
M.KrF  = P.RollStiffnessFront;
M.KrR  = P.RollStiffnessRear;

M.rho  = P.Assumed.AirDensity;
M.ClA  = P.ClA;
M.aeroF= P.AeroBalanceFront;
% CdA and RollingResistance are deliberately ABSENT. This model prescribes
% speed rather than integrating it, so drag and rolling resistance change
% nothing it computes. They were carried here as unread struct fields, which
% made the model look like it charged for drag when it does not -- and made
% vd_parameters claim CdA reached it. If the wing trade needs costing, that is
% fs_track_cycle's job, not this one's.

% ---- suspension kinematics -------------------------------------------
% Rates, not hardpoints. Camber gain is how much of body roll the geometry
% takes back out of the tyre; a gain of 1 keeps the wheel upright through the
% corner. Bump steer is toe per metre of travel and is a defect, so it is zero
% by design. See car_spec's Susp group.
M.camF   = P.Susp.StaticCamberFront * pi/180;
M.camR   = P.Susp.StaticCamberRear  * pi/180;
M.cgainF = P.Susp.CamberGainFront;
M.cgainR = P.Susp.CamberGainRear;
M.bumpF  = P.Susp.BumpSteerFront * pi/180;
M.bumpR  = P.Susp.BumpSteerRear  * pi/180;
M.camSens= P.Susp.CamberGripSensitivity;
M.kwF    = P.Derived.WheelRateFront;
M.kwR    = P.Derived.WheelRateRear;

M.ackermann = P.Assumed.AckermannFraction;
M.maxSteer  = P.MaxSteerAngle * pi/180;

% Wheel positions, body frame, ISO 8855: x forward, y LEFT. Order FL FR RL RR,
% the same order the plant uses -- getting these out of step between the two
% models would make every comparison meaningless in a way that looks like a
% physics disagreement.
M.wx = [ M.a;      M.a;     -M.b;     -M.b];
M.wy = [ M.tF/2;  -M.tF/2;   M.tR/2;  -M.tR/2];

% ---- the tyre ---------------------------------------------------------
% DERIVED from P, by exactly the formulas build_tyre_paramset uses to write
% the plant's Magic Formula set. Not loaded from that file, and the difference
% matters: the file is generated from the settings.json on disk, so a study
% that overrides TireMu would have gone straight past it and reported that
% changing the grip of the tyre changes nothing about the car.
%
% Deriving instead of loading does not weaken the guarantee that the two
% models share a tyre -- it strengthens it. test_dualtrack asserts these come
% out equal to the plant's file for the car as built, so the agreement is
% CHECKED rather than assumed, and it is checked against the same file the
% tyre block actually loads.
M.Fz0  = P.Derived.NominalWheelLoad;
M.PCY1 = P.Pacejka.LatC;
M.PDY1 = P.TireMu;
M.PDY2 = P.Assumed.TyreLoadSensitivity;
M.PEY1 = P.Pacejka.LatE;
M.PKY4 = 2;
M.PKY2 = P.Assumed.TyreStiffnessPeakLoadRatio;
M.PKY1 = -P.Derived.CorneringStiffness / ...
         (M.Fz0 * sin(M.PKY4 * atan(1/M.PKY2)));
M.mu   = M.PDY1;
M.tyreSource = tyreMat;
end
