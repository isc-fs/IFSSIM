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
%   It shares car_spec/settings.json with the plant, and it loads the SAME
%   Magic Formula coefficients the plant's tyre block loads. A design model
%   that drifts from its plant is worse than no design model, so the two
%   cannot be given different tyres by accident.

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
M.CdA  = P.CdA;
M.aeroF= P.AeroBalanceFront;
M.Crr  = P.RollingResistance;

M.ackermann = P.Assumed.AckermannFraction;
M.maxSteer  = P.MaxSteerAngle * pi/180;

% Wheel positions, body frame, ISO 8855: x forward, y LEFT. Order FL FR RL RR,
% the same order the plant uses -- getting these out of step between the two
% models would make every comparison meaningless in a way that looks like a
% physics disagreement.
M.wx = [ M.a;      M.a;     -M.b;     -M.b];
M.wy = [ M.tF/2;  -M.tF/2;   M.tR/2;  -M.tR/2];

% ---- the tyre, from the plant's own parameter set ---------------------
T = load(tyreMat);  T = T.ifssim_tyre;
M.Fz0  = T.FNOMIN;
M.PCY1 = T.PCY1;  M.PDY1 = T.PDY1;  M.PDY2 = T.PDY2;  M.PEY1 = T.PEY1;
M.PKY1 = T.PKY1;  M.PKY2 = T.PKY2;  M.PKY4 = T.PKY4;
M.mu   = T.PDY1;
M.tyreSource = tyreMat;
end
