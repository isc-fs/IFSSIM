function H = sm_hardpoints(axle)
%SM_HARDPOINTS  Double-wishbone pickup points, one corner, in body coordinates.
%
%   H = SM_HARDPOINTS('front') or SM_HARDPOINTS('rear') returns the hardpoints
%   for the LEFT corner of that axle. Frame is the car's: x FORWARD, y LEFT,
%   z UP, origin at the CoG ground projection.
%
%   THESE ARE ASSUMED, and that is the whole point of the file existing.
%
%   The plant we have today does not contain suspension geometry at all -- it
%   contains its CONSEQUENCES, as three numbers that were chosen rather than
%   derived: Susp.MotionRatioFront (0.70), Susp.CamberGainFront (0.80) and
%   Susp.ArbArmRadiusFront (0.20). A multibody corner computes all three from
%   the geometry instead, so those three assumptions collapse into one set of
%   coordinates -- which is a set the suspension department can actually
%   measure, argue about, and replace from CAD.
%
%   Until the CAD numbers arrive these are a plausible FS-car layout built
%   from the car's real track and a typical upright. Every value is a
%   placeholder; none of them came off the IFS-08. Replace them wholesale
%   rather than tuning them one at a time -- a hardpoint set is a geometry,
%   not seven independent knobs.

P = ifssim_load_workspace();
switch lower(axle)
    case 'front', t = P.TrackFront;  x0 =  P.Derived.aFront;
    case 'rear',  t = P.TrackRear;   x0 = -P.Derived.bRear;
    otherwise, error('sm_hardpoints:axle','axle must be front or rear');
end
half = t/2;
r    = P.WheelRadius;

% Chassis-side pivots. The lower arm sits low and the upper high; the
% difference in their inboard heights is most of what sets the roll centre.
H.lca_front = [x0+0.120,  0.180, 0.110];
H.lca_rear  = [x0-0.120,  0.180, 0.110];
H.uca_front = [x0+0.100,  0.220, 0.280];
H.uca_rear  = [x0-0.100,  0.220, 0.280];

% Upright-side ball joints. Their y offset from the wheel centreline is the
% kingpin offset; the z spread is the kingpin length.
H.lca_outer = [x0,        half-0.045, 0.120];
H.uca_outer = [x0,        half-0.075, 0.310];

% Track rod. Its inboard y and z are what set bump steer, so this is the
% hardpoint the department will move first.
H.tie_inner = [x0+0.145,  0.165, 0.140];
H.tie_outer = [x0+0.135,  half-0.055, 0.150];

% Wheel centre and contact patch.
H.wheel_centre = [x0, half, r];
H.contact      = [x0, half, 0];

% Pushrod, rocker, damper. These are what turn Susp.MotionRatioFront from an
% assumed 0.70 into a computed number -- the ratio of damper travel to wheel
% travel falls out of where these five points are.
%
% Pushrod-on-lower-arm, rising inboard to a rocker on the chassis top, with
% the damper lying across the car. The common FS front layout, and as much a
% placeholder as everything above.
%
% Laid out to the standard rule: at static ride height EACH ROCKER ARM IS
% PERPENDICULAR TO THE LINK IT DRIVES. That puts the leverage at its maximum
% and the toggle -- where an arm lines up with its link and the motion ratio
% passes through zero -- as far from the travel range as the geometry allows.
% The first placeholder here ignored that: its damper arm sat at 60 deg to
% the damper, the linkage toggled inside the travel, and the motion ratio
% went from +0.70 in droop to -2.54 in bump.
H.pr_lca      = [x0,        half-0.110, 0.135];   % pushrod pickup ON the lower arm
H.rk_pivot    = [x0,        0.200,      0.420];   % rocker pivot, chassis
H.rk_axis     = [1, 0, 0];                        % rocker turns about this, chassis frame
H.rk_pr       = [x0,        0.248,      0.456];   % 60 mm arm, perpendicular to the pushrod
H.rk_dmp      = [x0,        0.200,      0.480];   % 60 mm arm, straight up
H.dmp_chassis = [x0,       -0.080,      0.480];   % horizontal damper, perpendicular to that arm

H.axle  = lower(axle);
H.track = t;
H.wheel_radius = r;
end
