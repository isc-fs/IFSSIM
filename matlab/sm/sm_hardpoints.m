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

H.axle  = lower(axle);
H.track = t;
H.wheel_radius = r;
end
