function G = susp_geometry(H, cog)
%SUSP_GEOMETRY  Static suspension geometry from one corner's hardpoints.
%
%   G = SUSP_GEOMETRY(H)        H from sm_hardpoints (car frame, ground at z=0)
%   G = SUSP_GEOMETRY(H, cog)   cog.x  CoG x in the car frame      (default 0)
%                               cog.h  CoG height                  (default P.CoGHeight)
%                               cog.brakeFront  brake bias front   (default 0.6)
%
%   The definitions are the workbook's own (Susp_Geometry, FASE 1-3, after
%   Milliken RCVD Ch.17 and Dixon Ch.3-5), so the same hardpoints give the
%   same numbers -- test_susp_geometry reproduces every figure the sheet
%   computes, to its last digit. Two things differ, deliberately, and both are
%   reported rather than hidden:
%
%     GROUND. These use z = 0 of H, which sm_hardpoints puts at the ground
%     the car's spec states. The sheet put it at its own Z = 0.
%     ANTI-SQUAT. The sheet's "anti-squat" is computed with the 40% rear BRAKE
%     bias, which makes it anti-LIFT under braking. Under rear DRIVE (this car
%     is RWD, outboard hubs) the drive fraction is 1. Both are given.
%
%   Lengths in metres, angles in degrees. Left corner (y > 0).

if nargin < 2, cog = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','plant'));
if ~isfield(cog,'x'), cog.x = 0; end
if ~isfield(cog,'h') || ~isfield(cog,'brakeFront')
    P = ifssim_params([], H.car);
    if ~isfield(cog,'h'), cog.h = P.CoGHeight; end
    if ~isfield(cog,'brakeFront'), cog.brakeFront = 0.6; end
end
front = strcmp(H.axle, 'front');

LBJ = H.lca_outer;  UBJ = H.uca_outer;  WC = H.wheel_centre;  CP = H.contact;

% ---- steering axis (FASE 1) -------------------------------------------
d = UBJ - LBJ;
G.kpi_deg    = atand(abs(d(2)) / d(3));            % front view, from vertical
% caster: top of the axis REARWARD is positive. The car's x is forward, so
% rearward is -x: positive caster has d(1) < 0.
G.caster_deg = atand(-d(1) / d(3));
t = -LBJ(3) / d(3);                                % axis meets the ground
gnd = LBJ + t*d;
G.scrub_m    = CP(2) - gnd(2);                     % + : ground point inboard
G.trail_m    = gnd(1) - CP(1);                     % + : ground point ahead
tw = (WC(3) - LBJ(3)) / d(3);
G.kpOffset_m = WC(2) - (LBJ(2) + tw*d(2));
G.spindle_m  = H.spindle(2) - WC(2);
% Static camber and toe from the spindle: the wheel's axis points outboard
% along WC -> spindle. Tilted UP going outboard puts the top of the wheel IN:
% negative SAE camber. Pointing FORWARD going outboard is toe-OUT.
s = H.spindle - WC;
G.camber_deg = -atand(s(3) / s(2));
G.toe_deg    = -atand(s(1) / s(2));                % + toe-in

% ---- front view: instant centre, roll centre (FASE 2) ------------------
[A, B] = fv_line(H.lca_front, H.lca_rear, LBJ);
[Cc, D] = fv_line(H.uca_front, H.uca_rear, UBJ);
G.icFV = intersect2(A, B, Cc, D);                  % [y z]
cp = CP([2 3]);
tRC = -G.icFV(1) / (cp(1) - G.icFV(1));
G.rc_m    = G.icFV(2) + tRC*(cp(2) - G.icFV(2));
G.fvsa_m  = norm(G.icFV - cp);

% ---- side view: anti-dive, anti-lift, anti-squat (FASE 3) --------------
% Each arm's INNER pivot line, in side view; their intersection is the
% side-view instant centre. The slope from the contact patch to it, carried
% to the CoG, gives the height the anti line reaches there.
sv = intersect2(H.lca_front([1 3]), H.lca_rear([1 3]), H.uca_front([1 3]), H.uca_rear([1 3]));
G.icSV = sv;                                       % [x z]
m = (sv(2) - CP(3)) / (sv(1) - CP(1));
zAtCog = CP(3) + m*(cog.x - CP(1));
G.svsaHeightAtCog_m = zAtCog;
if front
    G.antiDive_pct = 100 * zAtCog / cog.h * cog.brakeFront;
else
    G.antiLift_pct  = 100 * zAtCog / cog.h * (1 - cog.brakeFront);
    G.antiSquat_pct = 100 * zAtCog / cog.h * 1.0;   % RWD, outboard hubs
end
end

% -------------------------------------------------------------------------
function [A, B] = fv_line(pf, pr, bj)
%FV_LINE  An arm in front view: its inner pivot line evaluated at the ball
%   joint's x (Dixon 3.2), to the ball joint. [y z] pairs.
t = (bj(1) - pf(1)) / (pr(1) - pf(1));
inner = pf + t*(pr - pf);
A = inner([2 3]);  B = bj([2 3]);
end

function p = intersect2(a, b, c, d)
%INTERSECT2  Lines a->b and c->d in a plane.
r = b - a;  q = d - c;
den = r(1)*(-q(2)) - r(2)*(-q(1));
if abs(den) < 1e-15, p = [Inf Inf]; return; end
s = ((c(1)-a(1))*(-q(2)) - (c(2)-a(2))*(-q(1))) / den;
p = a + s*r;
end
