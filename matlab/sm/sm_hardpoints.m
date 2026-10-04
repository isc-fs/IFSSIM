function H = sm_hardpoints(axle, opts)
%SM_HARDPOINTS  The car's double-wishbone hardpoints, one corner, in the car's frame.
%
%   H = SM_HARDPOINTS('front')      the LEFT front corner of the ACTIVE car
%   H = SM_HARDPOINTS('rear')       the left rear
%   H = SM_HARDPOINTS(axle, opts)   opts.car       another car ('IFS-09')
%                                   opts.GroundZ_mm the sheet z taken as the
%                                                   ground; 0 reproduces the
%                                                   workbook's own analysis
%
%   Frame: the car's. x FORWARD, y LEFT, z UP, origin at the CoG's ground
%   projection (the plant's). Metres.
%
%   THESE ARE THE CAR'S, NOT PLACEHOLDERS. They come from the car's hardpoint
%   file (C.Hardpoints in its spec), which for the IFS-08 is the team
%   workbook's Susp_Geometry table, row for row. The placeholder set this
%   replaced gave a camber gain of 0.29 and a motion ratio of 0.68 that were
%   properties of invented points; the numbers this produces are the car's.
%
%   THE CONVERSION, in one place, because the sheet's frame needs it:
%     x_car = aFront + XSign * (x_sheet - x_front_wheel_centre)
%             XSign = -1: the sheet's x grows rearward (see ifs08.m);
%             aFront is the plant's CoG-to-front-axle distance, so the
%             wheel centres land exactly where the plant puts its axles
%     y_car = y_sheet                    (left positive in both)
%     z_car = z_sheet - GroundZ          GroundZ from the car's spec
%
%   Fields, as build_sm_corner and sm_corner_sweep use them:
%     lca_front lca_rear lca_outer   lower wishbone (F1 F2 F3 / R1 R2 R3)
%     uca_front uca_rear uca_outer   upper wishbone (F4 F5 F6 / R4 R5 R6)
%     pr_lca rk_pr                   pushrod: on the lower arm, on the rocker (7, 8)
%     tie_outer tie_inner            track rod / toe link (9, 10)
%     dmp_chassis rk_dmp             damper: chassis end, rocker end (11, 12)
%     rk_pivot rk_axis               rocker axis: midpoint and direction of 13 -> 14
%     wheel_centre spindle contact   15, 16, and the ground under the centre

if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if ~isfield(opts,'car') || isempty(opts.car), opts.car = ifssim_car(); end
C = car_spec(opts.car);
P = ifssim_params([], opts.car);
if ~isfield(C, 'Hardpoints')
    error('sm_hardpoints:none', '%s has no hardpoint file in its spec (C.Hardpoints).', opts.car);
end
hp = C.Hardpoints;
groundZ = hp.GroundZ_mm;
if isfield(opts,'GroundZ_mm'), groundZ = opts.GroundZ_mm; end

T = readtable(fullfile(here,'..','spec','cars', hp.File), 'TextType','string');
pt = @(id) sheet_point(T, id);

switch lower(axle)
    case 'front', p = 'F';
    case 'rear',  p = 'R';
    otherwise, error('sm_hardpoints:axle','axle must be front or rear');
end
xFrontWC = pt('F15');  xFrontWC = xFrontWC(1);
conv = @(v) [P.Derived.aFront + hp.XSign*(v(1) - xFrontWC)/1000, v(2)/1000, (v(3) - groundZ)/1000];
g = @(n) conv(pt(sprintf('%s%d', p, n)));

H.lca_front   = g(1);   H.lca_rear = g(2);   H.lca_outer = g(3);
H.uca_front   = g(4);   H.uca_rear = g(5);   H.uca_outer = g(6);
H.pr_lca      = g(7);   H.rk_pr    = g(8);
H.tie_outer   = g(9);   H.tie_inner = g(10);
H.dmp_chassis = g(11);  H.rk_dmp   = g(12);
a1 = g(13);  a2 = g(14);
H.rk_pivot    = (a1 + a2)/2;
H.rk_axis     = (a2 - a1) / norm(a2 - a1);
H.wheel_centre = g(15);
H.spindle      = g(16);
H.contact      = [H.wheel_centre(1:2), 0];

% FRONT-AXLE "FRONT" PIVOT MUST BE THE FORWARD ONE after the conversion: a
% cheap guard that the x direction is right, since getting it wrong mirrors
% the whole corner and nothing else would fail.
if H.lca_front(1) <= H.lca_rear(1)
    error('sm_hardpoints:mirrored', ['the "front" lower pivot is not forward of the "rear" one: ' ...
          'the x direction of the hardpoint conversion is wrong.']);
end

H.axle  = lower(axle);
H.track = 2 * H.wheel_centre(2);
H.wheel_radius = H.wheel_centre(3);
H.car    = opts.car;
H.groundZ_mm = groundZ;
H.source = hp.Source;
end

function v = sheet_point(T, id)
i = find(T.id == id, 1);
if isempty(i), error('sm_hardpoints:missing', 'hardpoint %s is not in the file', id); end
v = [T.x_mm(i), T.y_mm(i), T.z_mm(i)];
end
