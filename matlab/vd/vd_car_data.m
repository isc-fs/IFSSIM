function D = vd_car_data(csvPath, opts)
%VD_CAR_DATA  Load a logged run and make it fit to compare a model against.
%
%   D = VD_CAR_DATA(CSV) reads a file written by
%   tools/vd_validation/bag_to_vd_csv.py and returns it cleaned, filtered and
%   with the two quantities a model actually needs as inputs -- forward speed
%   and road-wheel steer -- derived from what the bag recorded.
%
%   This is the front half of the thesis method (Diwakar, TU Delft 2018, ch. 6):
%   drive the model with what the car did, compare what it produced. The back
%   half is validate_vs_car.
%
%   FOUR THINGS THE DATA FORCES, none of them visible from the column names:
%
%   1. THE IMU HAS IMPOSSIBLE SAMPLES. One recorded run reports ax = -160 m/s^2
%      (16 g) and ay = +37.9 m/s^2. A 275 kg car on slicks does not do that.
%      Left in, a handful of such samples dominate any RMSE. They are gated on
%      a physical limit and interpolated across, and the fraction removed is
%      reported, because a run that is mostly glitches should not be scored.
%
%   2. THERE IS NO TORQUE. The thesis replayed measured wheel torques. These
%      exports carry motor rpm, steering and IMU only, so speed is derived
%      from rpm and the model is driven by the SPEED the car had, not the
%      force that produced it.
%
%   3. THE STEERING SCALE IS NOT KNOWN. /steering_angle is documented as the
%      road-wheel command in radians, but it reaches +/-0.50 rad against a
%      measured lock of 18.2 deg (0.318 rad), so it cannot simply be that.
%      Rather than guess, the scale is FITTED: at low speed a car is close to
%      kinematic, r = v*delta/L, so measured yaw rate against measured speed
%      determines how steer_rad maps to road-wheel angle. Crawl-speed bags --
%      useless for validating the dynamics -- are exactly right for this.
%
%   4. SMOOTHING IS SAVITZKY-GOLAY, as in the thesis. It fits a low-order
%      polynomial across a sliding window, which preserves peaks a moving
%      average would flatten. Implemented here directly: the Signal Processing
%      Toolbox is licensed but sgolayfilt is not installed.
%
%   OPTS fields, all optional:
%     steerScale   fix k instead of fitting it (road-wheel rad per steer_rad)
%     sgFrame      Savitzky-Golay window, samples, odd       (default ~0.1 s)
%     sgOrder      polynomial order                         (default 3)
%     gateAccel    reject |ax|,|ay| above this, m/s^2        (default 3 g)
%     gateYaw      reject |r| above this, rad/s              (default 3)

if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','plant'), fullfile(here,'..','spec'));
P = ifssim_load_workspace();

T = readtable(csvPath);
need = {'t_s','steer_rad','motor_rpm','yaw_rate_rps','ax_mps2','ay_mps2'};
miss = setdiff(need, T.Properties.VariableNames);
if ~isempty(miss)
    error('vd_car_data:columns','%s is missing %s', csvPath, strjoin(miss,', '));
end

D.file = csvPath;
D.t    = T.t_s(:) - T.t_s(1);
dt     = median(diff(D.t));
D.fs   = 1/dt;

% ---- 1. physical gate --------------------------------------------------
g = 9.81;
gA = getf(opts,'gateAccel', 3*g);
gR = getf(opts,'gateYaw',   3.0);
[D.ax, nax] = gate(T.ax_mps2,      gA);
[D.ay, nay] = gate(T.ay_mps2,      gA);
[D.r,  nr ] = gate(T.yaw_rate_rps, gR);
n = numel(D.t);
D.rejected = struct('ax', nax/n, 'ay', nay/n, 'r', nr/n);

% ---- 4. Savitzky-Golay -------------------------------------------------
frame = getf(opts,'sgFrame', 2*floor(0.05*D.fs) + 1);   % ~0.1 s, forced odd
order = getf(opts,'sgOrder', 3);
D.ax_f = sgolay_smooth(D.ax, order, frame);
D.ay_f = sgolay_smooth(D.ay, order, frame);
D.r_f  = sgolay_smooth(D.r,  order, frame);
D.sg   = [order frame];

% ---- 2. speed from motor rpm ------------------------------------------
% /motor_rpm is at the MOTOR, so the gear comes out before the wheel radius
% goes in. Wheel radius is the measured 0.202 m, not the 0.228 the sim once
% assumed.
D.vx   = T.motor_rpm(:) * 2*pi/60 / P.GearRatio * P.WheelRadius;
D.vx_f = sgolay_smooth(D.vx, order, frame);

% ---- 3. steering scale -------------------------------------------------
s = T.steer_rad(:);
D.has_steer = any(isfinite(s)) && any(abs(s(isfinite(s))) > 1e-4);
if ~D.has_steer
    D.delta = NaN(size(D.t));
    D.steer = struct('k',NaN,'bias',NaN,'R2',NaN,'n',0,'note', ...
        'no steering recorded -- this run cannot be replayed');
    return
end
s(~isfinite(s)) = 0;

if isfield(opts,'steerScale')
    k = opts.steerScale;  b = 0;  R2 = NaN;  nfit = 0;
    note = 'scale fixed by caller';
else
    [k, b, R2, nfit] = fit_steer_scale(s, D.vx_f, D.r_f, P.Wheelbase, D.fs);
    note = 'fitted from the kinematic regime';
end
D.delta = k * s;
D.steer = struct('k',k,'bias',b,'R2',R2,'n',nfit,'note',note, ...
    'max_delta_deg', max(abs(D.delta))*180/pi, ...
    'lock_deg', P.MaxSteerAngle);
end

% =========================================================================
function [x, nbad] = gate(x, lim)
x = x(:);
bad = ~isfinite(x) | abs(x) > lim;
nbad = nnz(bad);
if nbad == 0, return; end
good = find(~bad);
if numel(good) < 2, x(:) = 0; return; end
x(bad) = interp1(good, x(good), find(bad), 'linear', 'extrap');
end

function y = sgolay_smooth(x, order, frame)
%SGOLAY_SMOOTH  Savitzky-Golay smoothing by a least-squares convolution kernel.
x = x(:);
half = (frame - 1)/2;
k = (-half:half)';
A = k .^ (0:order);              % frame x (order+1) Vandermonde
H = pinv(A);                     % (order+1) x frame
h = H(1,:)';                     % row 1: the fitted value at the centre
% Mirror-pad so the ends are smoothed by the same polynomial fit rather than
% dragged towards zero by an implicit zero pad.
xp = [flipud(x(2:half+1)); x; flipud(x(end-half:end-1))];
y = conv(xp, flipud(h), 'valid');
end

function [k, b, R2, n] = fit_steer_scale(s, vx, r, L, fs)
%FIT_STEER_SCALE  Road-wheel angle per unit steer_rad, from kinematic yaw.
%
%   r = (v/L)*k*s + b.  Restricted to samples where the kinematic relation is
%   believable: moving (v > 1 m/s, so yaw rate is not pure gyro noise), and
%   steering HELD (small d(steer)/dt), because in a transient the car lags
%   the command and the kinematic model over-predicts. The intercept absorbs
%   gyro bias.
ds  = [0; diff(s)] * fs;
use = vx > 1.0 & abs(ds) < 0.2 & abs(s) > 0.02;
x   = (vx(use) / L) .* s(use);
y   = r(use);
n   = nnz(use);
if n < 50
    k = NaN; b = NaN; R2 = NaN; return
end
X = [x, ones(n,1)];
c = X \ y;
k = c(1);  b = c(2);
res = y - X*c;
R2 = 1 - sum(res.^2) / sum((y - mean(y)).^2);
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
