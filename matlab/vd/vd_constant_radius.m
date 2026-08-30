function C = vd_constant_radius(R, M, speeds)
%VD_CONSTANT_RADIUS  The constant-radius test, and the understeer gradient.
%
%   Drive a fixed circle at increasing speed and record the steering angle
%   needed to hold it. The classical characterisation of a car falls out of
%   the result:
%
%     delta = L/R + K * ay/g
%
%   L/R is the ACKERMANN angle -- the steering pure geometry would need at
%   walking pace. K is the UNDERSTEER GRADIENT, in degrees of extra steering
%   per g of lateral acceleration. K > 0 is understeer (you must add lock as
%   you go faster), K < 0 is oversteer, K = 0 is neutral.
%
%   This is the single most useful number a vehicle dynamicist can be handed
%   about a car, and it is the one the setup levers move: roll stiffness
%   distribution, weight distribution, tyre load sensitivity.

if nargin < 2 || isempty(M), M = dualtrack_build(); end
if nargin < 3 || isempty(speeds), speeds = 4:1:20; end

C = struct('R',R,'v',[],'delta',[],'ay',[],'beta',[], ...
           'alphaF',[],'alphaR',[],'settled',logical([]), ...
           'K',NaN,'ackermann',M.L/R,'v_max',NaN);

for v = speeds
    [okv, d, S] = hold_circle(v, R, M);
    if ~okv, break; end
    C.v(end+1)      = v;        C.delta(end+1)  = d;
    C.ay(end+1)     = S.ay;     C.beta(end+1)   = S.beta;
    C.alphaF(end+1) = S.alphaF; C.alphaR(end+1) = S.alphaR;
    C.settled(end+1) = true;
end

% ---- the limit speed -------------------------------------------------
% Found from a PHYSICAL criterion, not from whether a solver converged.
%
% The car can hold the circle at speed v if some steer angle produces the
% required yaw rate v/R. Yaw rate is not monotonic in steer -- it rises to the
% grip limit and falls away past it -- so the test is whether its MAXIMUM over
% steer still reaches v/R. That maximum is a smooth, decreasing function of
% speed, so bisecting on it is well posed.
%
% The previous version broke the sweep at the first speed where a root-find
% failed, which is a knife edge: the failures are not monotonic in speed, so a
% coarse sweep stepped over them and reported a higher limit than a fine one.
% It was not converged -- halving the step moved the answer by up to 1 m/s,
% and with it the skid-pad time, which is the number the team is meant to
% check against a stopwatch.
if ~isempty(C.v)
    g  = @(v) max_yaw(v, M) - v/R;
    lo = C.v(end);                       % holds
    hi = lo;
    for k = 1:40                         % walk up until it does not
        hi = hi + 0.25;
        if g(hi) < 0, break; end
    end
    if g(hi) < 0
        for k = 1:30
            mid = 0.5*(lo+hi);
            if g(mid) >= 0, lo = mid; else, hi = mid; end
        end
    end
    C.v(end) = lo;
    % At the limit there is NO steady state to solve for -- that is what makes
    % it the limit. Asking dualtrack_trim for one there returns an unsettled,
    % sign-flipped solution: r comes back NEGATIVE, the car notionally turning
    % the other way, with the corner loads mirrored. The tables never showed
    % it; the wheel-travel plot did, as outer and inner swapping over.
    %
    % So the limit point's lateral acceleration is taken from the DEFINITION of
    % the manoeuvre rather than from a solve. Holding radius R at speed v is
    % ay = v^2/R, exactly, by kinematics.
    C.ay(end)     = C.v(end)^2 / R;
    C.settled(end) = false;
end

if numel(C.v) >= 3
    % Fit only the linear range. Past about 0.5 g the tyres are bending and a
    % straight line through the whole sweep would report a gradient the car
    % never has.
    lin = abs(C.ay) <= 0.5*9.81;
    if nnz(lin) >= 2
        p   = polyfit(C.ay(lin)/9.81, C.delta(lin)*180/pi, 1);
        C.K = p(1);                           % deg per g
    end
    C.v_max = C.v(end);
end
end

function [ok, d, S] = hold_circle(v, R, M)
%HOLD_CIRCLE  Can the car hold this circle at this speed, and with what lock?
ok = false;  d = NaN;  S = [];
rt = v/R;
f  = @(x) trimr(x, v, M) - rt;
if f(M.maxSteer) < 0 && max_yaw(v, M) < rt, return; end
try
    d = fzero(f, [0 M.maxSteer], optimset('TolX',1e-6,'Display','off'));
catch
    return;
end
S  = dualtrack_trim(v, d, M);
ok = S.settled;
end

function d = hold_lock(v, R, M)
%HOLD_LOCK  The steer angle that holds the circle, or the one that comes closest.
d = NaN;
try
    d = fzero(@(x) trimr(x, v, M) - v/R, [0 M.maxSteer], ...
              optimset('TolX',1e-6,'Display','off'));
catch
    [~, d] = max_yaw(v, M);
end
end

function [rmax, dbest] = max_yaw(v, M)
%MAX_YAW  The most yaw rate this car can produce at this speed, over all lock.
%
%   Scanned and then refined, deliberately not root-found: near the peak the
%   curve is flat and a root-finder has nothing to bracket, while a scan just
%   works. Unsettled points are excluded rather than trusted -- past the limit
%   Newton can land on a spun equilibrium with a large yaw rate that the car
%   cannot actually sustain.
ds = linspace(1e-4, M.maxSteer, 40);
rr = nan(size(ds));
for i = 1:numel(ds)
    S = dualtrack_trim(v, ds(i), M);
    if S.settled, rr(i) = S.r; end
end
[rmax, i] = max(rr);
if isempty(rmax) || all(isnan(rr)), rmax = -inf; dbest = NaN; return; end
lo = ds(max(i-1,1));  hi = ds(min(i+1,numel(ds)));
ds2 = linspace(lo, hi, 20);
for i2 = 1:numel(ds2)
    S = dualtrack_trim(v, ds2(i2), M);
    if S.settled && S.r > rmax, rmax = S.r; dbest = ds2(i2); end
end
if ~exist('dbest','var'), dbest = ds(i); end
end

function r = trimr(d, v, M)
S = dualtrack_trim(v, d, M);
r = S.r;
end
