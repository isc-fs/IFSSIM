function Y = dualtrack_sim(t, delta, vx, M, x0)
%DUALTRACK_SIM  Run the dual-track model over a prescribed manoeuvre.
%
%   Y = DUALTRACK_SIM(t, delta, vx, M) integrates the two states over the time
%   vector t, given road-wheel steer delta(t) and forward speed vx(t). Both
%   may be scalars, in which case they are held.
%
%   This is the shape every validation in the literature takes: the steering
%   and speed a vehicle had, in; the yaw rate it should have produced, out.
%   Feed it a bag through tools/vd_validation/bag_to_vd_csv.py and the
%   comparison is Escofet eq. (12).
%
%   RK4 at the data's own step. The states are smooth and the stiffest thing
%   here is the tyre curve, so there is nothing to be gained from a variable
%   step and a lot to be said for a fixed one: it makes runs comparable.

if nargin < 5 || isempty(x0), x0 = [0;0]; end
t = t(:);  n = numel(t);
if isscalar(delta), delta = repmat(delta,n,1); end
if isscalar(vx),    vx    = repmat(vx,   n,1); end
delta = delta(:);  vx = vx(:);

% Longitudinal acceleration drives the longitudinal load transfer. Taking it
% from the speed trace rather than asking for it keeps the caller honest:
% ax and vx that disagree would put load on an axle the car never had.
ax = [0; diff(vx)./max(diff(t),eps)];
ax(~isfinite(ax)) = 0;

Y = struct();
Y.t = t;  Y.delta = delta;  Y.vx = vx;  Y.ax = ax;
Y.vy = zeros(n,1);  Y.r = zeros(n,1);  Y.ay = zeros(n,1);
Y.Fz = zeros(n,4);  Y.alpha = zeros(n,4);  Y.Fy = zeros(n,4);
Y.beta = zeros(n,1);

x = x0(:);
for k = 1:n
    uk = [delta(k); vx(k); ax(k)];
    [~, d] = dualtrack_rhs(x, uk, M);
    Y.vy(k) = x(1);  Y.r(k) = x(2);  Y.ay(k) = d.ay;
    Y.Fz(k,:) = d.Fz.';  Y.alpha(k,:) = d.alpha.';  Y.Fy(k,:) = d.Fy.';
    Y.beta(k) = atan2(x(1), max(vx(k),0.5));
    if k == n, break; end

    h  = t(k+1) - t(k);
    ip = @(v) v(k) + (v(min(k+1,n)) - v(k))*0.5;      % midpoint of the inputs
    um = [ip(delta); ip(vx); ip(ax)];
    un = [delta(k+1); vx(k+1); ax(k+1)];
    k1 = dualtrack_rhs(x,            uk, M);
    k2 = dualtrack_rhs(x + h/2*k1,   um, M);
    k3 = dualtrack_rhs(x + h/2*k2,   um, M);
    k4 = dualtrack_rhs(x + h*k3,     un, M);
    x  = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
end
