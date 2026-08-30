function S = vd_step_steer(v, delta, M)
%VD_STEP_STEER  Transient response: how fast the car answers the wheel.
%
%   The constant-radius test says where the car ends up; this says how long it
%   takes and whether it overshoots on the way. Both matter, and only the
%   second one is affected by yaw inertia -- which for us is P.Assumed.Izz, a
%   number nobody has measured, so this test is also the sensitivity of the
%   car's feel to that assumption.
%
%   Reports the standard three: response time to 90% of the settled yaw rate,
%   peak overshoot, and the settled value.

if nargin < 1 || isempty(v),     v = 12;            end
if nargin < 2 || isempty(delta), delta = 4*pi/180;  end
if nargin < 3 || isempty(M),     M = dualtrack_build(); end

t  = (0:0.001:3)';
d  = delta * (t >= 0.5);                    % step at 0.5 s, after a settled start
Y  = dualtrack_sim(t, d, v, M);

rss = mean(Y.r(t > 2.5));
S   = struct('v',v,'delta',delta,'r_ss',rss,'t90',NaN,'overshoot',NaN, ...
             'beta_ss',mean(Y.beta(t > 2.5)),'ay_ss',mean(Y.ay(t > 2.5)));
if abs(rss) > 1e-6
    k = find(t >= 0.5 & abs(Y.r) >= 0.9*abs(rss), 1);
    if ~isempty(k), S.t90 = t(k) - 0.5; end
    S.overshoot = 100*(max(abs(Y.r(t>=0.5))) - abs(rss))/abs(rss);
end
S.t = t; S.r = Y.r;
end
