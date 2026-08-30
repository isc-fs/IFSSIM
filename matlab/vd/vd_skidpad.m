function S = vd_skidpad(M)
%VD_SKIDPAD  The Formula Student skid pad, as an actual lap time.
%
%   FS skid pad is two circles: 15.25 m inner diameter, 21.25 m outer, so the
%   car's path is a circle of radius (7.625 + 10.625)/2 = 9.125 m. The scored
%   result is the time for one timed lap of that circle, which makes it the
%   one dynamic event that is PURE steady-state cornering -- no braking, no
%   acceleration, no line choice. That is exactly why it is the best possible
%   validation manoeuvre, and why Escofet's fit is highest on it (98.77%).
%
%   Reported here as the lap time the model says the car is capable of, which
%   is a number the team can compare against a stopwatch.

if nargin < 1 || isempty(M), M = dualtrack_build(); end
R = 9.125;

C = vd_constant_radius(R, M, 4:0.5:18);
S = struct('radius',R,'curve',C);
if isempty(C.v)
    S.v_max = NaN; S.lap_time = NaN; S.ay_max = NaN; return;
end
S.v_max    = C.v(end);
S.ay_max   = C.ay(end);
S.lap_time = 2*pi*R / S.v_max;
S.K        = C.K;
S.balance  = ternary(C.K >= 0, 'understeer', 'oversteer');
end

function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
