function [Rmin, ackermann, loss] = vd_min_radius(M)
%VD_MIN_RADIUS  The tightest circle the car can hold, and what lock costs.
%
%   Steering lock is a DESIGN CONSTRAINT and a hardware decision -- rack travel
%   and upright stops -- and it is the one part of this model that does not
%   depend on the tyre fit. The geometry half is trustworthy today, unlike
%   anything resting on a Magic Formula nobody has validated.
%
%   Returns the tightest radius the car can actually hold at low speed, the
%   pure-Ackermann radius geometry alone would give, and the fraction of
%   cornering capability lost at that radius to running out of lock rather
%   than out of grip.
%
%   FS matters: the tightest corners on an autocross or a skid pad entry are
%   where a lock-limited car loses time it cannot get back with setup.

if nargin < 1 || isempty(M)
    here = fileparts(mfilename('fullpath'));
    addpath(here); addpath(fullfile(here,'..','plant'));
    M = dualtrack_build();
end

ackermann = M.L / tan(M.maxSteer);

% The real one: the smallest radius that still has a steady state at walking
% pace, where grip is not the binding constraint.
v = 3.0;
Rmin = ackermann;
for R = ackermann : 0.05 : 4*ackermann
    S = dualtrack_trim(v, M.maxSteer, M);
    if S.settled && S.r >= v/R - 1e-6
        Rmin = R; break;
    end
end

% What lock costs. Compared at a tight radius against the car's best, because
% "loss at Rmin" is not the useful number -- at the very tightest radius the
% car is grip-limited again by the time it gets there.
loss = NaN;
try
    Ct = vd_constant_radius(max(Rmin*1.05, 5.0), M, 4:0.5:16);
    Cb = vd_constant_radius(12.0, M, 4:0.5:16);
    if ~isempty(Ct.ay) && ~isempty(Cb.ay)
        loss = 1 - max(Ct.ay)/max(Cb.ay);
    end
catch
end
end

function e = lockAy(v, R, M)
S = dualtrack_trim(v, M.maxSteer, M);
e = S.r - v/R;
end
