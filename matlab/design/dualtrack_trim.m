function [S, ok, x] = dualtrack_trim(v, delta, M, x0)
%DUALTRACK_TRIM  Steady-state cornering at a given speed and steer angle.
%
%   The operating point every classical vehicle-dynamics test is built from:
%   hold the speed and the steering, and report where the car settles.
%
%   SOLVED, not simulated. Steady state means xdot = 0, which is two equations
%   in two unknowns (vy, r) -- so a few Newton steps find it exactly, where
%   integrating to convergence needs several thousand RHS evaluations and only
%   ever gets close. That matters because this sits inside an fzero, inside a
%   speed sweep, inside a setup sweep: the report went from ten minutes to
%   seconds on this change alone, and a tool nobody will wait for is a tool
%   nobody uses.
%
%   ok is false when there is no steady state to find, which is not a
%   numerical failure -- it is the car spinning. Past the limit no steady
%   state exists, and a routine that returned a number anyway would be
%   inventing one.

    function F = res(x)
        F = dualtrack_rhs(x, [delta; v; 0], M);
    end

% Seeded from the previous solution when the caller has one. Near the limit
% the basin of attraction is narrow and a kinematic guess falls out of it,
% which shows up as a limit speed that jitters between neighbouring setups
% instead of moving smoothly. Continuation from the last solved point is what
% makes a swept limit reproducible.
if nargin >= 4 && ~isempty(x0) && all(isfinite(x0))
    x = x0(:);
else
    x = [0; v*tan(delta)/M.L];      % kinematic: geometry's yaw rate, no sideslip
end
ok = false;
for it = 1:40
    F = res(x);
    if norm(F) < 1e-10, ok = true; break; end
    % Numerical Jacobian. Two extra evaluations; the states are O(1) and O(10)
    % so the step is scaled per-state rather than absolute.
    J = zeros(2,2);
    for j = 1:2
        dx = max(1e-7, 1e-7*abs(x(j)));
        xp = x;  xp(j) = xp(j) + dx;
        J(:,j) = (res(xp) - F)/dx;
    end
    if rcond(J) < 1e-14, break; end
    step = -J\F;
    % Damped: undamped Newton on this system will happily jump onto the spun
    % branch from a good starting point.
    lam = 1;
    for b = 1:20
        xn = x + lam*step;
        if norm(res(xn)) < norm(F), break; end
        lam = lam/2;
    end
    x = x + lam*step;
end

vy = x(1);  r = x(2);
[~, d] = dualtrack_rhs(x, [delta; v; 0], M);
beta = atan2(vy, max(v,0.5));

% A settled car has a sideslip a driver could hold. Newton can land on a
% mathematically valid equilibrium that is a spin.
ok = ok && abs(beta) < 25*pi/180 && all(isfinite([vy r]));

S = struct('v',v,'delta',delta,'r',r,'vy',vy,'ay',d.ay,'beta',beta, ...
           'Fz',d.Fz.','alphaF',mean(d.alpha(1:2)),'alphaR',mean(d.alpha(3:4)), ...
           'settled',ok);
end
