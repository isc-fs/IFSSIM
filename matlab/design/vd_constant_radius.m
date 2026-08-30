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
           'alphaF',[],'alphaR',[],'K',NaN,'ackermann',M.L/R,'v_max',NaN);

for v = speeds
    [okv, d, S] = hold_circle(v, R, M);
    if ~okv, break; end
    C.v(end+1)      = v;        C.delta(end+1)  = d;
    C.ay(end+1)     = S.ay;     C.beta(end+1)   = S.beta;
    C.alphaF(end+1) = S.alphaF; C.alphaR(end+1) = S.alphaR;
end

% Refine the limit speed with a deterministic fine scan through the SAME test
% the sweep uses, rather than bisecting on whether a solver converged. The
% limit is physical -- full lock stops producing enough yaw rate to hold the
% circle -- and testing it directly gives the same answer every run. Bisecting
% on Newton convergence does not: it made the limit jitter between neighbouring
% setups, so the skid-pad time moved non-monotonically while the understeer
% gradient moved smoothly, which is the signature of a numerical artefact
% rather than a property of the car.
if ~isempty(C.v) && numel(speeds) > 1
    step = speeds(2) - speeds(1);
    for v = C.v(end) + (step/20 : step/20 : step)
        if ~hold_circle(v, R, M), break; end
        C.v(end) = v;
    end
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
if f(M.maxSteer) < 0, return; end          % not enough lock, whatever else
try
    d = fzero(f, [0 M.maxSteer], optimset('TolX',1e-6,'Display','off'));
catch
    return;
end
S  = dualtrack_trim(v, d, M);
ok = S.settled;
end

function r = trimr(d, v, M)
S = dualtrack_trim(v, d, M);
r = S.r;
end
