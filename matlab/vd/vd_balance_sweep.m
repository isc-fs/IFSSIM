function B = vd_balance_sweep(M, fracs)
%VD_BALANCE_SWEEP  What roll stiffness distribution does to the balance.
%
%   The one setup lever this model can actually turn, swept. Total roll
%   stiffness is held constant and only the FRONT SHARE moves, which is what
%   changing an anti-roll bar does on the car.
%
%   Understeer gradient K is reported in degrees of extra steering per g.
%   K > 0 understeer, K < 0 oversteer. More front roll stiffness moves more of
%   the lateral load transfer onto the front axle, which loses more grip to
%   load sensitivity, so the front gives up first: K rises.
%
%   This only works because two things landed together. Load transfer has to
%   exist (it did not until the moment arm was fixed) and grip has to depend
%   on load (PDY2, which was zero). With either missing this sweep is a flat
%   line, and the car has no balance to set.

if nargin < 1 || isempty(M), M = dualtrack_build(); end
if nargin < 2 || isempty(fracs), fracs = 0.40:0.05:0.70; end

Ktot = M.KrF + M.KrR;
B = struct('frac',[],'K',[],'ay_max',[],'skidpad',[]);
for f = fracs
    Mf = M;  Mf.KrF = f*Ktot;  Mf.KrR = (1-f)*Ktot;
    C  = vd_constant_radius(9.125, Mf, 4:0.5:18);
    B.frac(end+1)   = f;
    B.K(end+1)      = C.K;
    if isempty(C.ay)
        B.ay_max(end+1) = NaN;  B.skidpad(end+1) = NaN;
    else
        B.ay_max(end+1)  = C.ay(end);
        B.skidpad(end+1) = 2*pi*9.125/C.v(end);
    end
end
end
