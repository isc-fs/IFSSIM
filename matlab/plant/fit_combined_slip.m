function [Bxa, Byk, D] = fit_combined_slip(P)
%FIT_COMBINED_SLIP  Our own combined-slip coefficients, from our own curves.
%
%   Magic Formula does not get combined slip from the pure-slip fit. It takes
%   a separate family of R-coefficients that say how fast longitudinal force
%   collapses with slip ANGLE and lateral force with slip RATIO. Nobody has
%   run an IFS-08 tyre on a combined-slip rig, so we have no fit for them.
%
%   The tempting answer -- zero them, the way every other unmeasured effect
%   here is zeroed -- is wrong, and measurably so. With RBX1 = RBY1 = 0 the
%   weighting functions become identically 1, Fx and Fy stop interacting, and
%   the resultant force reaches 1.4142*mu*Fz: the tyre can pull a full mu
%   sideways WHILE pulling a full mu forwards. There is no friction circle
%   left. Zeroing is not "no assumption" here, it is the assumption that
%   grip is unlimited, which is worse than any tyre.
%
%   So instead of inheriting a road radial's coupling (which over-couples
%   ours -- it pinches the envelope to 0.93*mu*Fz at 45 degrees) we derive
%   the two coefficients from the pure-slip curves we do have, by asking for
%   the one property we are confident about: the force envelope should be a
%   circle of radius mu*Fz. That is an ASSUMPTION -- real tyres are slightly
%   elliptical -- but it is OUR assumption, stated, and it comes out of the
%   same numbers as the rest of the tyre.
%
%   Everything else in the R-family stays zero: the shape factors RCX1/RCY1
%   are 1 (a C of 0 would flatten the weighting back to no coupling at all),
%   and the load, camber and kappa-induced-Fy terms are effects we cannot
%   measure.

if nargin < 1 || isempty(P), P = ifssim_params(); end

Fz = P.Derived.NominalWheelLoad;   mu = P.TireMu;
Cy = P.Pacejka.LatC;  Ey = P.Pacejka.LatE;  Ky = P.Derived.CorneringStiffness;
Cx = P.Pacejka.LonC;  Ex = P.Pacejka.LonE;  Kx = P.Derived.LongSlipStiffness;
By = Ky/(Cy*mu*Fz);   Bx = Kx/(Cx*mu*Fz);

mf  = @(B,C,E,x) sin(C.*atan(B.*x - E.*(B.*x - atan(B.*x))));
Fy0 = @(a) mu*Fz*mf(By,Cy,Ey,a);
Fx0 = @(k) mu*Fz*mf(Bx,Cx,Ex,k);

% The slip space the car actually visits, sampled densely enough that the
% directional maxima below are the curve's and not the grid's.
k = linspace(0, 0.60, 241);
a = linspace(0, 35*pi/180, 241);
[K,A] = ndgrid(k,a);
th = linspace(0, pi/2, 73);          % force directions, 1.25 deg apart

cost = @(p) local_cost(p,K,A,Fx0,Fy0,mu,Fz,th);
p    = fminsearch(cost, [10 10], optimset('Display','off','TolX',1e-4,'TolFun',1e-9));
Bxa  = abs(p(1));  Byk = abs(p(2));

[~,E] = local_cost([Bxa Byk],K,A,Fx0,Fy0,mu,Fz,th);
D = struct('EnvelopeMin',min(E),'EnvelopeMax',max(E), ...
           'EnvelopeRms',sqrt(mean((E-1).^2)));
end

function [c,E] = local_cost(p,K,A,Fx0,Fy0,mu,Fz,th)
% Envelope of the combined force in each direction, as a fraction of mu*Fz.
% A friction circle is E == 1 everywhere; the cost is how far off it is.
Bxa = abs(p(1)); Byk = abs(p(2));
FX  = Fx0(K).*cos(atan(Bxa*A));      % Gxa with RCX1 = 1, REX* = 0, RHX1 = 0
FY  = Fy0(A).*cos(atan(Byk*K));      % Gyk with RCY1 = 1, REY* = 0, RHY* = 0
R   = hypot(FX,FY);  TH = atan2(FY,FX);
E   = nan(size(th));
half = (th(2)-th(1))/2;
for i = 1:numel(th)
    w = abs(TH - th(i)) < half;
    if any(w(:)), E(i) = max(R(w))/(mu*Fz); end
end
E = E(~isnan(E));
c = sum((E-1).^2);
end
