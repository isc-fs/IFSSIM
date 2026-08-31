function [camberRad, camberHslp] = vdb_camber_mask(staticCamberDeg, gain, track, F0z, Kz)
%VDB_CAMBER_MASK  Camber dialog values for the VDB double-wishbone block.
%
%   [CAM, HSLP] = VDB_CAMBER_MASK(STATICDEG, GAIN, TRACK, F0Z, KZ) returns the
%   Camber and CamberHslp an axle needs so the block's WhlAng reproduces the
%   inclination that axle's tyres actually see against the road.
%
%   The block's map was measured, not read off the documentation, by driving
%   it one wheel at a time (see docs/vdb_step1_port_semantics.md):
%
%       gamma_i = sigma_i * [ -Camber + CamberHslp * (F0z/Kz - WhlPz_i) ]
%       sigma   = [+1 -1 +1 -1]
%
%   Three consequences drive everything below.
%
%   1. SIGMA MIRRORS LEFT TO RIGHT. That is the ISO inclination convention the
%      Magic Formula expects: symmetric negative camber is a NEGATIVE gamma on
%      one side and a POSITIVE gamma on the other, because the tyre axis system
%      keeps y to the left on both wheels. A uniform static camber on all four
%      wheels is the wrong sign on one side of the car.
%
%   2. THE DATUM IS PER AXLE. The block measures travel from its own static
%      deflection F0z/Kz, which is 0.0292 m at the front and 0.0376 m at the
%      rear for this car -- 1.68 deg and 2.15 deg of camber offset at unit
%      slope. A single shared constant is wrong at one end by the difference.
%
%   3. BODY ROLL ARRIVES THROUGH THE TRAVEL, not as a separate term. A roll phi
%      puts -(t/2)*phi into WhlPz, so choosing the slope to leave (1-GAIN)*phi
%      standing after the geometry recovers GAIN*phi makes the block's output
%      the road-relative inclination directly. Heave then also produces camber,
%      which the closed form it replaces did not model.

if nargin < 4, error('vdb_camber_mask:datum', ...
    'F0z and Kz are required: the block''s travel datum is per axle.'); end

staticDeflection = F0z / Kz;                 % m, the block's own zero
camberHslp = -2*(1 - gain)/track;            % rad per m of compression
camberRad  = staticCamberDeg*pi/180 + camberHslp*staticDeflection;
end
