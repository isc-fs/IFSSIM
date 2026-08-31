function s = ifssim_step(useVDB)
%IFSSIM_STEP  The plant's fixed step, as the exact string every model must use.
%
%   '0.0010416666666666671' -- 1/960 plus 2 ulp, deliberately, and NOT the
%   double nearest 1/960 (which is ...667). The full reasoning is the long
%   note on STEP in build_plant_skeleton.m; the short version is that once a
%   referenced model carries continuous states, Simulink requires parent and
%   child to agree on step size to the bit, and it compares the NEGOTIATED
%   fundamental sample times rather than these strings.
%
%   This function exists because that literal was written out by hand in some
%   builders and as '1/960' in others, and the two are different doubles. The
%   VDB chassis was written with '1/960' and broke four of the ten stages:
%   the plant negotiated ...667 from it while IFSSIM_TireSuspension held
%   ...671, and the build failed with the two values printed side by side.
%   One source of truth is cheaper than finding that out twice.
%   IFSSIM_STEP(TRUE) returns '1/960' instead, and the reason is empirical
%   rather than principled. What Simulink matches is the NEGOTIATED
%   fundamental sample time, which each model arrives at from its own
%   contents -- so it moves when the contents change. With our discrete
%   chassis the plant negotiated ...671 and the literal above made everything
%   agree. With the VDB chassis, which is continuous, the plant negotiates
%   ...667 instead, and the same literal now makes both referenced models
%   disagree with it.
%
%   This is a pair of values chosen to make a negotiation converge, not a
%   physical step size; the two differ by 2 ulp, 8e-16 relative, under a
%   nanosecond per step. It is fragile by nature: anything that changes what
%   a model contains can move the negotiated rate again, and the failure is
%   loud -- the build stops with both values printed side by side.
if nargin < 1 || isempty(useVDB), useVDB = false; end
if useVDB
    s = '1/960';
else
    s = '0.0010416666666666671';
end
end
