function R = car_helpers_selftest()
%CAR_HELPERS_SELFTEST  Exercise par and inherit, which are private to spec/cars.
%
%   Lives here because private helpers are visible only to files in this
%   folder, and that privacy is the point: a car is written with par, not
%   assembled by anything else. test_cars calls this.
P = struct('Name','PARENT','Parent','','Simulator',true,'Fields',struct(),'Order',{{}});
P = par(P, 'Mass', 275, 'kg', 'MEASURED scales');
P = par(P, 'Pack.CurrentLimit', 200, 'A', 'MEASURED logged');
C = inherit(P, 'CHILD');
R.inheritedLabelled = startsWith(C.Fields.Mass.source, 'INHERITED from PARENT -- MEASURED');
R.inheritedNotSim   = ~C.Simulator && strcmp(C.Parent, 'PARENT');
n = numel(C.Order);
C = par(C, 'Mass', 290, 'kg', 'DESIGN target');
R.overrideInPlace   = numel(C.Order) == n && C.Fields.Mass.value == 290 && ...
                      startsWith(C.Fields.Mass.source, 'DESIGN');
R.otherUntouched    = startsWith(C.Fields.Pack_CurrentLimit.source, 'INHERITED');
try
    par(C, 'X', 1, '-', '  ');
    R.sourceRequired = false;
catch
    R.sourceRequired = true;
end
end
