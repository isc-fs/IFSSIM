function C = inherit(Parent, name)
%INHERIT  Start a new prototype as a copy of an existing one, honestly labelled.
%
%   C = INHERIT(ifs08(), 'IFS-09')
%
%   Every value is carried over, and every SOURCE is rewritten to say so:
%       INHERITED from IFS-08 -- MEASURED corner weights ...
%   The value is a starting point, not a fact about the new car. Nobody has
%   weighed the IFS-09 just because somebody weighed the IFS-08, and a
%   report that showed the IFS-08's "MEASURED" against an IFS-09 number would
%   be claiming exactly that. INHERITED is its own provenance class, counted
%   by check_car, so the new car's spec says how much of it is still borrowed.
%
%   Override with par(C, ...) and a source of the new car's own; the
%   INHERITED label goes with the old value.
C = Parent;
C.Name = name;
C.Parent = Parent.Name;
C.Simulator = false;
for i = 1:numel(C.Order)
    k = C.Order{i};
    C.Fields.(k).source = sprintf('INHERITED from %s -- %s', Parent.Name, C.Fields.(k).source);
end
end
