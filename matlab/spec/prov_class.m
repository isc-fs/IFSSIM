function c = prov_class(src)
%PROV_CLASS  The provenance class of a car_spec source string: its first word.
%
%   One copy, for every table that prints provenance. There were five, and
%   none of them knew INHERITED -- a value a prototype carries over from the
%   car before it -- so the IFS-09's borrowed numbers would have printed as
%   UNKNOWN, or worse, as the IFS-08's MEASURED.
%
%   For an INHERITED source the class is INHERITED, whatever the original was:
%   the original's MEASURED describes the other car.
w = upper(strtok(src));
known = {'MEASURED','MEASURED-ISH','GEOMETRY','DERIVED','DATASHEET', ...
         'SECONDARY','ASSUMED','DISPUTED','ZEROED','UNKNOWN','INHERITED','DESIGN'};
if any(strcmp(w, known)), c = w; else, c = 'UNKNOWN'; end
if strcmp(c,'MEASURED-ISH'), c = 'MEASURED~'; end
end
