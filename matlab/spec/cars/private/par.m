function C = par(C, name, value, unit, source)
%PAR  One parameter, with the source it came from. Source is mandatory.
%
%   Shared by every car in spec/cars. Setting a parameter that already exists
%   REPLACES it in place -- that is how a prototype that inherits from another
%   says what is different about it -- without adding it to the order twice.
if isempty(strtrim(source))
    error('car_spec:noSource','%s has no source. Write UNKNOWN if that is the truth.', name);
end
key = strrep(name,'.','_');
C.Fields.(key) = struct('name',name,'value',value,'unit',unit,'source',source);
if ~any(strcmp(C.Order, key))
    C.Order{end+1} = key;
end
end
