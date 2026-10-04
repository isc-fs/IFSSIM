function T = import_susp_geometry(xlsx, out)
%IMPORT_SUSP_GEOMETRY  The IFS-08 hardpoints, from the team workbook into the repo.
%
%   T = IMPORT_SUSP_GEOMETRY()   reads matlab/archive/MODEL_IFS_08/ISC_IFS_08.xlsx,
%                                sheet Susp_Geometry, and writes
%                                matlab/spec/cars/ifs08_hardpoints.csv
%
%   The workbook lives in an untracked archive; the CSV is committed, so the
%   geometry -- and where every number in it came from -- travels with the code.
%   Run this again only if the workbook changes.
%
%   THE COORDINATES ARE WRITTEN EXACTLY AS THE SHEET HAS THEM, in mm, with the
%   sheet row of every point. Converting them to the car's frame happens in
%   sm_hardpoints, in one place, because the sheet's frame needs care:
%
%     - its header says X is "positive forward", but its values grow REARWARD:
%       the front wheel centre is at x = 160 and the rear at x = 1730, and the
%       sheet's own caster (+6.2 deg, upper ball joint at LARGER x) is only a
%       positive caster if larger x is further back;
%     - its Z = 0 is NOT the ground, if the sheet's own tyre radius is right:
%       wheel centres at z = 310 with a 203.2 mm tyre put the ground at
%       z = 106.8. The sheet's roll-centre and scrub analysis assumes Z = 0
%       is the ground. See sm_hardpoints and test_susp_geometry.

here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(xlsx), xlsx = fullfile(here,'..','archive','MODEL_IFS_08','ISC_IFS_08.xlsx'); end
if nargin < 2 || isempty(out),  out  = fullfile(here,'..','spec','cars','ifs08_hardpoints.csv'); end
if ~isfile(xlsx)
    error('import_susp_geometry:missing', ['%s is not here. The CSV is committed; you only need ' ...
          'the workbook to regenerate it.'], xlsx);
end
c = readcell(xlsx, 'Sheet', 'Susp_Geometry');

% The two hardpoint tables: an "ID | Point Name | X | Y | Z | MONO ref" header
% row, then rows whose ID is F<n> or R<n>. Found by content, not by row number,
% so an inserted row in the workbook does not shift every point by one.
rows = {};
for r = 1:size(c,1)
    id = c{r,1};
    if (ischar(id) || isstring(id)) && ~isempty(regexp(char(id), '^[FR]\d+$', 'once'))
        xyz = [c{r,3}, c{r,4}, c{r,5}];
        if ~isnumeric(xyz) || numel(xyz) ~= 3 || any(isnan(xyz))
            error('import_susp_geometry:bad', 'row %d (%s) has no numeric X/Y/Z', r, id);
        end
        rows(end+1,:) = {char(id), char(string(c{r,2})), xyz(1), xyz(2), xyz(3), ...
                         sprintf('Susp_Geometry row %d', r)}; %#ok<AGROW>
    end
end
need = [arrayfun(@(k) sprintf('F%d',k), 1:16, 'uni', 0), arrayfun(@(k) sprintf('R%d',k), 1:16, 'uni', 0)];
missing = setdiff(need, rows(:,1));
if ~isempty(missing)
    error('import_susp_geometry:incomplete', 'missing hardpoints: %s', strjoin(missing, ', '));
end

% The sheet's own vehicle parameters that the frame conversion needs.
g = struct();
for r = 1:size(c,1)
    k = c{r,1};
    if ~(ischar(k) || isstring(k)), continue; end
    % FIRST occurrence only: 'Wheelbase' appears again at the spring-rate
    % section, as a different quantity, and the last one used to win.
    switch strtrim(char(k))
        case 'Tire Radius',  if ~isfield(g,'TireRadius_mm'), g.TireRadius_mm = c{r,2}; g.TireRadius_row = r; end
        case 'Ride Height',  if ~isfield(g,'RideHeight_mm'), g.RideHeight_mm = c{r,2}; g.RideHeight_row = r; end
        case 'Wheelbase',    if ~isfield(g,'Wheelbase_mm'),  g.Wheelbase_mm  = c{r,2}; g.Wheelbase_row  = r; end
    end
end

T = cell2table(rows, 'VariableNames', {'id','name','x_mm','y_mm','z_mm','source'});
writetable(T, out);
fprintf('wrote %d hardpoints to %s\n', height(T), out);
fprintf('  sheet tyre radius %.1f mm (row %d), ride height %.0f mm (row %d), wheelbase %.0f mm (row %d)\n', ...
        g.TireRadius_mm, g.TireRadius_row, g.RideHeight_mm, g.RideHeight_row, g.Wheelbase_mm, g.Wheelbase_row);
T.Properties.UserData = g;
end
