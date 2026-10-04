function ok = test_cars()
%TEST_CARS  The multi-prototype structure does what it promises.
%
%   1. IFS-08 IS UNCHANGED by the move to spec/cars: ifssim_params for it is
%      the parameter set settings.json was verified against.
%   2. INHERITANCE IS EXACT AND HONEST: a child car starts with every value of
%      its parent, identical, and every source marked INHERITED.
%   3. AN OVERRIDE REPLACES IN PLACE: no parameter appears twice.
%   4. THE ACTIVE CAR SWITCHES EVERYTHING: ifssim_params follows it, and its
%      models build somewhere other than the simulator car's.
%   5. EXACTLY ONE CAR IS THE SIMULATOR'S, and it is the IFS-08.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'));
ok = true;
fprintf('\n=== prototypes ===\n');
was = ifssim_car();
restore = onCleanup(@() ifssim_car(was));

% ---- 1 ------------------------------------------------------------------
P8 = ifssim_params([], 'IFS-08');
S  = jsondecode(fileread(fullfile(here,'..','..','settings.json')));
vn = fieldnames(S.Vehicles);  VP = S.Vehicles.(vn{1}).VehiclePhysics;
names = {'Mass','Wheelbase','CoGHeight','TireMu','GearRatio','CdA','ClA','WheelRadius'};
same = all(cellfun(@(n) abs(P8.(n) - VP.(n)) < 1e-9*max(1,abs(VP.(n))), names));
ok = check(ok, 'IFS-08 matches settings.json (8 headline values)', same);

% ---- 2 ------------------------------------------------------------------
A = car_spec('IFS-08');  B = car_spec('IFS-09');
ok = check(ok, 'IFS-09 names its parent', strcmp(B.Parent, 'IFS-08'));
keysA = A.Order;  inherited = 0;  identical = true;
for i = 1:numel(keysA)
    k = keysA{i};
    if strcmp(prov_class(B.Fields.(k).source), 'INHERITED')
        inherited = inherited + 1;
        identical = identical && isequal(A.Fields.(k).value, B.Fields.(k).value) && ...
                    contains(B.Fields.(k).source, A.Fields.(k).source);
    end
end
ok = check(ok, 'every inherited value identical, original source kept', identical);
fprintf('        IFS-09: %d of %d inherited\n', inherited, numel(B.Order));

% ---- 3 ------------------------------------------------------------------
ok = check(ok, 'no parameter listed twice', numel(unique(B.Order)) == numel(B.Order));
H = car_helpers_selftest();
ok = check(ok, 'inherit labels every source INHERITED from the parent', H.inheritedLabelled);
ok = check(ok, 'an inherited car is not the simulator''s', H.inheritedNotSim);
ok = check(ok, 'override replaces in place, with its own source', H.overrideInPlace);
ok = check(ok, 'an override leaves the rest inherited', H.otherUntouched);
ok = check(ok, 'a parameter without a source is refused', H.sourceRequired);

% ---- 4 ------------------------------------------------------------------
ifssim_car('IFS-09');
P9 = ifssim_params();
ok = check(ok, 'ifssim_params follows the active car', strcmp(P9.SpecName, 'IFS-09'));
d8 = ifssim_models_dir('IFS-08');  d9 = ifssim_models_dir('IFS-09');
ok = check(ok, 'IFS-09 builds outside the IFS-08''s models', ~strcmp(d8, d9) && contains(d9, fullfile('build','cars','IFS-09')));
ifssim_load_workspace();
p = [pathsep path pathsep];
ok = check(ok, 'only the active car''s models on the path', ...
           contains(p, [pathsep d9 pathsep]) && ~contains(p, [pathsep d8 pathsep]));
ifssim_car('IFS-08');  ifssim_load_workspace();

% ---- 5 ------------------------------------------------------------------
ok = check(ok, 'the simulator''s car is the IFS-08', strcmp(simulator_car(), 'IFS-08'));

fprintf('\n%s\n', tern(ok, 'prototypes PASS.', 'PROTOTYPES FAILED.'));
end

function ok = check(ok, name, pass)
fprintf('  [%s] %s\n', tern(pass,'ok  ','FAIL'), name);
if ~pass, ok = false; end
end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
