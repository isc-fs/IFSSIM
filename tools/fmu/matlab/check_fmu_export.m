function check_fmu_export()
%CHECK_FMU_EXPORT  Can this MATLAB install export an FMU, and if not, why not?
%
%   docs/fmu_plant_migration.md treats MathWorks licensing as the single
%   biggest cost unknown of the FMU migration, and it is not answerable from
%   public sources: what a campus licence covers varies by institution, and
%   "the product exists in toolbox/" is not the same as "it is licensed" which
%   is not the same as "it is installed".
%
%   This separates those three, because they have completely different fixes:
%     not licensed  -> a purchase
%     not installed -> a free re-run of the installer
%     not present   -> wrong release
%
%   Run it with no arguments:  check_fmu_export

fprintf('=== MATLAB %s on %s ===\n\n', version, computer);

%% 1. LICENCE — is the entitlement there at all?
% license('test') asks the licence, not the filesystem, so it answers even for
% products that were never installed.
prods = { 'MATLAB',            'MATLAB'
          'Simulink',          'Simulink'
          'Simulink_Compiler', 'Simulink Compiler'
          'Compiler',          'MATLAB Compiler'
          'Real-Time_Workshop','Simulink Coder'
          'RTW_Embedded_Coder','Embedded Coder'
          'MATLAB_Coder',      'MATLAB Coder' };

fprintf('--- licence entitlement ---\n');
allLicensed = true;
for i = 1:size(prods,1)
    try, ok = license('test', prods{i,1}); catch, ok = 0; end
    fprintf('  %-18s %s\n', prods{i,2}, ternary(ok, 'licensed', 'NOT LICENSED'));
    allLicensed = allLicensed && ok;
end

% A test can pass while every seat is taken, so actually take one.
fprintf('\n--- licence checkout (a test can pass while all seats are busy) ---\n');
for p = {'Simulink_Compiler','Compiler','Real-Time_Workshop'}
    [ok, msg] = license('checkout', p{1});
    fprintf('  %-18s %s %s\n', p{1}, ternary(ok,'OK','FAILED'), msg);
end

%% 2. INSTALLED — is it actually on disk?
fprintf('\n--- installed add-ons ---\n');
try
    a = matlab.addons.installedAddons;
    for i = 1:height(a), fprintf('  %s\n', a.Name(i)); end
catch ME
    fprintf('  could not enumerate: %s\n', ME.message);
end

%% 3. THE EXPORT PATH ITSELF
fprintf('\n--- FMU export entry points ---\n');
fprintf('  exportToFMU function      : %s\n', ternary(exist('exportToFMU','file')~=0, 'present', 'ABSENT'));
fprintf('  Simulink.FMUExporter class: %s\n', ternary(exist('Simulink.FMUExporter','class')~=0, 'present', 'ABSENT'));
fprintf('  exportToFMU_fcn (impl)    : %s\n', ternary(~isempty(which('exportToFMU_fcn')), 'present', 'ABSENT'));

% The code-generation targets ship with base MATLAB; their presence tells you
% the release supports FMI 2/3 even when the export front-end is missing.
d2 = dir(fullfile(matlabroot,'rtw','c','**','RTWCG_FMU2_target.c'));
d3 = dir(fullfile(matlabroot,'rtw','c','**','RTWCG_FMU3_target.c'));
fprintf('  FMI 2 codegen target      : %s\n', ternary(~isempty(d2),'present','ABSENT'));
fprintf('  FMI 3 codegen target      : %s\n', ternary(~isempty(d3),'present','ABSENT'));

%% 4. VERDICT
fprintf('\n=== verdict ===\n');
if ~allLicensed
    fprintf('BLOCKED ON LICENSING. One or more required products is not entitled.\n');
    fprintf('This is the expensive case: it needs a purchase or a different licence.\n');
elseif isempty(which('exportToFMU_fcn'))
    fprintf('LICENCE IS FINE — the export implementation is simply NOT INSTALLED.\n');
    fprintf('This is the cheap case. Re-run the MathWorks installer (or Add-Ons >\n');
    fprintf('Get Add-Ons) with the existing licence and add the Compiler/Coder\n');
    fprintf('products listed as licensed above. No purchase required.\n');
else
    fprintf('READY. Run export_spike.m to produce a real .fmu and inspect it.\n');
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
