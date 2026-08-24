function export_spike(outdir)
%EXPORT_SPIKE  Export a minimal fixed-step FMU and report what came out.
%
%   Answers, against a real export rather than the specification:
%     * which FMI version this toolchain emits
%     * what platform tuple lands in binaries/  (the loader's lookup key)
%     * whether canGetAndSetFMUState is actually set
%
%   The model deliberately carries STATE (a discrete integrator). A pure
%   feedthrough model would let GetFMUState/SetFMUState round-trip perfectly
%   while proving nothing, and state restore is what the whole deterministic
%   reset design rests on.
%
%   Then check the result with the offline inspector, which is the same set of
%   gates the engine applies:
%       python3 tools/fmu/inspect_fmu.py <the .fmu>

if nargin < 1 || isempty(outdir), outdir = pwd; end
mdl = 'fsds_fmu_spike';

if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(outdir, [mdl '.slx']);
if exist(f, 'file'), delete(f); end

new_system(mdl);
add_block('simulink/Sources/In1',                       [mdl '/u']);
add_block('simulink/Discrete/Discrete-Time Integrator', [mdl '/x']);
add_block('simulink/Sinks/Out1',                        [mdl '/y']);
add_line(mdl, 'u/1', 'x/1');
add_line(mdl, 'x/1', 'y/1');

% 1/960 s: the internal rate docs/fmu_plant_migration.md specifies, being the
% first 60-divisible rate at which the EMRAX current-loop lag exists at all.
set_param(mdl, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
               'FixedStep','1/960', 'StopTime','10');
save_system(mdl, f);
fprintf('model built: fixed-step discrete 1/960 s, one integrator state\n');

e = Simulink.FMUExporter(mdl);
e.SaveDirectory = outdir;
e.FMUName = mdl;

% Every one of these corresponds to a gate the platform enforces at load time.
wanted = { 'FMIVersion',                                  '3.0'
           'FMUType',                                     'CS'
           'canGetAndSetFMUStateOverride',                true
           'canBeInstantiatedOnlyOncePerProcessOverride', false
           'SaveSourceCodeToFMU',                         true
           'CreateModelAfterGeneratingFMU',               false };
for i = 1:size(wanted,1)
    n = wanted{i,1}; v = wanted{i,2};
    try
        e.Options.(n) = v;
    catch
        % Several options take an on/off char rather than a logical.
        try, e.Options.(n) = char(matlab.lang.OnOffSwitchState(v)); catch, end
    end
    fprintf('  option %-46s -> %s\n', n, string(v));
end

fprintf('\nexporting...\n');
e.export();

d = dir(fullfile(outdir, '*.fmu'));
for i = 1:numel(d)
    fprintf('PRODUCED %s (%d bytes)\n', d(i).name, d(i).bytes);
end
if isempty(d)
    error('export() returned but produced no .fmu');
end
end
