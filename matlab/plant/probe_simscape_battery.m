function ok = probe_simscape_battery()
%PROBE_SIMSCAPE_BATTERY  Can a Simscape battery live inside our FMU? Yes.
%
%   Run this before building anything on Simscape. It puts the smallest
%   possible Simscape Battery circuit through the exact pipeline the plant
%   uses -- fixed-step ode1 at the plant's step, then FMI 3.0 co-simulation
%   export -- and reports whether each stage works.
%
%   It exists because six things about this are not obvious, every one of
%   them cost time, and none of them are in any error message that tells you
%   what to do instead:
%
%   1. LICENCE NAMES LIE. license('test','Simulink_Coder') returns 0 on a
%      machine that generates code perfectly well. The feature is called
%      'Real-Time_Workshop'. Checking the wrong name concludes the whole
%      approach is impossible.
%
%   2. SIMSCAPE NEEDS A LOCAL SOLVER. By default it wants its own variable-
%      step DAE solver, which a fixed-step FMU cannot carry. Set
%      UseLocalSolver on the Solver Configuration, backward Euler, at the
%      SAME step as the model.
%
%   3. THE SOLVER CONFIGURATION'S PORT IS RConn, NOT LConn. It has no left
%      port at all. Connecting from LConn(1) throws an index error, and the
%      real symptom arrives later as "no Solver Configuration block connected
%      to Physical Network".
%
%   4. ELECTRICAL REFERENCE IS NOT A GLOBAL GROUND. Two Electrical Reference
%      blocks are two separate nodes unless something wires them together.
%      Grounding the solver on its own Reference block puts it on an island,
%      and the error names every block EXCEPT the one that is missing.
%
%   5. BRANCHING WORKS, and is how the solver attaches. add_line to a
%      physical port that already has a line creates a branch -- it does not
%      fail. Most of the trouble above came from assuming it would.
%
%   6. THE EXPORTER TAKES OPTIONS, NOT PROPERTIES. e.Options.(name) = value.
%      Setting e.FMUName directly is accepted and silently ignored.
%
%   Also worth knowing for what comes next: the Battery (Table-Based) block
%   can expose SOC_port and thermal_port, which is the surface any future
%   thermal model, ageing model or BMS hangs off. Enabling SOC_port adds the
%   port on the LEFT, so the block becomes LConn = [minus, SoC].

STEP = '0.0010416666666666671';   % the plant's step; see build_plant_skeleton
nm   = 'probe_simscape_battery_model';
ok   = true;

if bdIsLoaded(nm), close_system(nm,0); end
new_system(nm);
set_param(nm,'SolverType','Fixed-step','Solver','ode1','FixedStep',STEP, ...
             'StartTime','0','StopTime','2');

SOLVER = sprintf('nesl_utility/Solver\nConfiguration');
PS2S   = sprintf('nesl_utility/PS-Simulink\nConverter');

add_block('batt_lib/Cells/Battery (Table-Based)',[nm '/BATT'],'Position',[320 100 400 200]);
set_param([nm '/BATT'],'SOC_port','simscape.enum.tablebattery.enable.yes');
add_block('fl_lib/Electrical/Electrical Elements/Resistor',[nm '/R'], ...
          'Position',[500 100 560 160],'R','4');
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference',[nm '/GND'], ...
          'Position',[500 300 540 340]);
add_block(SOLVER,[nm '/SC'],  'Position',[160 300 220 340]);
add_block(PS2S,  [nm '/P2S'], 'Position',[200 200 240 240]);
add_block('simulink/Sinks/Out1',[nm '/SOC'],'Position',[300 210 330 230]);

P = @(b) get_param([nm '/' b],'PortHandles');
add_line(nm, P('BATT').RConn(1), P('R').LConn(1));      % + to the load
add_line(nm, P('R').RConn(1),    P('GND').LConn(1));    % load to ground
add_line(nm, P('BATT').LConn(1), P('GND').LConn(1));    % - to ground   (branch)
add_line(nm, P('SC').RConn(1),   P('GND').LConn(1));    % solver on the network (branch)
add_line(nm, P('BATT').LConn(2), P('P2S').LConn(1));    % SoC out
add_line(nm,'P2S/1','SOC/1');

set_param([nm '/SC'],'UseLocalSolver','on', ...
          'LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER', ...
          'LocalSolverSampleTime',STEP,'DoFixedCost','on');

fprintf('\n=== Simscape battery, through our pipeline ===\n');
fprintf('  %-34s %s\n','Simscape licence',        tf(license('test','Simscape')));
fprintf('  %-34s %s\n','Simscape Battery licence',tf(license('test','Simscape_Battery')));
fprintf('  %-34s %s\n','code generation',         tf(license('test','Real-Time_Workshop')));

try
    sim(nm);
    fprintf('  %-34s ok\n','simulates at fixed-step ode1');
catch e
    ok = false;
    fprintf('  %-34s FAILED: %s\n','simulates at fixed-step ode1', regexprep(e.message,'\s+',' '));
end

d = fullfile(tempdir,'ifssim_batt_probe');
if ~isfolder(d), mkdir(d); end
save_system(nm, fullfile(d,[nm '.slx']));
try
    old = cd(d); cl = onCleanup(@() cd(old)); %#ok<NASGU>
    e2  = Simulink.FMUExporter(nm);
    o = {'SaveDirectory',d;'FMUName',nm;'FMIVersion','3.0';'FMUType','CS'; ...
         'CreateModelAfterGeneratingFMU','off'};
    for i = 1:size(o,1)
        try, e2.Options.(o{i,1}) = o{i,2}; catch, end %#ok<CTCH>
    end
    if isfile(fullfile(d,[nm '.fmu'])), delete(fullfile(d,[nm '.fmu'])); end
    e2.export();
    f = dir(fullfile(d,[nm '.fmu']));
    if isempty(f)
        ok = false; fprintf('  %-34s produced nothing\n','FMI 3.0 co-simulation export');
    else
        fprintf('  %-34s ok, %.0f kB\n','FMI 3.0 co-simulation export', f.bytes/1024);
    end
catch e
    ok = false;
    fprintf('  %-34s FAILED: %s\n','FMI 3.0 co-simulation export', regexprep(e.message,'\s+',' '));
end

fprintf('\n  %s\n', ternary(ok, ...
    'VIABLE. Simscape can carry the battery without breaking the FMU.', ...
    'NOT VIABLE as configured -- read the failures above before building on it.'));
close_system(nm,0);
end

function s = tf(v),        if v, s='yes'; else, s='NO'; end, end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
