function build_plant_skeleton(outdir)
%BUILD_PLANT_SKELETON  Generate the IFSSIM plant model skeleton.
%
%   Output goes to matlab/plant/models — NOT 'generated'. The skeleton is
%   generated; the models are not disposable. Once an engineer fills in a
%   subsystem that file is the work, and a folder named 'generated' invites
%   someone to delete it.
%
%   Top-level model plus one REFERENCED MODEL per subsystem, so engineers own
%   separate .slx files and can work in parallel — .slx does not merge.
%
%   Generated from this script because a binary .slx cannot be reviewed in a
%   diff. Subsystem contents are owned by engineers; only the skeleton is
%   regenerated.
%
%   Every subsystem starts as a correctly-ported placeholder emitting zeros, so
%   the whole model compiles from day one and nobody is blocked.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
if ~isfolder(outdir), mkdir(outdir); end
addpath(fileparts(mfilename('fullpath')));
% The generated directory must be on the path BEFORE the top model is created,
% or every Model block resolves to "model not found".
addpath(outdir);

% Parameters and buses must exist in the base workspace before anything is
% typed against them.
P = ifssim_load_workspace();

fprintf('building plant skeleton in %s\n', outdir);
fprintf('  parameters from %s (%s)\n', P.SettingsPath, P.VehicleName);

% Fixed step chosen to match the FMU internal rate the platform requires: the
% first 60-divisible rate at which the EMRAX current-loop time constant is
% actually representable. See docs/fmu_plant_migration.md.
STEP = '1/960';

%% ---- referenced subsystem models -------------------------------------
% owner: which engineer this belongs to. It is written into the model as an
% annotation so the question "whose block is this?" has an answer inside
% Simulink, not just in a wiki nobody opens.
subs = {
  % name                     owner          inputs                              outputs
  'IFSSIM_Steering',        'dynamics',    {'Cmd'},                             {'steer'}
  'IFSSIM_Powertrain',      'powertrain',  {'Cmd','wheel_omega'},               {'drive_torque','Powertrain'}
  'IFSSIM_Brakes',          'braking',     {'Cmd','wheel_omega'},               {'brake_torque'}
  'IFSSIM_TireSuspension',  'dynamics',    {'Road','Pose','steer','drive_torque','brake_torque'}, {'Wheels','tyre_force','tyre_torque'}
  'IFSSIM_Aero',            'aero',        {'Pose'},                            {'aero_force','aero_torque'}
  'IFSSIM_Chassis',         'dynamics',    {'tyre_force','tyre_torque','aero_force','aero_torque','Env'}, {'Pose'}
};

% Signal spec: bus name, or numeric width for a plain vector.
spec = containers.Map( ...
  {'Cmd','Road','Env','Pose','Wheels','Powertrain', ...
   'steer','drive_torque','brake_torque','wheel_omega', ...
   'tyre_force','tyre_torque','aero_force','aero_torque'}, ...
  {'Bus: IFSSIM_CmdBus','Bus: IFSSIM_RoadBus','Bus: IFSSIM_EnvBus','Bus: IFSSIM_PoseBus', ...
   'Bus: IFSSIM_WheelsBus','Bus: IFSSIM_PowertrainBus', ...
    4, 4, 4, 4, 3, 3, 3, 3});

for i = 1:size(subs,1)
    make_subsystem_model(subs{i,1}, subs{i,2}, subs{i,3}, subs{i,4}, spec, STEP, outdir);
end

%% ---- top level --------------------------------------------------------
top = 'IFSSIM_Plant';
if bdIsLoaded(top), close_system(top,0); end
f = fullfile(outdir,[top '.slx']);
if isfile(f), delete(f); end
new_system(top,'Model');
configure_model(top, STEP);

x = 40; y = 40;
add_typed_port(top,'Inport','Cmd',  spec, [x y]);        y=y+70;
add_typed_port(top,'Inport','Road', spec, [x y]);        y=y+70;
add_typed_port(top,'Inport','Env',  spec, [x y]);

% Model reference blocks. Numbered because the number IS the signal-flow order
% — steering and torques first, then tyre forces, then the chassis that
% integrates them — and because an unnumbered 'Powertrain' block would collide
% with the Powertrain outport of the same name.
mx = 260; my = 40;
for i = 1:size(subs,1)
    blk = sprintf('%s/%d %s', top, i, strrep(subs{i,1},'IFSSIM_',''));
    add_block('simulink/Ports & Subsystems/Model', blk, ...
        'ModelNameDialog',[subs{i,1} '.slx'], ...
        'Position',[mx my mx+150 my+90]);
    my = my + 120;
end

ox = 560; oy = 40;
add_typed_port(top,'Outport','Pose',       spec, [ox oy]); oy=oy+70;
add_typed_port(top,'Outport','Wheels',     spec, [ox oy]); oy=oy+70;
add_typed_port(top,'Outport','Powertrain', spec, [ox oy]);

%% ---- wiring ------------------------------------------------------------
% Signal flow: commands -> torques and angles -> tyre forces -> motion.
%
% TWO FEEDBACK PATHS ARE BROKEN BY AN EXPLICIT UNIT DELAY. Pose feeds the tyre
% and aero blocks, and wheel speed feeds the powertrain and brakes. Both are
% genuine loops. The subsystems already read last-step state internally, but
% Simulink resolves feedthrough at the BUS level for model references and would
% report an algebraic loop anyway. A visible one-step delay is better than a
% solver working it out invisibly — and at 1/960 s it is 1 ms of lag.
add_block('simulink/Discrete/Unit Delay',[top '/Pose (1 step)'], ...
    'Position',[470 400 540 430],'InitialCondition','IFSSIM_PoseBus_init','SampleTime','-1');
add_block('simulink/Signal Routing/Bus Selector',[top '/Wheel Speed'], ...
    'Position',[600 480 610 520],'OutputSignals','omega');
add_block('simulink/Discrete/Unit Delay',[top '/wheel omega (1 step)'], ...
    'Position',[660 480 730 510],'InitialCondition','[0;0;0;0]','SampleTime','-1');

ST='1 Steering'; PT='2 Powertrain'; BR='3 Brakes';
TS='4 TireSuspension'; AE='5 Aero'; CH='6 Chassis';

% commands out
add_line(top,'Cmd/1',[ST '/1'],'autorouting','on');
add_line(top,'Cmd/1',[PT '/1'],'autorouting','on');
add_line(top,'Cmd/1',[BR '/1'],'autorouting','on');

% platform inputs
add_line(top,'Road/1',[TS '/1'],'autorouting','on');
add_line(top,'Env/1', [CH '/5'],'autorouting','on');

% angles and torques into the tyre block
add_line(top,[ST '/1'],[TS '/3'],'autorouting','on');
add_line(top,[PT '/1'],[TS '/4'],'autorouting','on');
add_line(top,[BR '/1'],[TS '/5'],'autorouting','on');

% tyre and aero wrenches into the chassis
add_line(top,[TS '/2'],[CH '/1'],'autorouting','on');
add_line(top,[TS '/3'],[CH '/2'],'autorouting','on');
add_line(top,[AE '/1'],[CH '/3'],'autorouting','on');
add_line(top,[AE '/2'],[CH '/4'],'autorouting','on');

% outputs
add_line(top,[CH '/1'],'Pose/1','autorouting','on');
add_line(top,[TS '/1'],'Wheels/1','autorouting','on');
add_line(top,[PT '/2'],'Powertrain/1','autorouting','on');

% feedback: pose -> tyre and aero
add_line(top,[CH '/1'],'Pose (1 step)/1','autorouting','on');
add_line(top,'Pose (1 step)/1',[TS '/2'],'autorouting','on');
add_line(top,'Pose (1 step)/1',[AE '/1'],'autorouting','on');

% feedback: wheel speed -> powertrain and brakes
add_line(top,[TS '/1'],'Wheel Speed/1','autorouting','on');
add_line(top,'Wheel Speed/1','wheel omega (1 step)/1','autorouting','on');
add_line(top,'wheel omega (1 step)/1',[PT '/2'],'autorouting','on');
add_line(top,'wheel omega (1 step)/1',[BR '/2'],'autorouting','on');

note = sprintf([ ...
 'IFSSIM PLANT — skeleton\\n\\n' ...
 'Generated by matlab/plant/build_plant_skeleton.m. Regenerating REPLACES this file;\\n' ...
 'subsystem contents live in the referenced models and are safe.\\n\\n' ...
 'Parameters come from settings.json via ifssim_params(), NOT from numbers typed\\n' ...
 'into this model. Mass %.0f kg, wheelbase %.3f m, wheel radius %.3f m,\\n' ...
 'wheel rate %.0f N/m per corner, ride frequency %.2f Hz.\\n\\n' ...
 'Fixed step %s s. The blocks are wired but EMPTY: each emits zeros until its\\n' ...
 'owner fills it in. The model compiles and runs from day one so nobody is\\n' ...
 'blocked waiting for someone else''s subsystem.'], ...
 P.Mass, P.Wheelbase, P.WheelRadius, P.Derived.WheelRateEach, P.Derived.RideFreqHz, STEP);
add_block('built-in/Note',[top '/Overview'],'Position',[40 380],'Text',note, ...
          'HorizontalAlignment','left');

save_system(top, f);
fprintf('  wrote %s\n', f);
close_system(top,0);

fprintf('\nskeleton complete. %d referenced models + top level.\n', size(subs,1));
end

%% =======================================================================
function make_subsystem_model(name, owner, ins, outs, spec, step, outdir)
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end
new_system(name,'Model');
configure_model(name, step);

y = 40;
for i = 1:numel(ins)
    add_typed_port(name,'Inport',ins{i},spec,[40 y]);
    % Terminate every input so the placeholder compiles without warnings. The
    % engineer deletes these as they connect real signals.
    add_block('simulink/Sinks/Terminator',[name '/term_' ins{i}],'Position',[180 y 200 y+20]);
    add_line(name,[ins{i} '/1'],['term_' ins{i} '/1'],'autorouting','on');
    y = y + 70;
end

y = 40;
for i = 1:numel(outs)
    o = outs{i};
    s = spec(o);
    src = ['zero_' o];
    if ischar(s) && startsWith(s,'Bus: ')
        % A zeroed struct of the right bus shape, so the placeholder emits a
        % correctly-typed bus rather than something that only looks right.
        busName = extractAfter(s,'Bus: ');
        zv = Simulink.Bus.createMATLABStruct(busName);
        varName = [busName '_zero'];
        assignin('base', varName, zv);
        add_block('simulink/Sources/Constant',[name '/' src], ...
            'Value',varName,'OutDataTypeStr',s,'Position',[380 y 460 y+30]);
    else
        add_block('simulink/Sources/Constant',[name '/' src], ...
            'Value',sprintf('zeros(%d,1)',s),'Position',[380 y 460 y+30]);
    end
    add_typed_port(name,'Outport',o,spec,[560 y]);
    add_line(name,[src '/1'],[o '/1'],'autorouting','on');
    y = y + 70;
end

add_block('built-in/Note',[name '/Owner'],'Position',[40 y+40], ...
    'Text',sprintf([ ...
      '%s\\n\\nOWNER: %s\\n\\n' ...
      'PLACEHOLDER — emits zeros. Replace the Constant block(s) with the real model.\\n\\n' ...
      'Rules of the contract:\\n' ...
      '  * do not change the port names, types or widths — the platform depends on them\\n' ...
      '  * read parameters from IFSSIM_P (loaded from settings.json), never type numbers in\\n' ...
      '  * keep it fixed-step at %s s and free of continuous states\\n' ...
      '  * body frame is ISO 8855: x forward, y LEFT, z up. Wheel order FL, FR, RL, RR.'], ...
      name, upper(owner), step), 'HorizontalAlignment','left');

save_system(name, f);
fprintf('  wrote %s   (owner: %s)\n', f, owner);
close_system(name,0);
end

%% =======================================================================
function add_typed_port(mdl, kind, portName, spec, pos)
blk = [mdl '/' portName];
if strcmp(kind,'Inport'), lib = 'simulink/Sources/In1'; else, lib = 'simulink/Sinks/Out1'; end
add_block(lib, blk, 'Position',[pos(1) pos(2) pos(1)+30 pos(2)+20]);
s = spec(portName);
if ischar(s)
    set_param(blk,'OutDataTypeStr',s);
    if strcmp(kind,'Inport'), set_param(blk,'BusOutputAsStruct','on'); end
else
    set_param(blk,'PortDimensions',num2str(s));
end
end

%% =======================================================================
function configure_model(mdl, step)
% Fixed-step, discrete, no continuous states: the configuration an FMU export
% needs, applied up front so nobody discovers it at export time.
set_param(mdl, ...
    'SolverType','Fixed-step', ...
    'Solver','FixedStepDiscrete', ...
    'FixedStep',step, ...
    'StartTime','0', ...
    'StopTime','inf', ...
    'SignalLogging','on', ...
    'SaveFormat','Dataset');
end
