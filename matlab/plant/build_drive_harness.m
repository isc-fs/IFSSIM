function build_drive_harness(outdir)
%BUILD_DRIVE_HARNESS  A model you open, run, and watch the car drive.
%
%   The test suite asserts; this one SHOWS. Open it, press Run, look at the
%   scopes. It is the model to keep open while tuning a subsystem.
%
%   The manoeuvre lives in one small MATLAB Function block ("Driver"), so
%   changing what the car does is editing four lines, not rewiring a diagram.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_Drive';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','8', ...
               'SaveFormat','Dataset','SignalLogging','on');

%% ---- driver ------------------------------------------------------------
drv = [name '/Driver'];
add_block('simulink/User-Defined Functions/MATLAB Function', drv, ...
          'Position',[60 60 190 140]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',drv);
chart.Script = driver_code();
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    if any(strcmp(data(k).Name,{'t','throttle','regen','steer'}))
        data(k).Props.Array.Size = '1';
    end
end
add_block('simulink/Sources/Clock',[name '/Clock'],'Position',[10 85 30 105]);
add_line(name,'Clock/1','Driver/1','autorouting','on');

% Driver outputs -> Cmd bus
% A Bus Creator matches elements BY SIGNAL NAME, so every line into it has to
% be named to match the bus object. Unnamed lines compile but warn, and the
% warning is the only thing standing between you and a silently mis-ordered bus.
add_block('simulink/Sources/Constant',[name '/no ebs'],'Value','0','Position',[60 170 100 190]);
add_block('simulink/Sources/Constant',[name '/no handbrake'],'Value','0','Position',[60 210 100 230]);
add_block('simulink/Signal Routing/Bus Creator',[name '/Cmd'], ...
          'Position',[250 55 260 195],'Inputs','5', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','NonVirtualBus','on');
lines = { 'Driver/1','Cmd/1','throttle'
          'Driver/2','Cmd/2','regen'
          'Driver/3','Cmd/3','steer_norm'
          'no ebs/1','Cmd/4','ebs_latch'
          'no handbrake/1','Cmd/5','handbrake' };
for i = 1:size(lines,1)
    lh = add_line(name, lines{i,1}, lines{i,2}, 'autorouting','on');
    set_param(lh,'Name',lines{i,3});
end

%% ---- environment -------------------------------------------------------
assignin('base','IFSSIM_RoadFlat', roadFlat(P));
assignin('base','IFSSIM_EnvFlat',  envFlat());
add_block('simulink/Sources/Constant',[name '/Flat Road'],'Value','IFSSIM_RoadFlat', ...
    'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[250 210 340 240]);
add_block('simulink/Sources/Constant',[name '/Gravity'],'Value','IFSSIM_EnvFlat', ...
    'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[250 260 340 290]);

%% ---- plant -------------------------------------------------------------
add_block('simulink/Ports & Subsystems/Model',[name '/Car'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[430 80 590 260]);
add_line(name,'Cmd/1','Car/1','autorouting','on');
add_line(name,'Flat Road/1','Car/2','autorouting','on');
add_line(name,'Gravity/1','Car/3','autorouting','on');

%% ---- scopes ------------------------------------------------------------
% Bus Selectors pull out what is worth watching. Add a signal here rather than
% digging through logged data afterwards.
add_block('simulink/Signal Routing/Bus Selector',[name '/pose sel'], ...
    'Position',[650 80 660 160],'OutputSignals','vel_body,omega_body,position');
add_line(name,'Car/1','pose sel/1','autorouting','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/wheel sel'], ...
    'Position',[650 200 660 280],'OutputSignals','omega,fz,slip_ratio,slip_angle');
add_line(name,'Car/2','wheel sel/1','autorouting','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/pt sel'], ...
    'Position',[650 320 660 380],'OutputSignals','motor_rpm,motor_power,batt_soc');
add_line(name,'Car/3','pt sel/1','autorouting','on');

scopes = { 'Speed  (body vx,vy,vz)',      'pose sel/1', 760,  60
           'Rates  (roll,pitch,yaw)',     'pose sel/2', 760, 130
           'Position (x,y,z)',            'pose sel/3', 760, 200
           'Wheel speed (FL FR RL RR)',   'wheel sel/1',760, 270
           'Tyre load Fz (N)',            'wheel sel/2',760, 340
           'Slip ratio',                  'wheel sel/3',760, 410
           'Slip angle (rad)',            'wheel sel/4',760, 480
           'Motor RPM',                   'pt sel/1',   760, 550
           'Motor power (W)',             'pt sel/2',   760, 620 };
for i = 1:size(scopes,1)
    blk = [name '/' scopes{i,1}];
    add_block('simulink/Sinks/Scope', blk, ...
        'Position',[scopes{i,3} scopes{i,4} scopes{i,3}+40 scopes{i,4}+40]);
    add_line(name, scopes{i,2}, [scopes{i,1} '/1'], 'autorouting','on');
end

save_system(name,f);
fprintf('wrote %s\n',f);
close_system(name,0);
end

%% =======================================================================
function c = driver_code()
L = {
"function [throttle, regen, steer] = driver(t)"
"%#codegen"
"% The manoeuvre. Edit these four phases to change what the car does."
"%"
"%   0-3 s   accelerate in a straight line"
"%   3-5 s   steady left turn, still on power"
"%   5-6 s   straighten and coast"
"%   6+  s   regen to a stop"
""
"throttle = 0; regen = 0; steer = 0;"
""
"if t < 3"
"    throttle = 0.5;"
"elseif t < 5"
"    throttle = 0.4;  steer = 0.4;"
"elseif t < 6"
"    throttle = 0.2;"
"else"
"    regen = 1.0;"
"end"
"end"
};
c = char(strjoin(string(L), newline));
end

function s = roadFlat(P)
s = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
end
function s = envFlat()
s = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
s.gravity_z=-9.81; s.ext_force=[0;0;0]; s.ext_torque=[0;0;0]; s.ext_point=[0;0;0];
s.chassis_grounded=0;
end
