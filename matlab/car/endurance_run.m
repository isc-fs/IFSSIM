function R = endurance_run(nLaps)
%ENDURANCE_RUN  What current does the car actually hold, lap after lap?
%
%   Drives the plant around the Formula Student duty cycle in
%   MODEL_IFS_08/SIMSCAPE/fsDCycle.mat -- a 1.33 km lap, 85 s, mean 56 km/h,
%   peak 95 km/h -- with a driver that simply tracks the speed trace, and
%   reports the current the accumulator is asked for.
%
%   THE NUMBER THAT MATTERS IS RMS, NOT MEAN. Cell heating is I^2*R, so a
%   lap that alternates between hard acceleration and coasting heats the pack
%   far more than its average current suggests. Reporting the mean would
%   flatter the answer in exactly the way that gets a pack cooked.
%
%   R = ENDURANCE_RUN(nLaps) runs nLaps of the cycle back to back.
%
%   See also PACK_FROM_CELLS, ACCEL_RUN.

if nargin < 1 || isempty(nLaps), nLaps = 1; end
here  = fileparts(mfilename('fullpath'));
plant = fullfile(here,'..','plant');
addpath(plant); addpath(fullfile(plant,'models')); addpath(here);
P  = ifssim_load_workspace();
PK = pack_from_cells(car_spec());
ifssim_plant_buses;

% The lap comes from FS_TRACK_CYCLE, not from the drive cycle shipped in
% MODEL_IFS_08/SIMSCAPE. That one is a MathWorks demo trace: it contains
% accelerations of 6.5 g, which nothing can do, and holds above 70 km/h for
% 99 m where the rules cap a straight at 80. Its speeds were never driven by
% a car. FS_TRACK_CYCLE builds a lap from the geometry the rules allow and
% solves the fastest way round it, which lands at the ~50 km/h average the
% rules describe for endurance.
c = fs_track_cycle(false);
lapT = c(end,1);
lapLen = 895;   % m, from fs_track_cycle
t = []; v = [];
for k = 0:nLaps-1
    t = [t; c(1:end-1,1) + k*lapT];   %#ok<AGROW>
    v = [v; c(1:end-1,2)];            %#ok<AGROW>
end
t(end+1) = nLaps*lapT; v(end+1) = c(end,2);
% FEED-FORWARD, not a gain on the error. The first version of this drove the
% lap with throttle = 0.6 * speed error, which saturates at any error over
% 1.7 m/s -- so the car ran full throttle or full regen and almost nothing
% between, tracked the trace with an RMS error of 5.4 m/s, and sat pinned at
% the current limit for a quarter of the lap. That measured the controller,
% not the track.
%
% The cycle already says what acceleration is wanted at every instant, so the
% torque needed to produce it can simply be computed: F = m*a + drag +
% rolling, and the throttle that asks for it. Feedback is left in at low gain
% to correct drift, not to do the driving.
P2 = ifssim_params();
aT = [0; diff(v)./max(diff(t),1e-3)];
Fdrag = 0.5*1.225*P2.CdA*v.^2 + P2.RollingResistance*P2.Mass*9.81;
Freq  = P2.Mass*aT + Fdrag;
Treq  = Freq * P2.WheelRadius / (P2.GearRatio*P2.DrivetrainEfficiency);
ffThr = max(0, min(1,  Treq / P2.MotorMaxTorque));
ffReg = max(0, min(1, -Treq / P2.MaxRegenTorque));
assignin('base','END_CYCLE',[t v ffThr ffReg]);
Tend = t(end);

h = 'endurance_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','0.0010416666666666671','StartTime','0', ...
            'StopTime',num2str(Tend),'SaveFormat','Dataset');

add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[400 60 560 220]);
add_block('simulink/Sources/From Workspace',[h '/Cycle'], ...
          'VariableName','END_CYCLE','Position',[40 40 110 70]);

% Driver: track the speed trace. Proportional, deliberately -- this is a duty
% cycle generator, not a controller study, and integral action would hide the
% current spikes that are the entire point of the exercise.
drv = [h '/Driver'];
add_block('simulink/User-Defined Functions/MATLAB Function', drv, 'Position',[170 40 290 140]);
S = sfroot; ch = S.find('-isa','Stateflow.EMChart','Path',drv);
ch.Script = char(strjoin(string({
"function [throttle, regen] = driver(v_target, v_actual, ff_thr, ff_reg)"
"%#codegen"
"% FEED-FORWARD plus a light correction. The feed-forward terms already carry"
"% the torque the lap asks for, computed from the trace and the car; the gain"
"% below only trims drift. A gain large enough to DRIVE would saturate on"
"% every corner and turn this into a bang-bang duty cycle, which is what it"
"% used to be and what made the current figure meaningless."
"e = v_target - v_actual;"
"k = 0.08;"
"throttle = min(1, max(0, ff_thr + k*e));"
"regen    = min(1, max(0, ff_reg - k*e));"
"end"}), newline));

add_block('simulink/Signal Routing/Bus Creator',[h '/CmdB'],'Position',[320 40 330 160], ...
          'Inputs','5','OutDataTypeStr','Bus: IFSSIM_CmdBus','NonVirtualBus','on');
add_block('simulink/Sources/Constant',[h '/z'],'Value','0','Position',[240 180 270 200]);
add_block('simulink/Discrete/Memory',[h '/v fb'],'Position',[300 260 360 290],'InitialCondition','0');

rd = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
rd.valid=ones(4,1); rd.height=zeros(4,1); rd.mu=P.TireMu*ones(4,1);
rd.normal_x=zeros(4,1); rd.normal_y=zeros(4,1); rd.normal_z=ones(4,1); rd.residual=zeros(4,1);
assignin('base','END_ROAD',rd);
ev = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
ev.gravity_z=-9.81; ev.ext_force=[0;0;0]; ev.ext_torque=[0;0;0];
ev.ext_point=[0;0;0]; ev.chassis_grounded=0;
assignin('base','END_ENV',ev);
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','END_ROAD', ...
          'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[300 300 370 330]);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','END_ENV', ...
          'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[300 350 370 380]);

add_block('simulink/Signal Routing/Demux',[h '/Cyc'],'Outputs','3','Position',[120 40 125 120]);
add_line(h,'Cycle/1','Cyc/1','autorouting','on');
add_line(h,'Cyc/1','Driver/1','autorouting','on');   % target speed
add_line(h,'Cyc/2','Driver/3','autorouting','on');   % feed-forward throttle
add_line(h,'Cyc/3','Driver/4','autorouting','on');   % feed-forward regen
add_line(h,'v fb/1','Driver/2','autorouting','on');  % measured speed
L = {'Driver/1','CmdB/1','throttle'; 'Driver/2','CmdB/2','regen'; 'z/1','CmdB/3','steer_norm'; ...
     'z/1','CmdB/4','ebs_latch'; 'z/1','CmdB/5','handbrake'};
for i = 1:size(L,1)
    lh = add_line(h,L{i,1},L{i,2},'autorouting','on'); set_param(lh,'Name',L{i,3});
end
add_line(h,'CmdB/1','Plant/1','autorouting','on');
add_line(h,'ROAD/1','Plant/2','autorouting','on');
add_line(h,'ENV/1','Plant/3','autorouting','on');
for i = 1:3
    nm = {'pose','wheels','ptrain'};
    add_block('simulink/Sinks/To Workspace',[h '/' nm{i} '_o'],'VariableName',['end_' nm{i}], ...
              'SaveFormat','Timeseries','Position',[620 40+60*i 690 70+60*i]);
    add_line(h,sprintf('Plant/%d',i),[nm{i} '_o/1'],'autorouting','on');
end
% Speed back to the driver, through the memory that breaks the loop.
add_block('simulink/Signal Routing/Bus Selector',[h '/Pose Sel'], ...
          'Position',[600 260 610 300],'OutputSignals','vel_body');
add_line(h,'Plant/1','Pose Sel/1','autorouting','on');
add_block('simulink/Signal Routing/Selector',[h '/vx'],'Position',[500 260 530 290], ...
          'IndexOptions','Index vector (dialog)','Indices','1','InputPortWidth','3');
add_line(h,'Pose Sel/1','vx/1','autorouting','on');
add_line(h,'vx/1','v fb/1','autorouting','on');

fprintf('\nrunning %d lap(s) of the FS cycle (%.0f s)...\n', nLaps, Tend);
r  = sim(h);
pt = r.get('end_ptrain');  po = r.get('end_pose');
tt = pt.motor_power.Time;
Pe = pt.motor_power.Data(:);
Vb = pt.batt_voltage.Data(:);
I  = Pe ./ max(Vb, 1);
vx = po.vel_body.Data(:,1);

Idis = I(I > 0);                      % discharge only; regen is a separate story
R.I_rms   = sqrt(mean(I.^2));
R.I_mean  = mean(Idis);
R.I_peak  = max(I);
R.I_regen = min(I);
R.cell_rms = R.I_rms / PK.Np;
R.watts_cell = R.cell_rms^2 * (PK.Rint*PK.Np/PK.Ns);
R.KperSec  = R.watts_cell / (0.0466*900);
R.t = tt; R.I = I; R.v = vx;

fprintf('\n=== accumulator current over %d lap(s) ===\n', nLaps);
fprintf('  RMS (this is what heats it)   %6.1f A   = %5.2f A per cell\n', R.I_rms, R.cell_rms);
fprintf('  mean while discharging        %6.1f A\n', R.I_mean);
fprintf('  peak                          %6.1f A   = %5.2f A per cell\n', R.I_peak, R.I_peak/PK.Np);
fprintf('  most negative (regen)         %6.1f A\n', R.I_regen);
fprintf('  speed tracked                 %.1f m/s mean, %.1f peak\n', mean(vx), max(vx));
R.pack_watts = R.watts_cell * PK.NCells;
nl = ceil(22000/lapLen);
fprintf('\n  heat, at the RMS current:\n');
fprintf('     %.1f W per cell, %.2f kW into the whole pack\n', R.watts_cell, R.pack_watts/1000);
fprintf('\n  ADIABATIC temperature rise -- NO cooling, no loss to air or structure.\n');
fprintf('  This is an upper bound, and the gap between it and reality IS the\n');
fprintf('  cooling requirement:\n');
fprintf('     %5.1f K over one %.0f s lap\n', R.KperSec*lapT, lapT);
fprintf('     %5.1f K over a 22 km endurance (%d laps, %.0f min)\n', ...
        R.KperSec*lapT*nl, nl, lapT*nl/60);
fprintf('\n  So the pack must shed of order %.1f kW to hold temperature, and\n', R.pack_watts/1000);
fprintf('  from 25 C ambient it reaches the datasheet''s 80 C test limit after\n');
fprintf('  about %.0f minutes with no cooling at all.\n', 55/R.KperSec/60);
close_system(h,0);
end
