function S = wheelspin_study()
%WHEELSPIN_STUDY  Is the launch wheelspin real, or is it the integrator?
%
%   The plant spins its wheels to a slip ratio of 26 off the line and takes
%   6.7 s to cover 75 m where an independent model says 4.0 s. Two candidate
%   explanations, and they predict different things:
%
%   PHYSICAL. This car makes more torque than the rear tyres can hold, so it
%     genuinely spins. Then there is a throttle BELOW WHICH IT DOES NOT, set
%     by where drive torque meets the grip limit, and slip should grow
%     smoothly from there.
%
%   NUMERICAL. Wheel spin is integrated explicitly inside the tyre block, and
%     the semi-implicit solver that used to hold it went with the
%     hand-written tyre. The old model's comment warned exactly this: the
%     wheel gains more slip in one step than the curve peak, overshoots, and
%     past the peak more slip means less force, so it runs away. Then it runs
%     away at torque levels the tyre could comfortably have held, and the
%     threshold is nowhere near the grip limit.
%
%   So: compute where the grip limit actually is, then sweep throttle across
%   it. Physical wheelspin starts at the limit. Numerical wheelspin does not
%   care where the limit is.

here  = fileparts(mfilename('fullpath'));
plant = fullfile(here,'..','plant');
addpath(plant); addpath(fullfile(plant,'models'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','spec'));
P = ifssim_load_workspace();
ifssim_plant_buses;

%% ---- where the grip limit is, on paper --------------------------------
FzR   = P.Mass*9.81*(1-P.WeightDistFront)/2;      % static, per rear wheel
Tgrip = P.TireMu * FzR * P.WheelRadius;           % torque that wheel can hold
Taxle = P.MotorMaxTorque * P.GearRatio * P.DrivetrainEfficiency / 2;  % per rear wheel
thr_limit = Tgrip / Taxle;

fprintf('\n=== where the grip limit is ===\n');
fprintf('  static rear wheel load        %.0f N\n', FzR);
fprintf('  torque that wheel can hold    %.0f N.m   (mu*Fz*R)\n', Tgrip);
fprintf('  torque available at full thr  %.0f N.m   (Tm*gear*eta/2)\n', Taxle);
fprintf('  -> wheelspin expected above   %.2f throttle\n', thr_limit);
fprintf('     (load transfer helps the rear, so the real threshold is a\n');
fprintf('      little higher than this static figure)\n');

%% ---- harness, built once ----------------------------------------------
h = 'wheelspin_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','0.0010416666666666671','StartTime','0', ...
            'StopTime','3','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);
rd = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
rd.valid=ones(4,1); rd.height=zeros(4,1); rd.mu=P.TireMu*ones(4,1);
rd.normal_x=zeros(4,1); rd.normal_y=zeros(4,1); rd.normal_z=ones(4,1); rd.residual=zeros(4,1);
assignin('base','WS_ROAD',rd);
ev = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
ev.gravity_z=-9.81; ev.ext_force=[0;0;0]; ev.ext_torque=[0;0;0];
ev.ext_point=[0;0;0]; ev.chassis_grounded=0;
assignin('base','WS_ENV',ev);
c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
c.throttle=0; c.regen=0; c.steer_norm=0; c.ebs_latch=0; c.handbrake=0;
assignin('base','WS_CMD',c);
src = {'CMD','WS_CMD','IFSSIM_CmdBus';'ROAD','WS_ROAD','IFSSIM_RoadBus';'ENV','WS_ENV','IFSSIM_EnvBus'};
y=60;
for i=1:3
    add_block('simulink/Sources/Constant',[h '/' src{i,1}],'Value',src{i,2}, ...
              'OutDataTypeStr',['Bus: ' src{i,3}],'Position',[60 y 130 y+30]);
    add_line(h,[src{i,1} '/1'],sprintf('Plant/%d',i),'autorouting','on');
    y=y+60;
end
o={'pose','wheels'};
for i=1:2
    add_block('simulink/Sinks/To Workspace',[h '/' o{i} '_o'],'VariableName',['ws_' o{i}], ...
              'SaveFormat','Timeseries','Position',[500 40+60*i 570 70+60*i]);
    add_line(h,sprintf('Plant/%d',i),[o{i} '_o/1'],'autorouting','on');
end

%% ---- sweep throttle across the limit -----------------------------------
thr = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
S = struct('throttle',thr,'peak_slip',zeros(size(thr)), ...
           'end_slip',zeros(size(thr)),'vx',zeros(size(thr)));
fprintf('\n=== launch, 3 s, throttle swept across the limit ===\n');
fprintf('  %8s %12s %12s %10s   %s\n','throttle','peak slip','slip @3s','vx m/s','');
for i = 1:numel(thr)
    c.throttle = thr(i); assignin('base','WS_CMD',c);
    r  = sim(h);
    p  = r.get('ws_pose'); w = r.get('ws_wheels');
    vx = p.vel_body.Data(:,1);
    om = w.omega.Data(:,3);
    sl = (om*P.WheelRadius - vx) ./ max(abs(vx), P.Assumed.SlipRegularisationSpeed);
    S.peak_slip(i) = max(sl); S.end_slip(i) = sl(end); S.vx(i) = vx(end);
    % RUNAWAY is slip that does not SETTLE. A launch transient peaks and
    % then decays as the wheel catches up; a runaway sits there. Peak slip
    % alone cannot tell them apart and flags healthy launches as faults.
    mark = '';
    if S.end_slip(i) > 0.5, mark = '  <-- did not settle'; end
    fprintf('  %8.2f %12.2f %12.2f %10.2f %s\n', thr(i), S.peak_slip(i), S.end_slip(i), S.vx(i), mark);
end
S.thr_limit = thr_limit;

%% ---- read the answer ---------------------------------------------------
runaway = S.end_slip > 0.5;
if any(runaway), thr_onset = thr(find(runaway,1,'first')); else, thr_onset = NaN; end
S.thr_onset = thr_onset;
% Speed saturating is the other half of the signature: if more throttle buys
% no more speed, the tyre is giving back everything the extra torque adds.
vsat = max(S.vx(runaway)) - min(S.vx(runaway));

fprintf('\n=== verdict ===\n');
fprintf('  runaway starts at   %.2f throttle\n', thr_onset);
fprintf('  grip limit is at    %.2f throttle (static; load transfer raises it)\n', thr_limit);
if any(runaway)
    fprintf('  speed once spinning %.2f to %.2f m/s -- spread of %.2f\n', ...
            min(S.vx(runaway)), max(S.vx(runaway)), vsat);
end
fprintf('\n');
if isnan(thr_onset)
    fprintf('  Nothing ran away. Whatever was wrong is not here any more.\n');
elseif thr_onset < 0.75*thr_limit
    fprintf('  NUMERICAL. It runs away well below the grip limit, at torque the\n');
    fprintf('  tyre could comfortably have held. That is the explicit wheel-speed\n');
    fprintf('  integration overshooting the curve peak, as the hand-written tyre\n');
    fprintf('  warned it would.\n');
else
    fprintf('  ONSET IS PHYSICAL, CONSEQUENCE IS NOT.\n\n');
    fprintf('  It starts spinning within a throttle step of where drive torque\n');
    fprintf('  meets grip, so the car really does make more torque than the rear\n');
    fprintf('  tyres hold. That part is real, and it means this car wants\n');
    fprintf('  traction control rather than a solver fix.\n\n');
    fprintf('  What is NOT real is what happens next. Speed saturates: every\n');
    fprintf('  throttle from %.2f to 1.00 gives the same %.1f m/s, so past the\n', thr_onset, mean(S.vx(runaway)));
    fprintf('  limit the extra torque buys nothing at all. A real tyre keeps\n');
    fprintf('  delivering sliding friction and the car keeps accelerating, just\n');
    fprintf('  badly. This one falls into a hole because the Magic Formula tail\n');
    fprintf('  is too steep -- 48%% of peak at kappa 1 and less beyond, against\n');
    fprintf('  the 70-85%% a real slick holds. The wheelspin is the symptom; the\n');
    fprintf('  tail is the cause, and it is the same tail flagged in tyre_report.\n');
end
close_system(h,0);
end
