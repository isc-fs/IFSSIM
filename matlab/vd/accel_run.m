function R = accel_run(s_target, throttle)
%ACCEL_RUN  Standing-start acceleration over a distance, run on the real plant.
%
%   R = ACCEL_RUN(75) launches the car from rest at full throttle and reports
%   when it has covered 75 m -- the Formula Student acceleration event.
%
%   THIS IS THE PATTERN FOR THE MANUAL TEAM. The point of it is not the
%   number it prints; it is that the number comes from the SAME plant the
%   driverless simulator drives, parameterised from the SAME car_spec. The
%   hand-written longitudinal model in DYNAMIC_MOD/GeneralCalculations does
%   the same job with its own copy of the mass, the tyre and the drag -- and
%   its own copy of a number is its own copy of a mistake. Anything that
%   model can answer, this one can answer without a second set of parameters
%   to keep in step.
%
%   What you get here that a point-mass model cannot give you:
%
%     - wheel spin as a real state, so wheelspin off the line is a RESULT
%       rather than something you decide to model or not
%     - load transfer, so the rear axle gains grip under acceleration by
%       itself, through the suspension, rather than through a formula
%     - the actual tyre, at the actual slip, including what it does past
%       the peak
%     - aero, brakes, powertrain and battery, already wired in
%
%   R.t_target   time to cover s_target                     [s]
%   R.v_end      speed at that point                        [m/s]
%   R.v_end_kmh  the same in km/h
%   R.hist       [t  s  v  ax  motor_rpm  wheel_slip]  -- same shape as the
%                legacy accelSimDistance, so the two can be plotted together
%
%   See also CAR_SPEC, BUILD_CAR.

if nargin < 1 || isempty(s_target), s_target = 75;  end
if nargin < 2 || isempty(throttle), throttle = 1.0; end

here  = fileparts(mfilename('fullpath'));
plant = fullfile(here,'..','plant');
addpath(plant); addpath(fullfile(plant,'models'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','spec'));
P = ifssim_load_workspace();
ifssim_plant_buses;

h = 'accel_run_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
% ode1 at the plant's own step, and long enough that 75 m is comfortably
% inside the run: the crossing is found afterwards, not simulated up to.
set_param(h,'SolverType','Fixed-step','Solver','ode1', ...
            'FixedStep','0.0010416666666666671','StartTime','0', ...
            'StopTime','12','SaveFormat','Dataset');

add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);

c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
c.throttle = throttle; c.regen = 0; c.steer_norm = 0;
c.ebs_latch = 0; c.handbrake = 0;
assignin('base','ACC_CMD', c);

rd = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
rd.valid = ones(4,1); rd.height = zeros(4,1); rd.mu = P.TireMu*ones(4,1);
rd.normal_x = zeros(4,1); rd.normal_y = zeros(4,1); rd.normal_z = ones(4,1);
rd.residual = zeros(4,1);
assignin('base','ACC_ROAD', rd);

ev = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
ev.gravity_z = -9.81; ev.ext_force = [0;0;0]; ev.ext_torque = [0;0;0];
ev.ext_point = [0;0;0]; ev.chassis_grounded = 0;
assignin('base','ACC_ENV', ev);

src = {'CMD','ACC_CMD','IFSSIM_CmdBus'; 'ROAD','ACC_ROAD','IFSSIM_RoadBus'; ...
       'ENV','ACC_ENV','IFSSIM_EnvBus'};
y = 60;
for i = 1:3
    add_block('simulink/Sources/Constant',[h '/' src{i,1}],'Value',src{i,2}, ...
              'OutDataTypeStr',['Bus: ' src{i,3}],'Position',[60 y 130 y+30]);
    add_line(h,[src{i,1} '/1'],sprintf('Plant/%d',i),'autorouting','on');
    y = y + 60;
end

out = {'pose','wheels','ptrain'};
for i = 1:3
    add_block('simulink/Sinks/To Workspace',[h '/' out{i} '_out'], ...
              'VariableName',['acc_' out{i}],'SaveFormat','Timeseries', ...
              'Position',[500 40+60*i 570 70+60*i]);
    add_line(h,sprintf('Plant/%d',i),[out{i} '_out/1'],'autorouting','on');
end

r = sim(h);
p  = r.get('acc_pose');
w  = r.get('acc_wheels');
pt = r.get('acc_ptrain');
t  = p.position.Time;
x  = p.position.Data(:,1);
vx = p.vel_body.Data(:,1);
rpm   = pt.motor_rpm.Data(:);
omega = w.omega.Data(:,3);                      % rear left
ax = [0; diff(vx)./max(diff(t),eps)];
slip = (omega*P.WheelRadius - vx) ./ max(abs(vx), P.Assumed.SlipRegularisationSpeed);

R = struct();
R.hist = [t x vx ax rpm slip];

i = find(x >= s_target, 1, 'first');
if isempty(i)
    warning('accel_run:short','only covered %.1f m in %.1f s', x(end), t(end));
    R.t_target = NaN; R.v_end = vx(end);
else
    % Interpolate rather than take the sample: at 960 Hz the sample is close,
    % but reporting a lap-time-like number to the sample is sloppy.
    j = max(i-1,1);
    if x(i) > x(j)
        f = (s_target - x(j)) / (x(i) - x(j));
    else
        f = 0;
    end
    R.t_target = t(j) + f*(t(i)-t(j));
    R.v_end    = vx(j) + f*(vx(i)-vx(j));
end
R.v_end_kmh = R.v_end * 3.6;
R.s_target  = s_target;
R.throttle  = throttle;

fprintf('\n=== acceleration, on the plant ===\n');
fprintf('  throttle           %.2f\n', throttle);
fprintf('  0 - %.0f m          %.3f s\n', s_target, R.t_target);
fprintf('  speed there        %.1f km/h (%.2f m/s)\n', R.v_end_kmh, R.v_end);
fprintf('  peak wheel slip    %.2f   (launch wheelspin, a result not a setting)\n', max(slip));
fprintf('  peak motor rpm     %.0f\n', max(rpm));
close_system(h,0);
end
