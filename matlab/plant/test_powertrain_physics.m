function ok = test_powertrain_physics()
%TEST_POWERTRAIN_PHYSICS  Check the motor envelope, regen limit and battery.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();
addpath(fullfile(fileparts(mfilename('fullpath')),'..','car'));
PK = pack_from_cells(car_spec());

h = 'powertrain_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.1','SaveFormat','Dataset');

add_block('simulink/Ports & Subsystems/Model',[h '/PT'], ...
          'ModelNameDialog','IFSSIM_Powertrain.slx','Position',[240 60 380 160]);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','CMD_P', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/W'],'Value','W_P','Position',[60 130 130 160]);
add_block('simulink/Sinks/To Workspace',[h '/t_out'],'VariableName','t_log', ...
          'SaveFormat','Timeseries','Position',[460 60 530 90]);
add_block('simulink/Sinks/To Workspace',[h '/p_out'],'VariableName','p_log', ...
          'SaveFormat','Timeseries','Position',[460 130 530 160]);
add_line(h,'CMD/1','PT/1','autorouting','on');
add_line(h,'W/1','PT/2','autorouting','on');
add_line(h,'PT/1','t_out/1','autorouting','on');
add_line(h,'PT/2','p_out/1','autorouting','on');

ok = true;
fprintf('\n=== powertrain ===\n');

%% 1. Stall: full throttle, wheels stopped. Torque limited, not power limited.
drive(1, 0, 0); r = sim(h); T = wheelT(r); B = bus(r);
ok = check(ok,'stall: motor at torque limit', B.motor_torque, P.MotorMaxTorque, 1e-9);
ok = check(ok,'stall: rear wheels get gr*eta*T', T(3), ...
           P.MotorMaxTorque*P.GearRatio*P.DrivetrainEfficiency/2, 1e-6);
ok = check(ok,'front wheels get nothing (RWD)', max(abs(T(1:2))), 0, 0);

%% 2. High speed: power limit binds instead of torque limit.
wfast = 200;                                  % rad/s at the wheel
drive(1, 0, wfast); r = sim(h); B = bus(r);
wm = wfast * P.GearRatio;
% THE CAP IS THE ACCUMULATOR, NOT THE MOTOR. These used to assert
% MotorMaxPower, 80 kW, and passed because the battery was computed and then
% ignored -- the envelope was min(Tmax, Pmax/w) and no pack quantity appeared
% in it. The pack is five Simscape modules now and it binds first: 180 A is
% what 6 cells in parallel can pass, and at the voltage the pack is actually
% sitting at that is around 54 kW to the shaft, not 80.
%
% So the assertion is that mechanical power equals what the PACK can give,
% and that this is below the motor's own limit. Asserting 80 kW here would be
% asserting that the accumulator is not part of the car.
Pmech = B.motor_torque * wm;
Pelec = Pmech / P.DrivetrainEfficiency;
Ipack = Pelec / max(B.batt_voltage, 1);
ok = check(ok,'power limited below torque limit', B.motor_torque < P.MotorMaxTorque, true, 0);
ok = check(ok,'accumulator, not motor, is the cap', Pmech < P.MotorMaxPower, true, 0);
ok = check(ok,'draws exactly what the pack can pass', Ipack, PK.IMaxPulse, 1e-3);
fprintf('        pack-limited: %.1f kW to the shaft at %.0f V, %.0f A\n', ...
        Pmech/1000, B.batt_voltage, Ipack);

%% 3. REGEN IS POWER LIMITED. This is what sets the braking capability, and it
%    binds by a large factor at any real speed.
wroll = 10 / P.WheelRadius;
drive(0, 1, wroll); r = sim(h); B = bus(r);
wm = wroll * P.GearRatio;
ok = check(ok,'regen: power limited, not torque limited', B.motor_torque, -P.MaxRegenPower/wm, 1e-6);
ok = check(ok,'regen torque far below envelope', abs(B.motor_torque) < 0.25*P.MaxRegenTorque, true, 0);
ok = check(ok,'regen returns power (negative)', B.motor_power < 0, true, 0);

%% 4. Drive and regen SUM rather than one being discarded.
drive(1, 1, wroll); r = sim(h); B = bus(r);
expect = P.MotorMaxTorque - P.MaxRegenTorque;   % = 0 for equal caps
ok = check(ok,'drive+regen sum, neither discarded', B.motor_torque, max(0,expect), 1e-6);

%% 5. Battery discharges under load and never leaves [0,1].
drive(1, 0, wroll); r = sim(h); B = bus(r); soc = socTrace(r);
ok = check(ok,'SoC decreases under drive', soc(end) < soc(1), true, 0);
ok = check(ok,'SoC stays in [0,1]', all(soc>=0 & soc<=1), true, 0);
ok = check(ok,'terminal voltage sags under load', B.batt_voltage < P.Derived.BatteryVMax, true, 0);

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'powertrain checks PASS.','POWERTRAIN CHECKS FAILED.'));

    function drive(th, rg, w)   % named 'drive', not 'set': set() is a builtin
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=th; c.regen=rg; c.steer_norm=0; c.ebs_latch=0; c.handbrake=0;
        assignin('base','CMD_P',c);
        assignin('base','W_P',[0;0;w;w]);
    end
end

function T = wheelT(r), d = r.get('t_log').Data; T = double(reshape(d(end,:),[],1)); end
function B = bus(r)
s = r.get('p_log'); B = struct();
for n = {'motor_rpm','motor_torque','motor_power','batt_soc','batt_voltage'}
    B.(n{1}) = double(s.(n{1}).Data(end));
end
end
function v = socTrace(r), v = double(r.get('p_log').batt_soc.Data); end

function ok = check(ok,name,got,want,tol)
if islogical(want)
    pass = isequal(logical(got),logical(want));
    fprintf('  [%s] %-42s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass = all(abs(got-want) <= tol);
    fprintf('  [%s] %-42s got %+.6g  want %+.6g\n', ternary(pass,'ok  ','FAIL'), name, got, want);
end
if ~pass, ok = false; end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
