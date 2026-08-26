function ok = test_brakes_physics()
%TEST_BRAKES_PHYSICS  Check the EBS, and that it stops the car.
%
%   Two levels: the block on its own, then the WHOLE CAR braking — because the
%   number that matters is a stopping distance, not a torque.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();
ok = true;
fprintf('\n=== brakes ===\n');

%% ---- the block alone --------------------------------------------------
h = 'brakes_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.05','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/BR'], ...
          'ModelNameDialog','IFSSIM_Brakes.slx','Position',[240 60 380 140]);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','CMD_B', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/W'],'Value','[50;50;50;50]','Position',[60 120 130 150]);
add_block('simulink/Sinks/To Workspace',[h '/T'],'VariableName','tl', ...
          'SaveFormat','Timeseries','Position',[460 80 520 110]);
add_line(h,'CMD/1','BR/1','autorouting','on');
add_line(h,'W/1','BR/2','autorouting','on');
add_line(h,'BR/1','T/1','autorouting','on');

setcmd(0,0); r = sim(h);
ok = check(ok,'no command: no brake torque', max(abs(T(r))), 0, 1e-12);
setcmd(1,0); r = sim(h); t = T(r);
ok = check(ok,'EBS latched: full torque, all four wheels', t, ...
           P.Derived.EbsTorquePerWheel*ones(4,1), 1e-6);
setcmd(0,1); r = sim(h);
ok = check(ok,'handbrake drives the same system', T(r), t, 1e-9);
fprintf('        EBS %.0f N.m/wheel vs grip limit %.0f N.m\n', ...
        t(1), P.TireMu*(P.Mass*9.81/4)*P.WheelRadius);
close_system(h,0);

%% ---- the whole car braking --------------------------------------------
d = 'brakes_stop_harness';
if bdIsLoaded(d), close_system(d,0); end
new_system(d,'Model');
set_param(d,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','9','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[d '/Car'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);
assignin('base','ROAD_B', roadFlat(P)); assignin('base','ENV_B', envFlat());
add_block('simulink/Sources/Constant',[d '/CMD'],'Value','CMD_S', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[d '/ROAD'],'Value','ROAD_B', ...
          'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 120 130 150]);
add_block('simulink/Sources/Constant',[d '/ENV'],'Value','ENV_B', ...
          'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 180 130 210]);
add_block('simulink/Sinks/To Workspace',[d '/P'],'VariableName','pl', ...
          'SaveFormat','Timeseries','Position',[500 70 560 100]);
add_block('simulink/Sinks/To Workspace',[d '/WH'],'VariableName','wl', ...
          'SaveFormat','Timeseries','Position',[500 140 560 170]);
add_line(d,'CMD/1','Car/1','autorouting','on');
add_line(d,'ROAD/1','Car/2','autorouting','on');
add_line(d,'ENV/1','Car/3','autorouting','on');
add_line(d,'Car/1','P/1','autorouting','on');
add_line(d,'Car/2','WH/1','autorouting','on');

% A time-based driver, so one run can accelerate AND then brake. A Constant
% cannot change mid-simulation, and the number worth having is a stopping
% distance, not a torque.
drv = [d '/Driver'];
add_block('simulink/User-Defined Functions/MATLAB Function', drv,'Position',[60 250 190 320]);
S2 = sfroot; ch = S2.find('-isa','Stateflow.EMChart','Path',drv);
ch.Script = char(strjoin(string({
 "function [throttle, ebs] = drv(t)"
 "%#codegen"
 "% Accelerate for 4 s, then latch the EBS and let it stop."
 "if t < 4"
 "    throttle = 0.6; ebs = 0;"
 "else"
 "    throttle = 0;   ebs = 1;"
 "end"
 "end"}), newline));
for dd = ch.find('-isa','Stateflow.Data')'
    if any(strcmp(dd.Name,{'t','throttle','ebs'})), dd.Props.Array.Size = '1'; end
end
add_block('simulink/Sources/Clock',[d '/Clock'],'Position',[20 275 40 295]);
add_block('simulink/Sources/Constant',[d '/z1'],'Value','0','Position',[60 340 90 360]);
add_block('simulink/Signal Routing/Bus Creator',[d '/CmdB'],'Position',[240 245 250 365], ...
          'Inputs','5','OutDataTypeStr','Bus: IFSSIM_CmdBus','NonVirtualBus','on');
add_line(d,'Clock/1','Driver/1','autorouting','on');
L = {'Driver/1','CmdB/1','throttle'; 'z1/1','CmdB/2','regen'; 'z1/1','CmdB/3','steer_norm';
     'Driver/2','CmdB/4','ebs_latch'; 'z1/1','CmdB/5','handbrake'};
for i = 1:size(L,1)
    lh = add_line(d,L{i,1},L{i,2},'autorouting','on'); set_param(lh,'Name',L{i,3});
end
delete_line(d, 'CMD/1', 'Car/1'); delete_block([d '/CMD']);
add_line(d,'CmdB/1','Car/1','autorouting','on');

r = sim(d); A = pose(r); W = wheels(r);
[~,i4] = min(abs(A.t-4));
v0 = A.vx(i4);
ok = check(ok,'car reaches speed before braking', v0 > 8, true);

% Stopping distance from the moment the EBS latches to the moment it stops.
moving = find(A.t > 4 & A.vx > 0.1);
if isempty(moving)
    stopIdx = i4;
else
    stopIdx = moving(end);
end
dist = A.x(stopIdx) - A.x(i4);
decel = v0 / max(A.t(stopIdx) - 4, 1e-6);
ok = check(ok,'EBS brings the car to a stop', A.vx(end) < 0.5, true);
ok = check(ok,'wheels lock rather than reverse', min(W.omega) >= -1e-9, true);
% Locked-wheel deceleration is set by the TYRE at full slip, not by the torque.
% mu * mf(-1) * g with these coefficients is about 8 m/s^2.
ok = check(ok,'deceleration is grip-limited, not torque-limited', ...
           decel > 4 && decel < P.TireMu*9.81*1.2, true);
fprintf('        EBS stop from %.1f m/s: %.1f m in %.2f s  (%.1f m/s^2, %.2f g)\n', ...
        v0, dist, A.t(stopIdx)-4, decel, decel/9.81);

close_system(d,0);
fprintf('\n%s\n', ternary(ok,'brake checks PASS.','BRAKE CHECKS FAILED.'));

    function setcmd(e,hb)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=0; c.regen=0; c.steer_norm=0; c.ebs_latch=e; c.handbrake=hb;
        assignin('base','CMD_B',c);
    end
    function stepcmd(th,rg,e)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=th; c.regen=rg; c.steer_norm=0; c.ebs_latch=e; c.handbrake=0;
        assignin('base','CMD_S',c);
    end
end

function t = T(r), d=r.get('tl').Data; t=double(reshape(d(end,:),[],1)); end
function A = pose(r)
p=r.get('pl'); A.t=p.position.Time; A.x=p.position.Data(:,1); A.vx=p.vel_body.Data(:,1);
end
function W = wheels(r), w=r.get('wl'); W.omega=w.omega.Data(:); end
function s = roadFlat(P)
s=Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
end
function s = envFlat()
s=Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
s.gravity_z=-9.81; s.ext_force=[0;0;0]; s.ext_torque=[0;0;0]; s.ext_point=[0;0;0];
s.chassis_grounded=0;
end
function ok = check(ok,name,got,want,tol)
if nargin<5, tol=0; end
if islogical(want)
    pass=isequal(logical(got),logical(want));
    fprintf('  [%s] %-44s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass=all(abs(got(:)-want(:))<=tol);
    fprintf('  [%s] %-44s got %s\n', ternary(pass,'ok  ','FAIL'), name, mat2str(round(got(:)',3)));
end
if ~pass, ok=false; end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
