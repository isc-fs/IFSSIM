function ok = test_steering_physics()
%TEST_STEERING_PHYSICS  Check the steering block against Ackermann geometry.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

h = 'steering_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.5','SaveFormat','Dataset');

add_block('simulink/Ports & Subsystems/Model',[h '/ST'], ...
          'ModelNameDialog','IFSSIM_Steering.slx','Position',[240 60 380 140]);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','CMD_S', ...
          'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 70 130 100]);
add_block('simulink/Sinks/To Workspace',[h '/s_out'],'VariableName','s_log', ...
          'SaveFormat','Timeseries','Position',[460 85 530 115]);
add_line(h,'CMD/1','ST/1','autorouting','on');
add_line(h,'ST/1','s_out/1','autorouting','on');

ok = true;
fprintf('\n=== steering ===\n');
L = P.Wheelbase; t = P.TrackFront; dmax = P.Derived.MaxSteerAngleRad;

%% 1. Straight ahead.
setcmd(0); r = sim(h); d = last(r);
ok = check(ok,'zero command -> zero angle', max(abs(d)), 0, 1e-12);

%% 2. Rear wheels never steer.
setcmd(1); r = sim(h); d = last(r);
ok = check(ok,'rear wheels do not steer', max(abs(d(3:4))), 0, 0);

%% 3. Full left: the single-track angle is MaxSteerAngle, and the wheels
%    straddle it — inner MORE, outer LESS. This is the check that would have
%    caught Chaos's reverse-Ackermann default.
inner = atan(L / (abs(L/tan(dmax)) - t/2));
outer = atan(L / (abs(L/tan(dmax)) + t/2));
ok = check(ok,'left turn: left (inner) wheel angle',  d(1), inner, 1e-6);
ok = check(ok,'left turn: right (outer) wheel angle', d(2), outer, 1e-6);
ok = check(ok,'INNER steers MORE than outer', d(1) > d(2), true, 0);
ok = check(ok,'single-track angle brackets both', d(2) < dmax && dmax < d(1), true, 0);

%% 4. Mirror symmetry.
setcmd(-1); r = sim(h); dn = last(r);
ok = check(ok,'right turn mirrors left', [dn(1) dn(2)], [-d(2) -d(1)], 1e-9);

%% 5. Both front wheels share one turn centre — the definition of Ackermann.
setcmd(0.5); r = sim(h); d = last(r);
yc_l = L/tan(d(1)) + t/2;      % lateral distance from each wheel to the centre
yc_r = L/tan(d(2)) - t/2;
ok = check(ok,'front wheels share a turn centre', yc_l, yc_r, 1e-6);

%% 6. Command is clamped.
setcmd(3); r = sim(h); d = last(r);
ok = check(ok,'command clamped at 1.0', d(1), inner, 1e-6);

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'steering checks PASS.','STEERING CHECKS FAILED.'));

    function setcmd(v)
        c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
        c.throttle=0; c.regen=0; c.steer_norm=v; c.ebs_latch=0; c.handbrake=0;
        assignin('base','CMD_S',c);
    end
end

function d = last(r)
x = r.get('s_log').Data;  d = double(reshape(x(end,:),[],1));
end

function ok = check(ok,name,got,want,tol)
if islogical(want)
    pass = isequal(logical(got),logical(want));
    fprintf('  [%s] %-40s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass = all(abs(got(:)-want(:)) <= tol);
    fprintf('  [%s] %-40s got %s  want %s\n', ternary(pass,'ok  ','FAIL'), name, ...
            mat2str(round(got(:)',6)), mat2str(round(want(:)',6)));
end
if ~pass, ok = false; end
end

function s = ternary(c,a,b)
if c, s=a; else, s=b; end
end
