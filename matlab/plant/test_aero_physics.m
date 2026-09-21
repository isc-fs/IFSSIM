function ok = test_aero_physics()
%TEST_AERO_PHYSICS  Check drag, downforce, balance and the body-frame claim.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
P = ifssim_load_workspace();

h = 'aero_test_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
            'FixedStep','1/960','StartTime','0','StopTime','0.05','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/AE'], ...
          'ModelNameDialog','IFSSIM_Aero.slx','Position',[240 60 380 140]);
add_block('simulink/Sources/Constant',[h '/POSE'],'Value','POSE_A', ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus','Position',[60 70 130 100]);
add_block('simulink/Sinks/To Workspace',[h '/F'],'VariableName','fl', ...
          'SaveFormat','Timeseries','Position',[460 60 520 90]);
add_block('simulink/Sinks/To Workspace',[h '/M'],'VariableName','ml', ...
          'SaveFormat','Timeseries','Position',[460 120 520 150]);
add_line(h,'POSE/1','AE/1','autorouting','on');
add_line(h,'AE/1','F/1','autorouting','on');
add_line(h,'AE/2','M/1','autorouting','on');

ok = true;
fprintf('\n=== aero ===\n');
rho = P.Assumed.AirDensity;

%% 1. At rest there is no aero at all.
setv([0;0;0]); r = sim(h);
ok = check(ok,'at rest: no force',  max(abs(F(r))), 0, 1e-12);
ok = check(ok,'at rest: no moment', max(abs(M(r))), 0, 1e-12);

%% 2. Straight ahead at 20 m/s: closed-form drag and downforce.
v = 20; q = 0.5*rho*v^2;
setv([v;0;0]); r = sim(h); f = F(r);
ok = check(ok,'drag = q*CdA, opposing motion', f(1), -q*P.CdA, 1e-6);
ok = check(ok,'downforce = q*ClA, pressing down', f(3), -q*P.ClA, 1e-6);
ok = check(ok,'no lateral force in a straight line', f(2), 0, 1e-12);
fprintf('        at 20 m/s: %.0f N drag, %.0f N downforce\n', -f(1), -f(3));

%% 3. Downforce scales with the square of speed.
setv([40;0;0]); r = sim(h); f40 = F(r);
ok = check(ok,'downforce quadruples when speed doubles', f40(3), 4*f(3), 1e-6);

%% 4. Balance: the pitch moment matches the declared front/rear split.
%    Front downforce ahead of the CoG pitches the nose DOWN.
m = M(r);
Fz = 0.5*rho*40^2*P.ClA;
expectMy = P.Derived.aFront*(Fz*P.AeroBalanceFront) ...
         - P.Derived.bRear *(Fz*(1-P.AeroBalanceFront));
ok = check(ok,'pitch moment matches the aero balance', m(2), expectMy, 1e-6);
ok = check(ok,'no roll or yaw moment in a straight line', [m(1) m(3)], [0 0], 1e-12);

%% 5. THE BODY-FRAME CLAIM. Sliding sideways must be dragged sideways — a model
%    that only drags along x would report zero here.
setv([0;10;0]); r = sim(h); f = F(r);
ok = check(ok,'pure sideslip is dragged sideways', f(2), -0.5*rho*100*P.CdA, 1e-6);
ok = check(ok,'sideslip still generates downforce', f(3) < 0, true);

close_system(h,0);
fprintf('\n%s\n', ternary(ok,'aero checks PASS.','AERO CHECKS FAILED.'));

    function setv(vb)
        s = Simulink.Bus.createMATLABStruct('IFSSIM_PoseBus');
        s.position=[0;0;P.CoGHeight]; s.quat=[1;0;0;0];
        s.vel_world=[0;0;0]; s.vel_body=vb; s.omega_body=[0;0;0];
        s.alpha_body=[0;0;0]; s.accel_proper=[0;0;0]; s.attitude=[0;0;0];
        assignin('base','POSE_A',s);
    end
end

function f = F(r), d=r.get('fl').Data; f=double(reshape(d(end,:),[],1)); end
function m = M(r), d=r.get('ml').Data; m=double(reshape(d(end,:),[],1)); end

function ok = check(ok,name,got,want,tol)
if nargin<5, tol=0; end
if islogical(want)
    pass = isequal(logical(got),logical(want));
    fprintf('  [%s] %-44s %s\n', ternary(pass,'ok  ','FAIL'), name, ternary(logical(got),'true','false'));
else
    pass = all(abs(got(:)-want(:)) <= tol);
    fprintf('  [%s] %-44s got %s  want %s\n', ternary(pass,'ok  ','FAIL'), name, ...
            mat2str(round(got(:)',4)), mat2str(round(want(:)',4)));
end
if ~pass, ok = false; end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
