function R = plant_study(varargin)
%PLANT_STUDY  Try a setup change on the FULL Simulink plant. No rebuild.
%
%   plant_study('HeaveStiffness', 120000)
%   plant_study('RollStiffnessFront', 32000, 'RollStiffnessRear', 17000)
%   plant_study('SuspensionDamping', 0.9)
%   plant_study('CoGHeight', 0.3441)
%
%   Runs a steady-state corner on the assembled plant twice -- the car as
%   specified, and the car with your change -- and prints both. Roll, corner
%   loads, lateral acceleration, ride frequency.
%
%   WHY THIS EXISTS ALONGSIDE VD_STUDY. vd_study drives the design model: two
%   states, algebraic load transfer, no suspension dynamics. It is the right
%   tool for balance and lap time and it answers in seconds. It cannot see
%   anything the plant has and it does not: dampers, ride, transient weight
%   transfer, a wheel lifting over a kerb. For those, ask the plant.
%
%   NO REBUILD IS NEEDED and that is not an accident. The suspension numbers
%   reach the models as base-workspace variables -- IFSSIM_kw, IFSSIM_cw,
%   IFSSIM_arbF, IFSSIM_arbR -- which Simulink resolves when the model
%   compiles. Change the variable, run again, done. Verified: tripling
%   IFSSIM_kw takes roll in a fixed corner from 0.92 to 0.43 degrees with the
%   .slx files untouched.
%
%   THE EXCEPTION IS THE TYRE. Its Magic Formula coefficients are written into
%   the block's mask when the model is BUILT, from ifssim_tyre.mat. Change
%   TireMu or a Pacejka number and this rebuilds the tyre first, which takes
%   about a minute. It will tell you when it does.
%
%   To make a change permanent: put it in matlab/spec/car_spec.m with a source
%   and run build_car. Nothing here touches the spec or settings.json.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
ifssim_workdir();
if isempty(varargin)
    error('plant_study:noChange', ...
          ['nothing to study. Give a parameter and a value, e.g.\n' ...
           '   plant_study(''HeaveStiffness'', 120000)']);
end

% Anything the tyre block bakes in at build time. Everything else is live.
TYRE = {'TireMu','Pacejka','Tyre','WheelRadius','WheelWidth','Mass','WeightDistFront'};
needsBuild = false;
for i = 1:2:numel(varargin)
    k = varargin{i};
    if any(cellfun(@(t) startsWith(k,t), TYRE)), needsBuild = true; end
end

P0 = ifssim_load_workspace();
A  = corner_case(P0, 'as specified');

fprintf('\n================== PLANT STUDY ==================\n');
for i = 1:2:numel(varargin)
    was = getdot(P0, varargin{i});
    fprintf('  %-28s %g  ->  %g\n', varargin{i}, was, varargin{i+1});
end
if needsBuild
    fprintf('\n  a tyre parameter changed, so the tyre set has to be rebuilt.\n');
    fprintf('  this takes about a minute; suspension-only changes do not.\n');
end

P1 = ifssim_load_workspace(varargin);
if needsBuild
    build_tyre_paramset(fullfile(here,'models'));
    build_tiresuspension(fullfile(here,'models'));
    ifssim_load_workspace(varargin);      % the rebuild reloads the baseline
end
B = corner_case(P1, 'study');

fprintf('\n  %-30s %12s %12s %10s\n', '', 'as specified', 'study', 'change');
row('roll [deg]',                 A.roll, B.roll);
row('lateral acceleration [g]',   A.ay/9.81, B.ay/9.81);
row('load transfer front [N]',    A.dFf, B.dFf);
row('load transfer rear [N]',     A.dFr, B.dFr);
row('front share of transfer [%]',A.frontPct, B.frontPct);
row('inside front load [N]',      A.Fz(1), B.Fz(1));
row('ride frequency [Hz]',        P0.Derived.RideFreqHz, P1.Derived.RideFreqHz);
row('wheel rate [N/m]',           P0.Derived.WheelRateEach, P1.Derived.WheelRateEach);
row('anti-roll bar front [N.m/rad]', P0.Derived.ArbFront, P1.Derived.ArbFront);

if min(B.Fz) < 1 && min(A.Fz) >= 1
    fprintf('\n  WARNING: this setup lifts an inside wheel in this corner.\n');
end
fprintf('================================================\n');
fprintf('  vd_report / vd_study for balance and lap time; this for the\n');
fprintf('  things only the full plant has.\n\n');

R = struct('baseline',A,'study',B,'overrides',{varargin},'rebuilt',needsBuild);

% put the workspace back, so a study leaves nothing behind
ifssim_load_workspace();
if needsBuild
    build_tyre_paramset(fullfile(here,'models'));
    build_tiresuspension(fullfile(here,'models'));
end
end

% -----------------------------------------------------------------------
function S = corner_case(P, ~)
h = 'plant_study_corner';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','1/960', ...
            'StartTime','0','StopTime','5.0','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);
s = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
s.valid=ones(4,1); s.height=zeros(4,1); s.mu=P.TireMu*ones(4,1);
s.normal_x=zeros(4,1); s.normal_y=zeros(4,1); s.normal_z=ones(4,1); s.residual=zeros(4,1);
assignin('base','PS_ROAD',s);
e = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
e.gravity_z=-9.81; e.ext_force=[0;0;0]; e.ext_torque=[0;0;0]; e.ext_point=[0;0;0];
e.chassis_grounded=0; assignin('base','PS_ENV',e);
c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
c.throttle=0.25; c.regen=0; c.steer_norm=0.5; c.ebs_latch=0; c.handbrake=0;
assignin('base','PS_CMD',c);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','PS_CMD','OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','PS_ROAD','OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 120 130 150]);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','PS_ENV','OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 180 130 210]);
add_line(h,'CMD/1','Plant/1','autorouting','on');
add_line(h,'ROAD/1','Plant/2','autorouting','on');
add_line(h,'ENV/1','Plant/3','autorouting','on');
add_block('simulink/Sinks/To Workspace',[h '/po'],'VariableName','ps_pose','SaveFormat','Timeseries','Position',[500 70 570 100]);
add_block('simulink/Sinks/To Workspace',[h '/wo'],'VariableName','ps_wheel','SaveFormat','Timeseries','Position',[500 130 570 160]);
add_line(h,'Plant/1','po/1','autorouting','on');
add_line(h,'Plant/2','wo/1','autorouting','on');

r  = sim(h);
p  = r.get('ps_pose');  w = r.get('ps_wheel');
n  = size(p.quat.Data,1);  k = round(0.9*n):n;
q  = mean(p.quat.Data(k,:),1);  q = q/norm(q);
S.roll = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))*180/pi;
vb = p.vel_body.Data;  ob = p.omega_body.Data;
S.ay = mean(vb(k,1).*ob(k,3));
F = mean(w.fz.Data(k,:),1);
S.Fz  = F;
S.dFf = abs(F(1)-F(2))/2;  S.dFr = abs(F(3)-F(4))/2;
S.frontPct = 100*S.dFf/max(S.dFf+S.dFr, eps);
end

function row(name, a, b)
fprintf(['  %-30s %12.3f %12.3f %+9.1f%%\n'], name, a, b, 100*(b-a)/max(abs(a),eps));
end

function v = getdot(P, key)
parts = split(key,'.');  v = NaN;
try
    v = P.(parts{1});
    for i = 2:numel(parts), v = v.(parts{i}); end
catch
end
end
