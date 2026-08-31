function ok = vdb_step2_check()
%VDB_STEP2_CHECK  Baseline vs VDB double-wishbone kinematics, same inputs.
%
%   Step 2 of docs/vdb_plant_migration.md swaps our closed-form camber for
%   the VDB "Independent Suspension - Double Wishbone" block. Nothing else
%   in the load path changes, so the two builds must agree: same camber ->
%   same tyre forces -> same wrench. Drift here means the block's mask
%   convention disagrees with ours, which is the whole point of the check.
%
%   Run it after any change to vdb_camber_mask or to the suspension
%   parameters in car_spec.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
ok = true;
fprintf('\n=== step 2: VDB suspension kinematics A/B ===\n');

% Camber only reaches the wrench through the tyre's lateral force, so a
% standing car cannot discriminate the two builds -- every case needs road
% speed and a slip angle, or the comparison is vacuous and passes blind.
%
% 'match' cases are pure roll and heave-free, where the two formulations model
% the same thing and must agree. The tolerance is 5e-3 deg, which is the
% small-angle residual and nothing else: the closed form is linear in roll,
% the block works off travel proportional to sin(roll), and at 3 deg that gap
% is 4.6e-4 deg.
%
% 'gained' cases put a wheel into heave. The block produces camber there and
% the closed form cannot -- it has no term for it. A difference is the point,
% not a fault, so the check reports the size instead of asserting on it.
%
%          name                          roll rad steer rad  road z (m)   vel_body      omega_body   expect
cases = {'rolling straight, 10 m/s',      0.0,    0.0,    [0;0;0;0],    [10;0;0],     [0;0;0],    'match'
         'roll 3 deg, 2 deg sideslip',    0.0524, 0.0,    [0;0;0;0],    [10;0.35;0],  [0;0;0],    'match'
         'roll 3 deg, 15 deg lock, yaw',  0.0524, 0.262,  [0;0;0;0],    [10;0.35;0],  [0;0;0.8],  'match'
         'roll 5 deg, opposite lock',    -0.0873,-0.262,  [0;0;0;0],    [12;-0.6;0],  [0;0;-1.1], 'match'
         'front-left on a 40 mm bump',    0.0,    0.0,    [0.04;0;0;0], [10;0.35;0],  [0;0;0],    'gained'
         'front-left in a hole',          0.0,    0.0,    [-1;0;0;0],   [10;0.35;0],  [0;0;0],    'gained'};

R = cell(2,1);
for v = 1:2
    evalc('build_tiresuspension([], [], v == 2)');    % quiet; v==2 is the VDB build
    R{v} = run_cases(cases);
end

fprintf(['\n  Camber at the tyre input, front-left / front-right, degrees.\n' ...
         '  This is the comparison that matters: the wrench cannot see camber\n' ...
         '  at all while the Magic Formula camber terms are zeroed.\n\n']);
TOL = 5e-3;   % deg -- the small-angle residual, see the case table
fprintf('  %-31s %8s %8s %8s %8s %9s\n', ...
        'case','base FL','VDB FL','base FR','VDB FR','max d deg');
for c = 1:size(cases,1)
    a = R{1}(c); b = R{2}(c);
    dc = max(abs(a.camber - b.camber))*180/pi;
    fprintf('  %-31s %8.3f %8.3f %8.3f %8.3f %9.2e', cases{c,1}, ...
            a.camber(1)*180/pi, b.camber(1)*180/pi, ...
            a.camber(2)*180/pi, b.camber(2)*180/pi, dc);
    if strcmp(cases{c,7},'match')
        if dc > TOL, fprintf('  <-- DIVERGED'); ok = false; end
    else
        fprintf('  <-- heave camber, closed form has none');
    end
    fprintf('\n');
end

dw = max(abs([R{1}(2).F - R{2}(2).F; R{1}(2).M - R{2}(2).M]));
fprintf('\n  wrench delta (expected 0 -- camber is inert in the tyre): %.2e\n', dw);

if ok
    fprintf(['\n  On roll the two agree to the small-angle residual, so the block\n' ...
             '  is wired and masked correctly. On heave it produces camber the\n' ...
             '  closed form never had -- that is what adopting it buys.\n\n' ...
             '  Still true and still worth saying: the tyre IGNORES all of this.\n' ...
             '  The Magic Formula camber terms are zero for want of rig data, so\n' ...
             '  the wrench delta above is 0 by construction, not by agreement.\n' ...
             '  The block''s own Fz is computed and terminated; taking it is\n' ...
             '  step 3, and it needs a max(Fz,0) clamp the block does not do.\n']);
else
    fprintf('\n  DISAGREEMENT -- check vdb_camber_mask against the block dialog.\n');
end
end

% -------------------------------------------------------------------------
function out = run_cases(cases)
P = ifssim_load_workspace();
h = 'vdb_step2_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','1/960', ...
            'StartTime','0','StopTime','0.25','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/TS'], ...
          'ModelNameDialog','IFSSIM_TireSuspension.slx', ...
          'SimulationMode','Normal', ...   % accelerator would hide the probe
          'Position',[260 60 420 260]);
set_param(h,'SignalLogging','on','SignalLoggingName','tsLog');
names = {'ROAD','POSE','STEER','DRV','BRK'};
for k = 1:5
    add_block('simulink/Sources/Constant',[h '/' names{k}],'Value','0', ...
              'Position',[60 40+60*k 130 70+60*k]);
    add_line(h,[names{k} '/1'],sprintf('TS/%d',k),'autorouting','on');
end
set_param([h '/ROAD'],'OutDataTypeStr','Bus: IFSSIM_RoadBus');
set_param([h '/POSE'],'OutDataTypeStr','Bus: IFSSIM_PoseBus');
set_param([h '/DRV'],'Value','[0;0;0;0]');
set_param([h '/BRK'],'Value','[0;0;0;0]');
outs = {'w_log','f_log','m_log'};
for k = 1:3
    add_block('simulink/Sinks/To Workspace',[h '/o' num2str(k)], ...
              'VariableName',outs{k},'SaveFormat','Timeseries', ...
              'Position',[500 40+60*k 570 70+60*k]);
    add_line(h,sprintf('TS/%d',k),['o' num2str(k) '/1'],'autorouting','on');
end

out = struct('fz',{},'fy',{},'F',{},'M',{},'camber',{});
for c = 1:size(cases,1)
    phi = cases{c,2};  dlt = cases{c,3};  hgt = cases{c,4};
    vb  = cases{c,5};  wb  = cases{c,6};
    road = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
    road.valid = ones(4,1); road.height = hgt; road.mu = P.TireMu*ones(4,1);
    road.normal_x = zeros(4,1); road.normal_y = zeros(4,1);
    road.normal_z = ones(4,1);  road.residual = zeros(4,1);
    pose = Simulink.Bus.createMATLABStruct('IFSSIM_PoseBus');
    pose.position = [0;0;P.CoGHeight];
    pose.quat = [cos(phi/2); sin(phi/2); 0; 0];      % roll about body x
    pose.vel_world=[0;0;0]; pose.vel_body=vb; pose.omega_body=wb;
    pose.alpha_body=[0;0;0]; pose.accel_proper=[0;0;0]; pose.attitude=[phi;0;0];
    assignin('base','VDB_ROAD',road); assignin('base','VDB_POSE',pose);
    set_param([h '/ROAD'],'Value','VDB_ROAD');
    set_param([h '/POSE'],'Value','VDB_POSE');
    set_param([h '/STEER'],'Value',sprintf('[%.10g;%.10g;0;0]',dlt,dlt));
    r = sim(h);
    w = r.get('w_log');
    out(c).fz = double(reshape(w.fz.Data(end,:),[],1));
    out(c).fy = double(reshape(w.fy.Data(end,:),[],1));
    out(c).F  = double(reshape(r.get('f_log').Data(end,:),[],1));
    out(c).M  = double(reshape(r.get('m_log').Data(end,:),[],1));
    % A logged 4x1 column arrives as [4 1 N], a logged 4-vector as [N 4].
    % Slicing both with (end,:) silently flattens the first into 4N values.
    out(c).camber = last_sample(r.get('tsLog').get('camber_at_tyre').Values.Data);
end
close_system(h,0);
end

% -------------------------------------------------------------------------
function v = last_sample(D)
if ndims(D) == 3, v = D(:,:,end); else, v = D(end,:); end
v = double(v(:));
end
