function R = plant_ab(mode, tag)
%PLANT_AB  Compare the plant against a recorded baseline, signal by signal.
%
%   plant_ab('capture')        record the plant as it stands as the reference
%   plant_ab('check')          run it again and report every divergence
%   plant_ab('capture','vdb')  keep more than one reference, by name
%
%   THE POINT OF THIS IS THE VDB MIGRATION. Steps 1 to 4 of
%   docs/vdb_plant_migration.md each replace a piece of the plant with a
%   Vehicle Dynamics Blockset block. Without a baseline there is no way to tell
%   a modelling difference from a mistake, and every step after the first is
%   guesswork on top of guesswork. Capture before touching anything.
%
%   It is deliberately NOT a pass/fail test. ifssim_plant_check already asserts
%   what must be true; this answers a different question -- WHAT CHANGED and BY
%   HOW MUCH -- because during a migration most differences are expected and
%   the job is to notice the one that is not.
%
%   Fixed step, fixed inputs, no randomness: two runs of the same plant agree
%   to the last bit, so any divergence is the change under test.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models')); addpath(fullfile(here,'..','spec'));
ifssim_workdir();
if nargin < 1 || isempty(mode), mode = 'check'; end
if nargin < 2 || isempty(tag),  tag  = 'baseline'; end
refFile = fullfile(here, 'reference', ['plant_ab_' tag '.mat']);

P = ifssim_load_workspace();
h = build_harness(P);

% The manoeuvres. Chosen to exercise exactly what the migration touches:
% static loads, roll and lateral transfer, pitch under power, a wheel leaving
% the ground, and a transient. A migration that changes none of these has not
% changed the car; one that changes all of them has, and this says which.
M = { 'settle',      0.00, 0.0, 0.0, 3.0, false
      'accelerate',  1.00, 0.0, 0.0, 4.0, false
      'steady corner',0.25, 0.5, 0.0, 5.0, false
      'hard corner', 0.25, 1.0, 0.0, 5.0, false
      'brake',       0.00, 0.0, 1.0, 3.0, false
      'wheel lift',  0.25, 1.0, 0.0, 4.0, true  };

R = struct('tag',tag,'runs',struct([]));
fprintf('\n=== plant A/B: %s ===\n', mode);
for i = 1:size(M,1)
    r = run_case(h, P, M{i,2}, M{i,3}, M{i,4}, M{i,5}, M{i,6});
    r.name = M{i,1};
    R.runs(i).name = M{i,1};
    R.runs(i).sig  = r;
end

if strcmpi(mode,'capture')
    if ~isfolder(fileparts(refFile)), mkdir(fileparts(refFile)); end
    ref = R; %#ok<NASGU>
    save(refFile,'ref');
    fprintf('  captured %d manoeuvres to %s\n', numel(R.runs), refFile);
    summarise(R);
    return;
end

if ~isfile(refFile)
    error('plant_ab:noReference', ...
          ['no reference at %s.\n\nRun  plant_ab(''capture'')  BEFORE changing ' ...
           'the plant, not after.'], refFile);
end
L = load(refFile);  ref = L.ref;
compare(ref, R);
end

% -----------------------------------------------------------------------
function summarise(R)
fprintf('\n  %-15s %9s %9s %9s %9s %9s\n','manoeuvre','x [m]','|roll|deg','ay [g]','minFz','maxFz');
for i = 1:numel(R.runs)
    s = R.runs(i).sig;
    fprintf('  %-15s %9.4f %9.4f %9.4f %9.1f %9.1f\n', R.runs(i).name, ...
            s.x_end, abs(s.roll_deg), s.ay_g, s.fz_min, s.fz_max);
end
end

function compare(ref, cur)
fields = {'x_end','y_end','z_end','roll_deg','pitch_deg','yaw_deg', ...
          'vx_end','ay_g','fz_min','fz_max','fz_front','fz_rear', ...
          'transfer_front','transfer_rear','omega_rl','travel_max'};
worst = 0;  worstWhat = '';
for i = 1:numel(cur.runs)
    nm = cur.runs(i).name;
    j = find(strcmp({ref.runs.name}, nm), 1);
    if isempty(j)
        fprintf('  %-15s  NEW, no reference\n', nm); continue;
    end
    a = ref.runs(j).sig;  b = cur.runs(i).sig;
    lines = {};
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(a,f) || ~isfield(b,f), continue; end
        d = b.(f) - a.(f);
        rel = abs(d)/max(abs(a.(f)), 1e-6);
        if abs(d) > 1e-9
            lines{end+1} = sprintf('      %-16s %12.5f -> %12.5f  (%+.3g, %+.2f%%)', ...
                                   f, a.(f), b.(f), d, 100*rel); %#ok<AGROW>
            if rel > worst, worst = rel; worstWhat = sprintf('%s / %s', nm, f); end
        end
    end
    if isempty(lines)
        fprintf('  %-15s  identical\n', nm);
    else
        fprintf('  %-15s  %d signals differ\n', nm, numel(lines));
        fprintf('%s\n', lines{:});
    end
end
fprintf('\n  largest relative change: %.2f%%  (%s)\n', 100*worst, worstWhat);
if worst == 0
    fprintf('  NOTHING CHANGED. If you expected a change, it did not reach the plant.\n');
end
end

% -----------------------------------------------------------------------
function h = build_harness(P)
h = 'plant_ab_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','1/960', ...
            'StartTime','0','SaveFormat','Dataset');
add_block('simulink/Ports & Subsystems/Model',[h '/Plant'], ...
          'ModelNameDialog','IFSSIM_Plant.slx','Position',[260 60 420 220]);
add_block('simulink/Sources/Constant',[h '/CMD'],'Value','AB_CMD', ...
    'OutDataTypeStr','Bus: IFSSIM_CmdBus','Position',[60 60 130 90]);
add_block('simulink/Sources/Constant',[h '/ROAD'],'Value','AB_ROAD', ...
    'OutDataTypeStr','Bus: IFSSIM_RoadBus','Position',[60 120 130 150]);
add_block('simulink/Sources/Constant',[h '/ENV'],'Value','AB_ENV', ...
    'OutDataTypeStr','Bus: IFSSIM_EnvBus','Position',[60 180 130 210]);
add_line(h,'CMD/1','Plant/1','autorouting','on');
add_line(h,'ROAD/1','Plant/2','autorouting','on');
add_line(h,'ENV/1','Plant/3','autorouting','on');
add_block('simulink/Sinks/To Workspace',[h '/po'],'VariableName','ab_pose', ...
    'SaveFormat','Timeseries','Position',[500 70 570 100]);
add_block('simulink/Sinks/To Workspace',[h '/wo'],'VariableName','ab_wheel', ...
    'SaveFormat','Timeseries','Position',[500 130 570 160]);
add_line(h,'Plant/1','po/1','autorouting','on');
add_line(h,'Plant/2','wo/1','autorouting','on');
end

function s = run_case(h, P, thr, steer, regen, tEnd, lift)
c = Simulink.Bus.createMATLABStruct('IFSSIM_CmdBus');
c.throttle=thr; c.regen=regen; c.steer_norm=steer; c.ebs_latch=0; c.handbrake=0;
assignin('base','AB_CMD',c);
rd = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
rd.valid=ones(4,1); rd.height=zeros(4,1); rd.mu=P.TireMu*ones(4,1);
rd.normal_x=zeros(4,1); rd.normal_y=zeros(4,1); rd.normal_z=ones(4,1); rd.residual=zeros(4,1);
if lift
    % Front-left over a hole deep enough that the wheel cannot reach it. The
    % one manoeuvre that asks whether a tyre can stop carrying load at all.
    rd.height(1) = -0.5;
end
assignin('base','AB_ROAD',rd);
e = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
e.gravity_z=-9.81; e.ext_force=[0;0;0]; e.ext_torque=[0;0;0]; e.ext_point=[0;0;0];
e.chassis_grounded=0; assignin('base','AB_ENV',e);

set_param(h,'StopTime',num2str(tEnd));
r = sim(h);
p = r.get('ab_pose');  w = r.get('ab_wheel');
pos = p.position.Data;  q = p.quat.Data;  vb = p.vel_body.Data;  ob = p.omega_body.Data;
fz = w.fz.Data;  om = w.omega.Data;  tr = w.susp_travel.Data;
n = size(pos,1);  k = round(0.9*n):n;
qq = mean(q(k,:),1);  qq = qq/norm(qq);
s.x_end = pos(end,1);  s.y_end = pos(end,2);  s.z_end = pos(end,3);
s.roll_deg  = atan2(2*(qq(1)*qq(2)+qq(3)*qq(4)), 1-2*(qq(2)^2+qq(3)^2))*180/pi;
s.pitch_deg = asin(max(-1,min(1,2*(qq(1)*qq(3)-qq(4)*qq(2)))))*180/pi;
s.yaw_deg   = atan2(2*(qq(1)*qq(4)+qq(2)*qq(3)), 1-2*(qq(3)^2+qq(4)^2))*180/pi;
s.vx_end = vb(end,1);
s.ay_g   = mean(vb(k,1).*ob(k,3))/9.81;
F = mean(fz(k,:),1);
s.fz_min = min(F);  s.fz_max = max(F);
s.fz_front = (F(1)+F(2))/2;  s.fz_rear = (F(3)+F(4))/2;
s.transfer_front = abs(F(1)-F(2))/2;  s.transfer_rear = abs(F(3)-F(4))/2;
s.omega_rl = mean(om(k,3));
s.travel_max = max(abs(tr(end,:)));
end
