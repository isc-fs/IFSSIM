function ok = vdb_step3_check()
%VDB_STEP3_CHECK  The block's vertical load against the closed form it replaces.
%
%   Step 3 of docs/vdb_plant_migration.md hands the tyre's Fext over to the
%   suspension block's own WhlF, clamped at zero and gated on ground contact.
%
%   Both forces are logged in the SAME run -- ours is kept and terminated
%   rather than deleted -- so there is no second build to drift and no
%   question of comparing two different parameter sets.
%
%   What must agree: static loads, load transfer in roll and pitch, and the
%   wheel-lift clamp. Load transfer is not at risk by construction, because it
%   never lived in a static term -- it arises from the corner deflections the
%   body pose produces, and the block is driven by those same deflections.
%
%   What must NOT agree, and is reported rather than asserted: the anti-roll
%   bar. Ours is dead linear in roll; the block's sweeps on an arm and softens
%   as it works. That is real geometry, and the size of it is the point.

here = fileparts(mfilename('fullpath'));
addpath(here); addpath(fullfile(here,'models'));
ok = true;
fprintf('\n=== step 3: block WhlF vs the closed form ===\n');

evalc('build_tiresuspension([], [], true)');
P = ifssim_load_workspace();

% 'match' needs travel that is EQUAL across each axle, because any difference
% engages the anti-roll bar and the two bars are not the same model. A
% one-wheel bump is therefore an 'arb' case, not a 'match' case -- labelling it
% 'match' was simply wrong, and the check was right to fail it.
%
%          name                         roll     pitch   road z (m)      expect
cases = {'static, level',               0.0,     0.0,   [0;0;0;0],      'match'
         'pitch 2 deg (braking)',       0.0,     0.035, [0;0;0;0],      'match'
         'heave, all four 20 mm bump',  0.0,     0.0,   [0.02;0.02;0.02;0.02], 'match'
         'front-left on a 40 mm bump',  0.0,     0.0,   [0.04;0;0;0],   'arb'
         'front-left 40 mm of droop',   0.0,     0.0,   [-0.04;0;0;0],  'arb'
         'front-left off the road',     0.0,     0.0,   [-1;0;0;0],     'clamp'
         'body roll 1 deg',             0.0175,  0.0,   [0;0;0;0],      'arb'
         'body roll 3 deg',             0.0524,  0.0,   [0;0;0;0],      'arb'
         'body roll 5 deg',             0.0873,  0.0,   [0;0;0;0],      'arb'};

R = run_cases(cases, P);

fprintf('\n  %-28s %10s %10s %10s %8s\n', ...
        'case','sum ours','sum block','max d [N]','d/Fz');
for c = 1:size(cases,1)
    r = R(c);
    d = max(abs(r.ours - r.block));
    rel = d / max(sum(r.ours), 1);
    fprintf('  %-28s %10.2f %10.2f %10.3f %7.2f%%', cases{c,1}, ...
            sum(r.ours), sum(r.block), d, 100*rel);
    switch cases{c,5}
        case 'match'
            if d > 1e-3, fprintf('  <-- DIVERGED'); ok = false; end
        case 'clamp'
            if any(r.block < -1e-9), fprintf('  <-- NEGATIVE Fz'); ok = false; end
            if r.block(1) ~= 0, fprintf('  <-- lifted wheel still loaded'); ok = false; end
        case 'arb'
            fprintf('  <-- ARB arm sweep, expected');
    end
    fprintf('\n');
    if d > 1e-6
        fprintf('       per wheel ours  [%8.1f %8.1f %8.1f %8.1f]\n', r.ours);
        fprintf('       per wheel block [%8.1f %8.1f %8.1f %8.1f]\n', r.block);
    end
end

% Static loads are an absolute check, not a relative one: they are the one
% number in here with an answer known independently of either model.
r = R(1); mg = P.Mass*9.81;
ok = assert_near(ok,'block: total Fz = m*g',    sum(r.block), mg, 1e-3);
ok = assert_near(ok,'block: front corner load', r.block(1), P.Derived.StaticLoadFront, 1e-3);
ok = assert_near(ok,'block: rear corner load',  r.block(3), P.Derived.StaticLoadRear,  1e-3);

fprintf(['\n  The ARB rows are the behaviour change this step buys and costs.\n' ...
         '  Ours is linear in travel difference; the block''s bar sweeps on a\n' ...
         '  %.2f m arm and softens as it works. Under 5 deg of body roll that is\n' ...
         '  about 1%% of total load -- small, but roll stiffness sets the balance,\n' ...
         '  so it is reported rather than absorbed. If it looks too large, the arm\n' ...
         '  radius in car_spec is a guess and wants measuring: it sets how\n' ...
         '  PROGRESSIVE the bar is, not just its rate.\n\n' ...
         '  The off-the-road row is where they part company completely, and it is\n' ...
         '  worth understanding rather than dismissing. A dropped wheel puts a\n' ...
         '  metre of travel difference across the axle. Our linear bar happily\n' ...
         '  extrapolates that to about 8.7 kN dumped on the opposite wheel --\n' ...
         '  three times the weight of the car, out of an anti-roll bar. The\n' ...
         '  block''s arm has swept over by then and delivers a bounded fraction of\n' ...
         '  it. Neither model is VALIDATED out there; the block is merely the one\n' ...
         '  that fails safely. A real bar would have hit its droop limit long\n' ...
         '  before, and modelling that is the bump-stop job Hmax does not do.\n'], ...
         P.Susp.ArbArmRadiusFront);
if ok, fprintf('\n  Step 3 force path agrees where it must.\n');
else,  fprintf('\n  Step 3 force path DISAGREES.\n'); end
end

% -------------------------------------------------------------------------
function out = run_cases(cases, P)
h = 'vdb_step3_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','1/960', ...
            'StartTime','0','StopTime','0.25','SaveFormat','Dataset', ...
            'SignalLogging','on','SignalLoggingName','tsLog');
add_block('simulink/Ports & Subsystems/Model',[h '/TS'], ...
          'ModelNameDialog','IFSSIM_TireSuspension.slx', ...
          'SimulationMode','Normal','Position',[260 60 420 260]);
names = {'ROAD','POSE','STEER','DRV','BRK'};
for k = 1:5
    add_block('simulink/Sources/Constant',[h '/' names{k}],'Value','0', ...
              'Position',[60 40+60*k 130 70+60*k]);
    add_line(h,[names{k} '/1'],sprintf('TS/%d',k),'autorouting','on');
end
set_param([h '/ROAD'],'OutDataTypeStr','Bus: IFSSIM_RoadBus');
set_param([h '/POSE'],'OutDataTypeStr','Bus: IFSSIM_PoseBus');
set_param([h '/STEER'],'Value','[0;0;0;0]');
set_param([h '/DRV'],'Value','[0;0;0;0]');
set_param([h '/BRK'],'Value','[0;0;0;0]');
for k = 1:3
    add_block('simulink/Sinks/Terminator',[h '/t' num2str(k)],'Position',[500 40+40*k 520 56+40*k]);
    add_line(h,sprintf('TS/%d',k),['t' num2str(k) '/1'],'autorouting','on');
end

out = struct('ours',{},'block',{});
for c = 1:size(cases,1)
    phi = cases{c,2};  th = cases{c,3};  hgt = cases{c,4};
    road = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
    road.valid = ones(4,1); road.height = hgt; road.mu = P.TireMu*ones(4,1);
    road.normal_x = zeros(4,1); road.normal_y = zeros(4,1);
    road.normal_z = ones(4,1);  road.residual = zeros(4,1);
    pose = Simulink.Bus.createMATLABStruct('IFSSIM_PoseBus');
    pose.position = [0;0;P.CoGHeight];
    cr = cos(phi/2); sr = sin(phi/2); cp = cos(th/2); sp = sin(th/2);
    pose.quat = [cr*cp; sr*cp; cr*sp; -sr*sp];      % roll then pitch
    pose.vel_world=[0;0;0]; pose.vel_body=[0;0;0]; pose.omega_body=[0;0;0];
    pose.alpha_body=[0;0;0]; pose.accel_proper=[0;0;0]; pose.attitude=[phi;th;0];
    assignin('base','S3_ROAD',road); assignin('base','S3_POSE',pose);
    set_param([h '/ROAD'],'Value','S3_ROAD');
    set_param([h '/POSE'],'Value','S3_POSE');
    r = sim(h);  L = r.get('tsLog');
    out(c).ours  = last_sample(L.get('fz_ours').Values.Data);
    out(c).block = last_sample(L.get('fz_block').Values.Data);
end
close_system(h,0);
end

function v = last_sample(D)
if ndims(D) == 3, v = D(:,:,end); else, v = D(end,:); end
v = double(v(:));
end

function ok = assert_near(ok,nm,got,want,tol)
pass = all(abs(got-want) <= tol);
if pass, tag = 'ok  '; else, tag = 'FAIL'; end
fprintf('  [%s] %-34s got %+.4f  want %+.4f\n', tag, nm, got, want);
if ~pass, ok = false; end
end
