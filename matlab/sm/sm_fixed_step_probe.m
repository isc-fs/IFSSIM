function R = sm_fixed_step_probe(withFMU, F)
%SM_FIXED_STEP_PROBE  Can a closed-loop multibody corner live in the plant's FMU?
%
%   R = SM_FIXED_STEP_PROBE()       solver study only
%   R = SM_FIXED_STEP_PROBE(true, F) ... at a load F [N] (default 600, ~30 mm)
%   R = SM_FIXED_STEP_PROBE(true)   ... and export the best fixed-step
%                                   configuration as an FMI 3.0 Co-Simulation
%                                   FMU, run it back, and compare
%
%   The question the whole "multibody in the plant" idea turns on, asked of
%   the smallest thing that can answer it: ONE corner of the car (the real
%   IFS-08 front hardpoints), a closed kinematic loop -- two wishbones, track
%   rod, pushrod, rocker -- with the car's spring and damper on the damper
%   line, a vertical load stepped onto the contact patch, and wheel travel
%   out. Gravity off and the spring unloaded at the hardpoints, so static is
%   exactly the hardpoints and any drift is the solver's.
%
%   Run four ways and compared against a tight variable-step reference:
%     ode23t       the design-study solver (the reference, tol 1e-8)
%     ode14x       global fixed-step, implicit, at the plant's 1/960 s
%     ode1be       global fixed-step, backward Euler, 1/960 s
%     ode1 + local the PLANT's configuration: explicit global ode1 with a
%                  backward-Euler Simscape local solver, 1/960 s
%   For each: does it run, how far is it from the reference, and how fast
%   (the simulator needs real time with margin: the whole plant shares it).
%
%   WHAT IT FOUND (2026-10-04, IFS-08 front):
%     at 600 N (28.7 mm, the travel the car uses) every configuration runs,
%     the PLANT'S included (0.09 mm from the reference), and its FMU, stepped
%     as the simulator steps it (tools/fmu/step_timing), runs at ~80x real
%     time -- about what the whole current plant costs. So a multibody corner
%     FITS the real-time plant.
%     The plant's solver holds the linkage to 58.5 mm of bump and loses it
%     past that, where the linkage itself snaps through (the variable-step
%     reference jumps from 58.5 to 92 mm for 200 N more). A bump stop before
%     that is a requirement, on the model and on the car.
%     The first run used 1500 N, put the wheel 90 mm into bump, and concluded
%     the opposite -- an unrepresentative load, kept in this note so nobody
%     repeats it.

if nargin < 1, withFMU = false; end
if nargin < 2 || isempty(F), F = 600; end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
P = ifssim_load_workspace();
wd = fullfile(ifssim_workdir(), 'sm_probe');  if ~isfolder(wd), mkdir(wd); end
addpath(wd);
STEP = '0.0010416666666666671';     % the plant's step (1/960 s)
% 600 N: about the travel the car really uses (+-30 mm). 1500 N, the first
% choice, drove the wheel 90 mm into bump -- past the swept range and into the
% rocker's near-toggle region, which is a stress test of the wrong thing.
T_END = 0.6;  T_STEP = 0.05;               % F: N up at the contact patch

H = sm_hardpoints('front');
mdl = 'IFSSIM_SM_Probe';
evalc('build_sm_corner(H, mdl, wd);');
load_system(mdl);
instrument(mdl, H, P, T_STEP, F);
set_param(mdl, 'StopTime', num2str(T_END));
save_system(mdl);

cases = { 'ode23t (reference)', @(m) set_param(m,'SolverType','Variable-step','Solver','ode23t','RelTol','1e-8','AbsTol','1e-10','MaxStep','1e-3'), false
          'ode14x, 1/960',      @(m) set_param(m,'SolverType','Fixed-step','Solver','ode14x','FixedStep',STEP), false
          'ode1be, 1/960',      @(m) set_param(m,'SolverType','Fixed-step','Solver','ode1be','FixedStep',STEP), false
          'ode1 + local BE (the plant''s)', @(m) set_param(m,'SolverType','Fixed-step','Solver','ode1','FixedStep',STEP), true };
R = struct('name',{},'ok',{},'t',{},'z',{},'wall',{},'rtf',{},'err_mm',{},'msg',{});
for k = 1:size(cases,1)
    cases{k,2}(mdl);
    set_param([mdl '/Solver'], 'UseLocalSolver', tern(cases{k,3}, 'on', 'off'));
    if cases{k,3}
        set_param([mdl '/Solver'], 'LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER', ...
                  'LocalSolverSampleTime', STEP, 'DoFixedCost','on');
    end
    r = struct('name',cases{k,1},'ok',false,'t',[],'z',[],'wall',NaN,'rtf',NaN,'err_mm',NaN,'msg','');
    try
        evalc('sim(mdl);');                         % warm: compile once
        tic;  out = sim(mdl);  r.wall = toc;
        zz = out.get('PROBE_z');
        r.t = zz.Time;  r.z = squeeze(zz.Data);
        r.ok = all(isfinite(r.z));
        r.rtf = T_END / r.wall;
    catch ME
        r.msg = strtrim(regexprep(ME.message, '\s+', ' '));
    end
    R(end+1) = r; %#ok<AGROW>
end

ref = R(1);
fprintf('\n=== one closed-loop multibody corner (IFS-08 front), %g N stepped at %.2f s ===\n', F, T_STEP);
for k = 1:numel(R)
    if R(k).ok && ref.ok
        zi = interp1(R(k).t, R(k).z, ref.t, 'linear', 'extrap');
        R(k).err_mm = max(abs(zi - ref.z)) * 1000;
    end
    if R(k).ok
        fprintf('  %-32s runs   travel %6.2f mm   max err %7.4f mm   %6.1fx real time\n', ...
                R(k).name, (R(k).z(end) - R(k).z(1))*1000, R(k).err_mm, R(k).rtf);
    else
        fprintf('  %-32s FAILS  %s\n', R(k).name, R(k).msg(1:min(end,140)));
    end
end
if ref.ok
    kw = P.Susp.SpringRateFront * P.Susp.MotionRatioFront^2;
    fprintf('  static check: %g N on a %.0f N/m wheel rate is %.2f mm (linear; the rocker is regressive)\n', ...
            F, kw, F/kw*1000);
end

if withFMU
    R = fmu_leg(R, mdl, wd, STEP, T_END);
end
close_system(mdl, 0);
end

% -------------------------------------------------------------------------
function instrument(mdl, H, P, tStep, F)
load_system('sm_lib'); load_system('nesl_utility');
SD  = ['sm_lib/Forces and' newline 'Torques/Spring and' newline 'Damper Force'];
EF  = ['sm_lib/Forces and' newline 'Torques/External Force' newline 'and Torque'];
RT  = ['sm_lib/Frames and' newline 'Transforms/Rigid Transform'];
TS  = ['sm_lib/Frames and' newline 'Transforms/Transform Sensor'];
S2PS = ['nesl_utility/Simulink-PS' newline 'Converter'];
PS2S = ['nesl_utility/PS-Simulink' newline 'Converter'];

% Gravity off: static is then exactly the hardpoints.
set_param([mdl '/Mechanism'], 'GravityVector', '[0 0 0]');

% The car's spring and damper, on the damper line, unloaded at the hardpoints.
L0 = norm(H.rk_dmp - H.dmp_chassis);
cw = P.Derived.SuspensionDampingCoeff;                 % at the wheel
cd = cw / P.Susp.MotionRatioFront^2;                   % at the damper
add_block(SD, [mdl '/SpringDamper'], 'Position',[900 960 960 1010]);
set_param([mdl '/SpringDamper'], 'NaturalLength', num2str(L0,10), 'NaturalLengthUnits','m', ...
    'SpringStiffness', num2str(P.Susp.SpringRateFront,10), 'SpringStiffnessUnits','N/m', ...
    'DampingCoefficient', num2str(cd,10), 'DampingCoefficientUnits','N/(m/s)');
add_line(mdl, 'mount_dmp/RConn1', 'SpringDamper/LConn1', 'autorouting','on');
add_line(mdl, 'arm_rk_dmp/RConn1', 'SpringDamper/RConn1', 'autorouting','on');

% The load: vertical, in the world frame, at the contact patch.
add_block(RT, [mdl '/to_contact'], 'Position',[760 300 820 350]);
set_param([mdl '/to_contact'], 'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset', sprintf('[0 0 %.10g]', -H.wheel_radius), 'TranslationCartesianOffsetUnits','m');
add_line(mdl, 'Upright/RConn1', 'to_contact/LConn1', 'autorouting','on');
add_block(EF, [mdl '/Load'], 'Position',[900 300 960 350]);
set_param([mdl '/Load'], 'EnableForceZ','on', 'ForceResolutionFrame','World');
add_line(mdl, 'to_contact/RConn1', 'Load/RConn1', 'autorouting','on');
add_block('simulink/Sources/In1', [mdl '/Fz'], 'Position',[640 210 670 230]);
add_block(S2PS, [mdl '/Fz_ps'], 'Position',[720 205 760 235]);
set_param([mdl '/Fz_ps'], 'Unit','N');
add_line(mdl, 'Fz/1', 'Fz_ps/1');
ph = get_param([mdl '/Load'],'PortHandles');  pf = get_param([mdl '/Fz_ps'],'PortHandles');
add_line(mdl, pf.RConn(1), ph.LConn(1));
% The input, as data the FMU leg can reuse: a step from 0 to F.
t = (0:1e-3:2)';  u = F * (t >= tStep);
assignin('base', 'PROBE_u', [t u]);
set_param(mdl, 'LoadExternalInput','on', 'ExternalInput','PROBE_u');

% Out: the wheel centre's height.
add_block(TS, [mdl '/zSense'], 'Position',[900 420 960 470]);
set_param([mdl '/zSense'], 'MeasurementFrame','World', 'SenseZ','on');
add_line(mdl, 'World/RConn1', 'zSense/LConn1', 'autorouting','on');
add_line(mdl, 'Upright/RConn1', 'zSense/RConn1', 'autorouting','on');
add_block(PS2S, [mdl '/z_s'], 'Position',[1000 420 1040 450]);
set_param([mdl '/z_s'], 'Unit','m');
pz = get_param([mdl '/zSense'],'PortHandles');  pzs = get_param([mdl '/z_s'],'PortHandles');
add_line(mdl, pz.RConn(2), pzs.LConn(1));
add_block('simulink/Sinks/Out1', [mdl '/z'], 'Position',[1080 425 1110 445]);
add_line(mdl, 'z_s/1', 'z/1');
add_block('simulink/Sinks/To Workspace', [mdl '/zlog'], 'VariableName','PROBE_z', ...
          'SaveFormat','Timeseries', 'Position',[1080 480 1140 500]);
add_line(mdl, 'z_s/1', 'zlog/1');
end

function R = fmu_leg(R, mdl, wd, STEP, T_END)
% Export the PLANT's configuration if it runs, else the best fixed-step one.
cand = find([R.ok] & ~strcmp({R.name}, R(1).name));
if isempty(cand)
    fprintf('\n  no fixed-step configuration ran, so there is nothing to export.\n');
    return
end
% The plant's own configuration if it ran; otherwise the MOST ACCURATE
% fixed-step one -- exporting the cheapest would test the wrong thing.
if R(4).ok, pick = 4; else, [~, j] = min([R(cand).err_mm]); pick = cand(j); end
fprintf('\n=== FMU: exporting "%s" ===\n', R(pick).name);
switch pick
    case 2, set_param(mdl,'SolverType','Fixed-step','Solver','ode14x','FixedStep',STEP); set_param([mdl '/Solver'],'UseLocalSolver','off');
    case 3, set_param(mdl,'SolverType','Fixed-step','Solver','ode1be','FixedStep',STEP); set_param([mdl '/Solver'],'UseLocalSolver','off');
    case 4, set_param(mdl,'SolverType','Fixed-step','Solver','ode1','FixedStep',STEP);
            set_param([mdl '/Solver'],'UseLocalSolver','on','LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER', ...
                      'LocalSolverSampleTime',STEP,'DoFixedCost','on');
end
set_param(mdl, 'LoadExternalInput','off');             % the FMU takes Fz as its input
delete_block([mdl '/zlog']);
save_system(mdl);
fdir = fullfile(wd, 'fmu');  if ~isfolder(fdir), mkdir(fdir); end
prev = fullfile(fdir, [mdl '.fmu']);  if isfile(prev), delete(prev); end
here0 = pwd;  back = onCleanup(@() cd(here0));  cd(wd);
e = Simulink.FMUExporter(mdl);
for o = {'SaveDirectory',fdir; 'FMUName',mdl; 'FMIVersion','3.0'; 'FMUType','CS'; ...
         'CreateModelAfterGeneratingFMU','off'; 'AddIcon','off'}'
    try, e.Options.(o{1}) = o{2}; catch, end
end
try
    tic; evalc('e.export();'); tex = toc;
catch ME
    fprintf('  EXPORT FAILS: %s\n', strtrim(regexprep(ME.message,'\s+',' ')));
    R(end+1) = struct('name','FMU','ok',false,'t',[],'z',[],'wall',NaN,'rtf',NaN,'err_mm',NaN, ...
                      'msg',ME.message);
    return
end
d = dir(fullfile(fdir,'*.fmu'));
fprintf('  exported %s (%.0f kB) in %.0f s\n', d(1).name, d(1).bytes/1024, tex);

% Run it back: an FMU Import block driven by the same step.
h = 'IFSSIM_SM_Probe_FMUrun';
if bdIsLoaded(h), close_system(h,0); end
new_system(h);  set_param(h,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',STEP, ...
                          'StopTime',num2str(T_END));
add_block('simulink/Sources/From Workspace', [h '/u'], 'VariableName','PROBE_u', 'Position',[40 50 100 80]);
add_block('simulink_extras/FMU Import/FMU', [h '/FMU'], 'Position',[180 40 300 100]);
addpath(fdir);                                         % the block takes a NAME on the path, not a path
set_param([h '/FMU'], 'FMUName', d(1).name);
add_block('simulink/Sinks/To Workspace', [h '/z'], 'VariableName','PROBE_zf', 'SaveFormat','Timeseries', ...
          'Position',[380 50 440 80]);
add_line(h, 'u/1', 'FMU/1');  add_line(h, 'FMU/1', 'z/1');
r = struct('name','FMU (exported, run back)','ok',false,'t',[],'z',[],'wall',NaN,'rtf',NaN,'err_mm',NaN,'msg','');
try
    evalc('sim(h);');
    tic; out = sim(h); r.wall = toc;
    zz = out.get('PROBE_zf');  r.t = zz.Time;  r.z = squeeze(zz.Data);
    r.ok = all(isfinite(r.z));  r.rtf = T_END / r.wall;
    zi = interp1(R(pick).t, R(pick).z, r.t, 'linear', 'extrap');
    r.err_mm = max(abs(r.z - zi)) * 1000;       % against the model it came from
    zr = interp1(R(1).t, R(1).z, r.t, 'linear', 'extrap');
    fprintf('  FMU runs: travel %.2f mm, %.4f mm from its source model, %.4f mm from the reference, %.1fx real time\n', ...
            (r.z(end)-r.z(1))*1000, r.err_mm, max(abs(r.z - zr))*1000, r.rtf);
catch ME
    r.msg = strtrim(regexprep(ME.message,'\s+',' '));
    fprintf('  FMU FAILS TO RUN: %s\n', r.msg(1:min(end,200)));
end
R(end+1) = r;
close_system(h, 0);
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
