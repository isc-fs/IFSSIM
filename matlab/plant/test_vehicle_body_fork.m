function ok = test_vehicle_body_fork()
%TEST_VEHICLE_BODY_FORK  The gate on step 4a: does the fork actually reset?
%
%   Step 4a of docs/vdb_plant_migration.md. The forked Vehicle Body 6DOF must
%   honour the same contract IFSSIM_Chassis honours today: when sync.enable
%   goes high, the body's state becomes EXACTLY the pose the platform asked
%   for. That is how the car gets put on the start gate. If this does not
%   hold, step 4b must not proceed -- a body that cannot be positioned is
%   not usable however good its dynamics are.
%
%   The test drives the body with a force so its state is genuinely moving,
%   then injects a pose that has nothing to do with where it was heading, and
%   checks the state lands on the injected value rather than near it.

here = fileparts(mfilename('fullpath'));
addpath(here);
ok = true;
fprintf('\n=== vehicle body fork: state injection ===\n');

h = 'vb_fork_harness';
if bdIsLoaded(h), close_system(h,0); end
new_system(h,'Model');
set_param(h,'SolverType','Fixed-step','Solver','ode1','FixedStep','0.001', ...
            'StartTime','0','StopTime','2.0','SaveFormat','Dataset');

info = fork_vehicle_body(h, [h '/Body'], [300 100 460 300]);

% Drive it with a steady force and moment so nothing is sitting still.
zero3 = {'FSusp','zeros(3,4)'; 'MSusp','zeros(3,4)'};
for k = 1:2
    add_block('simulink/Sources/Constant',[h '/' zero3{k,1}], ...
              'Value',zero3{k,2},'Position',[150 100+40*k 220 116+40*k]);
    add_line(h,[zero3{k,1} '/1'],sprintf('Body/%d',k),'autorouting','on');
end
add_block('simulink/Sources/Constant',[h '/FExt'],'Value','[400;150;0]', ...
          'Position',[150 200 220 216]);
add_block('simulink/Sources/Constant',[h '/MExt'],'Value','[0;0;60]', ...
          'Position',[150 240 220 256]);
add_block('simulink/Sources/Constant',[h '/Wind'],'Value','[0;0;0]', ...
          'Position',[150 280 220 296]);
add_line(h,'FExt/1','Body/3','autorouting','on');
add_line(h,'MExt/1','Body/4','autorouting','on');
add_line(h,'Wind/1','Body/5','autorouting','on');

% The injection: a step at t = 1 s into the trigger Goto, and constants into
% the four state Gotos. The pose is deliberately unrelated to the trajectory.
TGT.euler = [0.10; -0.20;  1.30];
TGT.pqr   = [0.30;  0.40; -0.50];
TGT.vb    = [7.00; -1.50;  0.25];
TGT.xe    = [123.0; -45.0;  6.0];
add_block('simulink/Sources/Step',[h '/trig'],'Time','1.0','Before','0','After','1', ...
          'Position',[60 60 130 76]);
add_line(h,'trig/1','IFSSIM_SYNC_TRIG_goto/1','autorouting','on');
src = {'IFSSIM_SYNC_EULER',TGT.euler; 'IFSSIM_SYNC_PQR',TGT.pqr
       'IFSSIM_SYNC_VB',   TGT.vb;    'IFSSIM_SYNC_XE', TGT.xe
       'IFSSIM_SYNC_ACC',  0};
for k = 1:size(src,1)
    add_block('simulink/Sources/Constant',[h '/ic' num2str(k)], ...
              'Value',mat2str(src{k,2}),'Position',[60 100+40*k 130 116+40*k]);
    add_line(h,['ic' num2str(k) '/1'],[src{k,1} '_goto/1'],'autorouting','on');
end

% Log the states we are asserting on.
outs = {'Vb',2; 'pqr',3; 'Euler',5; 'Xe',6};
for k = 1:size(outs,1)
    add_block('simulink/Sinks/To Workspace',[h '/log_' outs{k,1}], ...
              'VariableName',['log_' outs{k,1}],'SaveFormat','Timeseries', ...
              'Position',[560 80+50*k 630 110+50*k]);
    add_line(h,sprintf('Body/%d',outs{k,2}),['log_' outs{k,1} '/1'],'autorouting','on');
end
for k = [1 4 7]
    add_block('simulink/Sinks/Terminator',[h '/term' num2str(k)],'Position',[560 380+30*k 580 396+30*k]);
    add_line(h,sprintf('Body/%d',k),['term' num2str(k) '/1'],'autorouting','on');
end

r = sim(h);

% WHERE to sample is the whole difficulty, and getting it wrong looks exactly
% like a broken reset. The first version read one step PAST the edge, by which
% time the body had already integrated away from the injected value -- 7 m/s
% times 1 ms is the 0.007 m it was "failing" by. The state had jumped
% perfectly; the probe was late.
%
% So the assertion is the contract itself, with no index guessed: within one
% step of the trigger the state must equal the injected value EXACTLY. The
% search reports which step it landed on, so a reset that arrived late or
% approached gradually is still visible rather than absorbed.
tol = 1e-9;
ok = at(ok, r, 'log_Euler', TGT.euler, 'Euler angles',  tol);
ok = at(ok, r, 'log_pqr',   TGT.pqr,   'body rates',    tol);
ok = at(ok, r, 'log_Vb',    TGT.vb,    'body velocity', tol);
ok = at(ok, r, 'log_Xe',    TGT.xe,    'earth position',tol);

% And it must keep integrating afterwards, not stay latched at the injected
% value -- a reset that never releases is just as broken as one that never
% fires, and it would pass every check above.
moved = moved_after(r,'log_Xe');
fprintf('  [%s] %-34s moved %.4f m in the 0.5 s after the reset\n', ...
        tern(moved > 1e-6,'ok  ','FAIL'), 'releases and keeps integrating', moved);
if moved <= 1e-6, ok = false; end

close_system(h,0);
fprintf('\n  %s\n', tern(ok, ...
    'The fork honours the injection contract. Step 4b may proceed.', ...
    'THE FORK DOES NOT RESET. Step 4b must NOT proceed.'));
end

% -------------------------------------------------------------------------
function ok = at(ok, r, nm, want, label, tol)
ts = r.get(nm);  t = ts.Time;  D = ts.Data;
i0 = find(t >= 1.0 - 1e-12, 1, 'first');
best = inf; bestk = i0;
for k = i0:min(i0+2, numel(t))          % the edge step, or one either side
    d = max(abs(squeeze(D(k,:))' - want(:)));
    if d < best, best = d; bestk = k; end
end
lag  = t(bestk) - 1.0;
pass = (best <= tol) && (lag <= 1.5e-3);
fprintf('  [%s] %-24s err %.3e at t=1%+.4f s\n', ...
        tern(pass,'ok  ','FAIL'), label, best, lag);
if ~pass
    fprintf('         got  %s\n         want %s\n', ...
            mat2str(squeeze(D(bestk,:)),6), mat2str(want(:)',6));
    ok = false;
end
end

function d = moved_after(r, nm)
ts = r.get(nm); t = ts.Time; D = ts.Data;
i = find(t >= 1.0 - 1e-12, 1, 'first');
j = find(t >= 1.5, 1, 'first');
d = norm(squeeze(D(j,:)) - squeeze(D(i,:)));
end

function s = tern(c,a,b)
if c, s = a; else, s = b; end
end
