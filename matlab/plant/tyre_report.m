function R = tyre_report(savePng)
%TYRE_REPORT  Plot what the tyre actually does, and every constraint behind it.
%
%   TYRE_REPORT sweeps the REAL Combined Slip Wheel 2DOF block -- configured by
%   the same CONFIGURE_TYRE_BLOCK the plant uses, so this cannot validate a
%   tyre the car does not have -- and plots it against the Magic Formula curve
%   written in settings.json.
%
%   It exists because the claims made about this tyre ("the tail is 48% of
%   peak", "Fx tracks to 6% up to the peak") are only worth anything if you
%   can re-run them and disagree. Every operating condition the sweeps impose
%   is listed on the figure and printed to the console, because those
%   conditions are where the conclusions actually come from.
%
%   TYRE_REPORT(true) also writes tyre_report.png next to the models.
%
%   See also BUILD_TYRE_PARAMSET, CONFIGURE_TYRE_BLOCK, IFSSIM_PARAMS_REPORT.

if nargin < 1, savePng = false; end
here   = fileparts(mfilename('fullpath'));
outdir = fullfile(here,'models');
addpath(here); addpath(outdir);

P  = ifssim_load_workspace();
TP = build_tyre_paramset(outdir);

Fz   = P.Derived.NominalWheelLoad;
Vx   = 10;
Rw   = P.WheelRadius;
Tset = 0.35;            % long enough for relaxation to settle: sigma/Vx = 0.03 s

%% ---- what is being assumed, and where each number came from -----------
% The coefficients as actually written to the block, so the table below
% reports what the tyre IS rather than what this file thinks it should be.
TPcs = load(fullfile(fileparts(mfilename('fullpath')),'models','ifssim_tyre.mat'));
TPcs = TPcs.ifssim_tyre;
prov = {
 'peak friction mu',        sprintf('%.2f',P.TireMu),                    'settings.json'
 'lateral shape  LatC',     sprintf('%.2f',P.Pacejka.LatC),              'settings.json'
 'lateral curvature LatE',  sprintf('%.2f',P.Pacejka.LatE),              'settings.json'
 'lateral stiffness LatB',  sprintf('%.2f',P.Pacejka.LatB),              'settings.json'
 'long. shape    LonC',     sprintf('%.3f',P.Pacejka.LonC),              'settings.json'
 'long. curvature LonE',    sprintf('%.2f',P.Pacejka.LonE),              'settings.json'
 'long. stiffness LonB',    sprintf('%.3f',P.Pacejka.LonB),              'settings.json'
 'wheel radius',            sprintf('%.3f m',Rw),                        'settings.json'
 'nominal wheel load',      sprintf('%.1f N',Fz),                        'DERIVED: static, mass*g/4'
 'cornering stiffness',     sprintf('%.0f N/rad',P.Derived.CorneringStiffness), 'DERIVED: mu*Fz*LatC*LatB'
 'slip stiffness',          sprintf('%.0f N',P.Derived.LongSlipStiffness),      'DERIVED: mu*Fz*LonC*LonB'
 'wheel inertia',           sprintf('%.2f kg m^2',P.Assumed.WheelInertia),'ASSUMED, never measured'
 'relaxation long/lat',     sprintf('%.2f / %.2f m',P.Assumed.RelaxLengthLong,P.Assumed.RelaxLengthLat),'ASSUMED, never measured'
 'tyre width',              sprintf('%.3f m',P.WheelWidth),        'ASSUMED, never measured'
 'load sensitivity PDY2',   '0',                                          'ZEROED: no data'
 'camber sensitivity',      '0',                                          'ZEROED: suspension has no camber DOF'
 'pressure sensitivity',    '0',                                          'ZEROED: runs at nominal pressure'
 'ply steer / turn slip',   'off',                                        'ZEROED: no data'
 'MF rolling resistance',   'QSY1..8 = 0',                                'ZEROED: plant applies its own Crr'
 'carcass vertical stiff.', '1e9 N/m (rigid)',                            'FORCED: keeps rolling radius = wheel radius'
 'friction vs slip speed',  sprintf('LONGVL %.0f m/s',P.Assumed.TyreRefVelocity), 'ZEROED: no data; LMUV is ignored by the block'
 'combined slip RBX1/RBY1', sprintf('%.2f / %.2f',TPcs.RBX1,TPcs.RBY1),   'DERIVED: friction circle, see fit_combined_slip'
 'Mx / My / Mz families',   'QSX, QSY, Q*Z = 0',                          'ZEROED: all three moments are terminated'
 'MF scaling factors L*',   '1',                                          'DERIVED: ours IS the fit, so nothing to scale'
 'anything not listed',     '0',                                          'STRIPPED: nothing is inherited, see build_tyre_paramset'
};

fprintf('\n=== TYRE: what the model is, and what it rests on ===\n');
fprintf('  %-24s %-22s %s\n','QUANTITY','VALUE','PROVENANCE');
for i = 1:size(prov,1)
    fprintf('  %-24s %-22s %s\n', prov{i,1}, prov{i,2}, prov{i,3});
end
fprintf('\n  NOTHING HERE WAS MEASURED ON THIS TYRE. The shape came from a fit\n');
fprintf('  nobody validated; the assumptions above are typical values. FSAE TTC\n');
fprintf('  data would replace the whole first block and most of the second.\n');

constraints = {
 sprintf('Fz held at %.0f N (static, per wheel): THE RIG has no load', Fz)
 'transfer. The PLANT does -- per corner, from suspension'
 'deflection; see the pitch/roll checks in test_tiresuspension'
 sprintf('Vx = %g m/s throughout; MF has speed-dependent terms', Vx)
 'camber = 0, turn slip off, ply steer off'
 'pressure = nominal, so every pressure term is inert'
 'carcass rigid, so rolling radius stays at the wheel radius'
 'WHEEL SPIN IS FROZEN (inertia 1e6) so slip is imposed, not settled'
 sprintf('each point run %.2f s to let relaxation settle', Tset)
};
fprintf('\n=== SWEEP CONDITIONS (this is where the conclusions come from) ===\n');
for i = 1:numel(constraints), fprintf('  - %s\n', constraints{i}); end

%% ---- sweeps against the real block -----------------------------------
mf = @(x,B,C,E) sin(C*atan(B*x - E*(B*x - atan(B*x))));

alpha = [0 1 2 3 4 5 6 7.5 9 10.5 12.5 15 17.5 20];        % deg
kappa = [0 .02 .04 .06 .08 .10 .13 .16 .20 .30 .45 .70 1.0];

fprintf('\nsweeping the block (%d lateral + %d longitudinal points)...\n', ...
        numel(alpha), numel(kappa));

Fy = zeros(size(alpha));
for i = 1:numel(alpha)
    Fy(i) = abs(rig(TP,P,Fz,Vx,Rw,Tset, Vx*tand(alpha(i)), 0, 'Fy'));
end
Fx = zeros(size(kappa));
for i = 1:numel(kappa)
    Fx(i) = abs(rig(TP,P,Fz,Vx,Rw,Tset, 0, kappa(i), 'Fx'));
end

Fy_fit = P.TireMu*Fz*mf(alpha*pi/180, P.Pacejka.LatB,P.Pacejka.LatC,P.Pacejka.LatE);
Fx_fit = P.TireMu*Fz*mf(kappa,        P.Pacejka.LonB,P.Pacejka.LonC,P.Pacejka.LonE);

R = struct('alpha_deg',alpha,'Fy_block',Fy,'Fy_fit',Fy_fit, ...
           'kappa',kappa,'Fx_block',Fx,'Fx_fit',Fx_fit, ...
           'Fz',Fz,'Vx',Vx,'constraints',{constraints},'provenance',{prov});

pk = P.TireMu*Fz;
fprintf('\n  %8s %10s %10s %8s   |  %7s %10s %10s %8s\n', ...
        'alpha','block','fit','diff','kappa','block','fit','diff');
n = max(numel(alpha),numel(kappa));
for i = 1:n
    la = ''; lk = '';
    if i<=numel(alpha), la = sprintf('%7.1f %10.1f %10.1f %7.1f%%', alpha(i), Fy(i), Fy_fit(i), 100*(Fy(i)-Fy_fit(i))/max(Fy_fit(i),1)); end
    if i<=numel(kappa), lk = sprintf('%7.2f %10.1f %10.1f %7.1f%%', kappa(i), Fx(i), Fx_fit(i), 100*(Fx(i)-Fx_fit(i))/max(Fx_fit(i),1)); end
    fprintf('  %-38s |  %s\n', la, lk);
end
fprintf('\n  peak available (mu*Fz)      %.0f N\n', pk);
fprintf('  lateral  : block peak %.0f N (%.0f%% of mu*Fz), tail at 20 deg %.0f%%\n', ...
        max(Fy), 100*max(Fy)/pk, 100*Fy(end)/pk);
fprintf('  long.    : block peak %.0f N (%.0f%% of mu*Fz), tail at kappa 1 %.0f%%\n', ...
        max(Fx), 100*max(Fx)/pk, 100*Fx(end)/pk);
fprintf('  a real slick is usually quoted holding 70-85%% at full slide.\n\n');

%% ---- figure -----------------------------------------------------------
f = figure('Name','IFSSIM tyre','Color','w','Position',[80 80 1180 720]);

subplot(2,2,1); hold on; grid on; box on;
plot(alpha, Fy_fit,'--','LineWidth',1.6,'DisplayName','settings.json curve');
plot(alpha, Fy,'-o','LineWidth',1.8,'MarkerSize',4,'DisplayName','MF 6.2 block');
yline(pk,':','mu*Fz','LabelHorizontalAlignment','left','HandleVisibility','off');
xlabel('slip angle  [deg]'); ylabel('|F_y|  [N]');
title(sprintf('Lateral, pure slip   (F_z = %.0f N, V_x = %g m/s)',Fz,Vx));
legend('Location','southeast'); ylim([0 pk*1.15]);

subplot(2,2,2); hold on; grid on; box on;
plot(kappa, Fx_fit,'--','LineWidth',1.6,'DisplayName','settings.json curve');
plot(kappa, Fx,'-o','LineWidth',1.8,'MarkerSize',4,'DisplayName','MF 6.2 block');
yline(pk,':','mu*Fz','LabelHorizontalAlignment','left','HandleVisibility','off');
xlabel('slip ratio  \kappa  [-]'); ylabel('|F_x|  [N]');
title('Longitudinal, pure slip');
legend('Location','northeast'); ylim([0 pk*1.15]);

subplot(2,2,3); hold on; grid on; box on;
% Points where the fit is near zero are dropped: at zero slip both curves are
% zero and the ratio is meaningless, not a 100% disagreement.
ia = Fy_fit > 0.02*pk;  ik = Fx_fit > 0.02*pk;
plot(alpha(ia),    100*Fy(ia)./Fy_fit(ia)-100,'-o','LineWidth',1.6,'MarkerSize',4,'DisplayName','lateral');
plot(kappa(ik)*20, 100*Fx(ik)./Fx_fit(ik)-100,'-s','LineWidth',1.6,'MarkerSize',4,'DisplayName','longitudinal');
yline(0,'k:','HandleVisibility','off');
xlabel('slip angle [deg]   /   slip ratio \times 20'); ylabel('block - fit   [%]');
title('Where the block departs from the fit'); legend('Location','southwest');

ax = subplot(2,2,4); axis(ax,'off');
txt = [{'\bfSWEEP CONDITIONS\rm'}; cellfun(@(c) ['  \bullet ' c], constraints,'uni',0); ...
       {''}; {'\bfThese conditions ARE the conclusions.\rm'}; ...
       {'  Change any of them and the numbers move.'}; ...
       {'  Nothing here was measured on this tyre.'}];
text(ax,0,1,txt,'VerticalAlignment','top','FontSize',9,'Interpreter','tex');

if savePng
    png = fullfile(outdir,'tyre_report.png');
    exportgraphics(f, png, 'Resolution',150);
    fprintf('wrote %s\n', png);
end
end

%% =======================================================================
function y = rig(TP, P, Fz, Vx, Rw, Tend, Vy, kap, want)
%RIG  One steady-state point off the real block, with the wheel held still.
%
%   Wheel spin is FROZEN by giving the wheel an enormous inertia, so omega
%   stays at whatever the slip ratio asks for and the tyre is measured at an
%   IMPOSED slip rather than wherever the wheel happens to settle. That is the
%   single biggest liberty this rig takes, and it is why these curves are
%   comparable to the algebraic fit at all.
nm = 'ifssim_tyre_rig';
if bdIsLoaded(nm), close_system(nm,0); end
new_system(nm);
set_param(nm,'SolverType','Fixed-step','FixedStep','1/960','Solver','ode1', ...
             'StopTime',num2str(Tend));
b = [nm '/W'];
add_block('vehdynlibtire/Combined Slip Wheel 2DOF', b);
configure_tyre_block(b, TP, struct('Radius',Rw,'Inertia',1e6, ...
                                   'Omega0',(1+kap)*Vx/Rw));
vals = {'0', num2str(Vx), num2str(Vy), '0', '0', ...
        num2str(P.Assumed.TyrePressure), num2str(Fz), '0', 'ones(27,1)'};
for k = 1:9
    add_block('simulink/Sources/Constant',[nm '/s' num2str(k)],'Value',vals{k});
    add_line(nm,['s' num2str(k) '/1'],['W/' num2str(k)]);
end
og = {'Info','Omega','Fx','Fy','Fz','Mx','My','Mz'};
for i = 1:8
    add_block('simulink/Sinks/To Workspace',[nm '/o' num2str(i)], ...
              'VariableName',['rig_' og{i}],'SaveFormat','Structure With Time');
    add_line(nm,['W/' num2str(i)],['o' num2str(i) '/1']);
end
r = sim(nm);
v = r.get(['rig_' want]).signals.values(:);
y = v(end);
close_system(nm,0);
end
