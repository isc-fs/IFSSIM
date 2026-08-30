function figs = vd_plots(M, label, outdir)
%VD_PLOTS  The standard vehicle-dynamics figures.
%
%   figs = VD_PLOTS(M) draws the five plots a suspension engineer reads, from
%   the design model. Called with no arguments it uses the car as built.
%
%   Each figure is also written to matlab/vd/figures as a PNG, so a run
%   leaves something you can put in a report or a slide without screenshotting
%   a MATLAB window.

if nargin < 1 || isempty(M)
    here = fileparts(mfilename('fullpath'));
    addpath(here); addpath(fullfile(here,'..','plant'));
    M = dualtrack_build();
end
if nargin < 2 || isempty(label),  label  = 'as built'; end
if nargin < 3 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'figures');
end
if ~isfolder(outdir), mkdir(outdir); end
figs = struct();

%% 1. The understeer plot. Steering to hold a circle, against lateral g.
% The classical picture: a flat line is neutral, rising is understeer, falling
% is oversteer, and the intercept is the Ackermann angle geometry alone needs.
figs.understeer = figure('Name','Understeer','Color','w');
hold on; grid on;
cols = lines(3); k = 0;
for R = [9.125 15 25]
    k = k + 1;
    C = vd_constant_radius(R, M, 4:0.5:20);
    if isempty(C.v), continue; end
    plot(C.ay/9.81, C.delta*180/pi, '-', 'LineWidth', 2, 'Color', cols(k,:), ...
         'DisplayName', sprintf('R = %.2f m  (K = %+.2f deg/g)', R, C.K));
    yline(C.ackermann*180/pi, ':', 'Color', cols(k,:), 'HandleVisibility','off');
end
xlabel('lateral acceleration  [g]');
ylabel('steer angle at the road wheel  [deg]');
title(sprintf('Understeer — %s', label));
legend('Location','best'); 
text(0.02, 0.04, 'dotted = Ackermann angle (pure geometry)', ...
     'Units','normalized','FontSize',8,'Color',[.4 .4 .4]);
savepng(figs.understeer, outdir, 'understeer');

%% 2. The tyre. Everything else in this model is downstream of this curve.
figs.tyre = figure('Name','Tyre','Color','w');
hold on; grid on;
a = linspace(0, 20*pi/180, 200);
for Fz = [0.5 0.75 1.0 1.5 2.0]*M.Fz0
    Fy = arrayfun(@(x) mf_lat(x, Fz, M), a);
    plot(a*180/pi, Fy, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Fz = %.0f N  (mu = %.2f)', Fz, ...
                                M.PDY1 + M.PDY2*(Fz-M.Fz0)/M.Fz0));
end
xlabel('slip angle  [deg]'); ylabel('lateral force  [N]');
title(sprintf('Tyre, pure lateral slip — %s', label));
legend('Location','southeast');
text(0.02, 0.95, 'peak mu falls as load rises: that is load sensitivity (PDY2)', ...
     'Units','normalized','FontSize',8,'Color',[.4 .4 .4]);
savepng(figs.tyre, outdir, 'tyre');

%% 3. The setup lever, and the only one this model has.
figs.balance = figure('Name','Balance','Color','w');
B = vd_balance_sweep(M);
yyaxis left
plot(100*B.frac, B.K, '-o', 'LineWidth', 2); grid on;
ylabel('understeer gradient K  [deg/g]');
yline(0, 'k:', 'neutral', 'HandleVisibility','off');
yyaxis right
plot(100*B.frac, B.skidpad, '-s', 'LineWidth', 1.5);
ylabel('skid pad lap time  [s]');
xlabel('front share of roll stiffness  [%]');
title(sprintf('Roll stiffness distribution — %s', label));
xline(100*M.KrF/(M.KrF+M.KrR), 'k--', 'as built', 'HandleVisibility','off');
savepng(figs.balance, outdir, 'balance');

%% 4. Transient response, stepped to a common lateral acceleration.
figs.step = figure('Name','Step steer','Color','w');
hold on; grid on;
for v = [8 12 16]
    d = steer_for_ay(v, 0.4*9.81, M);
    if isnan(d), continue; end
    S = vd_step_steer(v, d, M);
    plot(S.t - 0.5, S.r, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('%.0f m/s  (%.2f deg, t90 = %.3f s)', v, d*180/pi, S.t90));
end
xlim([-0.1 1.5]);
xlabel('time from the step  [s]'); ylabel('yaw rate  [rad/s]');
title(sprintf('Step to 0.4 g — %s', label));
legend('Location','southeast');
savepng(figs.step, outdir, 'step_steer');

%% 5. Where the load goes. This is what the whole suspension argument is about.
figs.load = figure('Name','Load transfer','Color','w');
hold on; grid on;
C = vd_constant_radius(9.125, M, 4:0.5:20);
names = {'front outer','front inner','rear outer','rear inner'};
idx   = [2 1 4 3];                        % right wheels are outer in a left turn
Fz = zeros(numel(C.v),4);
for i = 1:numel(C.v)
    S = dualtrack_trim(C.v(i), C.delta(i), M);
    Fz(i,:) = S.Fz;
end
for j = 1:4
    plot(C.ay/9.81, Fz(:,idx(j)), 'LineWidth', 1.8, 'DisplayName', names{j});
end
xlabel('lateral acceleration  [g]'); ylabel('vertical load  [N]');
title(sprintf('Corner loads on a 9.125 m circle — %s', label));
legend('Location','best');
text(0.02, 0.06, 'an inner load reaching zero is a wheel lifting', ...
     'Units','normalized','FontSize',8,'Color',[.4 .4 .4]);
savepng(figs.load, outdir, 'load_transfer');

fprintf('  figures written to %s\n', outdir);
end

% -----------------------------------------------------------------------
function savepng(f, outdir, name)
try
    exportgraphics(f, fullfile(outdir, [name '.png']), 'Resolution', 150);
catch
    saveas(f, fullfile(outdir, [name '.png']));
end
end

function Fy = mf_lat(alpha, Fz, M)
dfz = (Fz - M.Fz0)/M.Fz0;
mu  = M.PDY1 + M.PDY2*dfz;
D   = mu*Fz;  C = M.PCY1;  E = M.PEY1;
K   = abs(M.PKY1)*M.Fz0*sin(M.PKY4*atan(Fz/(M.PKY2*M.Fz0)));
B   = K/(C*D);
Fy  = D*sin(C*atan(B*alpha - E*(B*alpha - atan(B*alpha))));
end

function d = steer_for_ay(v, ayWant, M)
d = NaN;  lo = 1e-4;  alo = trimay(lo, v, M);
for hi = linspace(0.01, M.maxSteer, 60)
    ahi = trimay(hi, v, M);
    if ahi >= ayWant
        for it = 1:40
            mid = 0.5*(lo+hi);
            if trimay(mid, v, M) < ayWant, lo = mid; else, hi = mid; end
        end
        d = 0.5*(lo+hi); return;
    end
    if ahi < alo, return; end
    lo = hi;  alo = ahi;
end
end

function a = trimay(d, v, M)
S = dualtrack_trim(v, d, M);  a = S.ay;
end
