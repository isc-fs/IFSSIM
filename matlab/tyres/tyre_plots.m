function tyre_plots(K, Kb, labels, outdir)
%TYRE_PLOTS  The tyre curves, as figures. Saved to tyres/figures/.
%
%   TYRE_PLOTS()                    the car as specified
%   TYRE_PLOTS(K, Kb, labels, dir)  two tyre_kpis results overlaid (tyre_study)
%
%   1. lateral force against slip angle at four loads -- the curve the plant
%      and the design model both run.
%   2. peak friction and cornering stiffness against load: load sensitivity
%      is the slope of the first, and it is what load transfer costs.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(K), K = tyre_kpis(); end
if nargin < 2, Kb = []; end
if nargin < 3 || isempty(labels), labels = {'as specified', 'study'}; end
if nargin < 4 || isempty(outdir), outdir = fullfile(here, 'figures'); end
if ~isfolder(outdir), mkdir(outdir); end
blue = [0 0.35 0.65];  amber = [0.85 0.55 0];

% ---- 1. Fy against slip angle ---------------------------------------------
f = figure('Visible','off','Position',[100 100 800 480]); hold on; grid on; box on;
a = linspace(0, 20, 400) * pi/180;
loads = [0.5 1 1.5 2];
shade = linspace(0.35, 1, numel(loads));
for k = 1:numel(loads)
    plot(a*180/pi, fy(K.M, a, loads(k)), '-', 'Color', blue*shade(k) + (1-shade(k)), ...
         'LineWidth', 1.8, 'DisplayName', sprintf('%.1f Fz0, %s', loads(k), labels{1}));
    if ~isempty(Kb)
        plot(a*180/pi, fy(Kb.M, a, loads(k)), '--', 'Color', amber*shade(k) + (1-shade(k)), ...
             'LineWidth', 1.4, 'DisplayName', sprintf('%.1f Fz0, %s', loads(k), labels{2}));
    end
end
xlabel('slip angle [deg]'); ylabel('lateral force [N]');
title(sprintf('Lateral force: peak at %.1f deg, %.0f%% left once sliding', K.alpha_peak, 100*K.tail_lat));
legend('Location','southeast','NumColumns', 1 + ~isempty(Kb));
exportgraphics(f, fullfile(outdir,'tyre_fy.png'), 'Resolution', 150); close(f);

% ---- 2. against load -------------------------------------------------------
f = figure('Visible','off','Position',[100 100 1000 420]);
x = linspace(0.2, 2.2, 100);
subplot(1,2,1); hold on; grid on; box on;
plot(x, K.M.PDY1 + K.M.PDY2*(x-1), 'Color', blue, 'LineWidth', 2, 'DisplayName', labels{1});
if ~isempty(Kb), plot(x, Kb.M.PDY1 + Kb.M.PDY2*(x-1), 'Color', amber, 'LineWidth', 2, 'DisplayName', labels{2}); end
xline(1, ':', 'nominal', 'HandleVisibility','off');
xlabel('load / nominal'); ylabel('peak friction');
title('Load sensitivity: what transfer costs'); legend('Location','northeast');
subplot(1,2,2); hold on; grid on; box on;
plot(x, ca(K.M, x), 'Color', blue, 'LineWidth', 2, 'DisplayName', labels{1});
if ~isempty(Kb), plot(x, ca(Kb.M, x), 'Color', amber, 'LineWidth', 2, 'DisplayName', labels{2}); end
xline(1, ':', 'nominal', 'HandleVisibility','off');
xlabel('load / nominal'); ylabel('cornering stiffness [N/deg]');
title('Cornering stiffness against load'); legend('Location','southeast');
exportgraphics(f, fullfile(outdir,'tyre_load.png'), 'Resolution', 150); close(f);
fprintf('  figures written to %s\n', outdir);
end

function F = fy(M, a, frac)
Fz = frac*M.Fz0;
D = (M.PDY1 + M.PDY2*(frac-1)) * Fz;
Kya = -M.PKY1*M.Fz0*sin(M.PKY4*atan(Fz/(M.PKY2*M.Fz0)));
B = Kya/(M.PCY1*D);  E = M.PEY1;
F = D*sin(M.PCY1*atan(B*a - E*(B*a - atan(B*a))));
end

function c = ca(M, x)
c = -M.PKY1*M.Fz0*sin(M.PKY4*atan(x/M.PKY2)) * pi/180;
end
