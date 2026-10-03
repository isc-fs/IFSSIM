function aero_plots(K, Kb, labels, outdir, withMap)
%AERO_PLOTS  The aero, as figures. Saved to aero/figures/.
%
%   AERO_PLOTS()                          the car as specified, with the trade map
%   AERO_PLOTS(K, Kb, labels, dir, map)   two aero_kpis results overlaid (aero_study)
%
%   1. the lap: speed against distance, coloured by what limits it there.
%   2. cornering grip against speed: what four tyres pooled could give, what
%      yaw balance lets the car use, and which axle runs out first.
%   3. the trade map: lap time over (ClA, CdA), the current package marked.
%      ~25 laps, so about half a minute; aero_study skips it.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(K), K = aero_kpis(); end
if nargin < 2, Kb = []; end
if nargin < 3 || isempty(labels), labels = {'as specified', 'study'}; end
if nargin < 4 || isempty(outdir), outdir = fullfile(here, 'figures'); end
if nargin < 5, withMap = isempty(Kb); end
if ~isfolder(outdir), mkdir(outdir); end
blue = [0 0.35 0.65];  amber = [0.85 0.55 0];

% ---- 1. the lap ----------------------------------------------------------
L = K.L;
f = figure('Visible','off','Position',[100 100 1000 420]); hold on; grid on; box on;
cols = struct('corner',[0.75 0.30 0.10], 'accel',blue, 'brake',[0.45 0.45 0.45]);
for key = ["corner" "accel" "brake"]
    vv = 3.6*L.v;  vv(L.limit ~= key) = NaN;
    plot(L.s, vv, '.', 'Color', cols.(key), 'MarkerSize', 4, 'DisplayName', key + "-limited");
end
if ~isempty(Kb)
    plot(Kb.L.s, 3.6*Kb.L.v, '-', 'Color', amber, 'LineWidth', 1.2, 'DisplayName', labels{2});
end
xlabel('distance [m]'); ylabel('speed [km/h]');
title(sprintf('Lap: %.2f s, mean %.1f km/h (%s)', K.lap, K.lap_mean_kmh, labels{1}));
legend('Location','southoutside','Orientation','horizontal');
exportgraphics(f, fullfile(outdir,'aero_lap.png'), 'Resolution', 150); close(f);

% ---- 2. cornering grip against speed --------------------------------------
G = K.G;  B = G.balance;  g = 9.81;
f = figure('Visible','off','Position',[100 100 1000 420]);
subplot(1,2,1); hold on; grid on; box on;
plot(G.v, G.ay0_pooled/g, '--', 'Color', blue, 'LineWidth', 1.2, 'DisplayName', 'four tyres pooled');
plot(G.v, G.ay0/g, '-', 'Color', blue, 'LineWidth', 2, 'DisplayName', ['yaw-balanced, ' labels{1}]);
if ~isempty(Kb), plot(Kb.G.v, Kb.G.ay0/g, '-', 'Color', amber, 'LineWidth', 2, 'DisplayName', ['yaw-balanced, ' labels{2}]); end
yline(K.G.P.TireMu, ':', 'mu, no aero', 'HandleVisibility','off');
xlim([3 35]); xlabel('speed [m/s]'); ylabel('lateral grip [g]');
title('Cornering limit'); legend('Location','northwest');
subplot(1,2,2); hold on; grid on; box on;
plot(B.v, B.ratio, '-', 'Color', blue, 'LineWidth', 2, 'DisplayName', labels{1});
if ~isempty(Kb), plot(Kb.G.balance.v, Kb.G.balance.ratio, '-', 'Color', amber, 'LineWidth', 2, 'DisplayName', labels{2}); end
yline(1, ':', 'neutral', 'HandleVisibility','off');
xlim([3 35]); xlabel('speed [m/s]'); ylabel('front / rear axle capacity');
title('Limit balance: above 1 the rear goes first'); legend('Location','northwest');
exportgraphics(f, fullfile(outdir,'aero_balance.png'), 'Resolution', 150); close(f);

% ---- 3. the trade map -----------------------------------------------------
if withMap
    P = K.G.P;
    cl = P.ClA * linspace(0.6, 1.4, 5);
    cd = P.CdA * linspace(0.6, 1.4, 5);
    T = zeros(numel(cd), numel(cl));
    for i = 1:numel(cd)
        for j = 1:numel(cl)
            T(i,j) = lap_sim(lap_ggv(ifssim_params({'ClA', cl(j), 'CdA', cd(i)}))).time;
        end
    end
    f = figure('Visible','off','Position',[100 100 700 560]); hold on; box on;
    [c, h] = contourf(cl, cd, T, 12); h.LabelFormat = '%.2f s'; clabel(c, h, 'Color', [0.15 0.15 0.15]);
    colormap(flipud(parula)); cb = colorbar; cb.Label.String = 'lap time [s]';
    plot(P.ClA, P.CdA, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'w');
    text(P.ClA, P.CdA, '  as specified', 'VerticalAlignment', 'bottom');
    xlabel('ClA [m^2]'); ylabel('CdA [m^2]');
    title('Lap time over the aero package: contours run nearly vertical');
    exportgraphics(f, fullfile(outdir,'aero_trade.png'), 'Resolution', 150); close(f);
end
fprintf('  figures written to %s\n', outdir);
end
