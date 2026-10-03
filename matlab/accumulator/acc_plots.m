function acc_plots(K, Kb, labels, outdir)
%ACC_PLOTS  The accumulator, as figures. Saved to accumulator/figures/.
%
%   ACC_PLOTS()                    the car as specified
%   ACC_PLOTS(K, Kb, labels, dir)  two acc_kpis results overlaid (acc_study)
%
%   1. one lap's current, cell by cell, against the sustained rating and the
%      highest characterised rate -- where on the track the cells are pushed.
%   2. state of charge and lowest cell voltage across the endurance, against
%      the floor.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(K), K = acc_kpis(); end
if nargin < 2, Kb = []; end
if nargin < 3 || isempty(labels), labels = {'as specified', 'study'}; end
if nargin < 4 || isempty(outdir), outdir = fullfile(here, 'figures'); end
if ~isfolder(outdir), mkdir(outdir); end
blue = [0 0.35 0.65];  amber = [0.85 0.55 0];

% ---- 1. one lap's cell current --------------------------------------------
f = figure('Visible','off','Position',[100 100 1000 420]); hold on; grid on; box on;
[s, Ic] = lap_current(K);
plot(s, Ic, '-', 'Color', blue, 'LineWidth', 1.2, 'DisplayName', labels{1});
if ~isempty(Kb), [sb, Icb] = lap_current(Kb); plot(sb, Icb, '-', 'Color', amber, 'LineWidth', 1.2, 'DisplayName', labels{2}); end
yline(K.P.Cell.ISustained, '--', 'sustained', 'HandleVisibility','off');
yline(K.P.Cell.ICharacterised, ':', 'highest characterised', 'HandleVisibility','off');
yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off');
xlabel('distance [m]'); ylabel('current per cell [A]  (negative = regen)');
title(sprintf('One flat-out lap at %.0f%% charge: rms %.1f A per cell', 100*K.PK.SoC0, K.I_cell_rms));
legend('Location','southoutside','Orientation','horizontal');
exportgraphics(f, fullfile(outdir,'acc_lap_current.png'), 'Resolution', 150); close(f);

% ---- 2. across the endurance --------------------------------------------
f = figure('Visible','off','Position',[100 100 1000 420]);
subplot(1,2,1); hold on; grid on; box on;
[lap, soc, vmin] = march(K);
plot(lap, 100*soc, '-', 'Color', blue, 'LineWidth', 2, 'DisplayName', labels{1});
if ~isempty(Kb), [lb, sb2, vb] = march(Kb); plot(lb, 100*sb2, '-', 'Color', amber, 'LineWidth', 2, 'DisplayName', labels{2}); end
xline(K.R.laps_needed, ':', 'flag', 'HandleVisibility','off');
xlabel('lap'); ylabel('state of charge [%]'); title('Charge across the endurance');
legend('Location','northeast');
subplot(1,2,2); hold on; grid on; box on;
plot(lap, vmin, '-', 'Color', blue, 'LineWidth', 2, 'DisplayName', labels{1});
if ~isempty(Kb), plot(lb, vb, '-', 'Color', amber, 'LineWidth', 2, 'DisplayName', labels{2}); end
yline(K.P.Cell.VMin, '--', 'cell floor', 'HandleVisibility','off');
xline(K.R.laps_needed, ':', 'flag', 'HandleVisibility','off');
xlabel('lap'); ylabel('lowest cell voltage in the lap [V]'); title('Sag against the floor');
exportgraphics(f, fullfile(outdir,'acc_endurance.png'), 'Resolution', 150); close(f);
fprintf('  figures written to %s\n', outdir);
end

function [s, Ic] = lap_current(K)
L = lap_sim(lap_ggv(K.P, K.PK.SoC0));
Voc = K.PK.OCV(K.PK.SoC0);
I = (Voc - sqrt(max(Voc^2 - 4*K.PK.Rint*L.P_elec, 0))) / (2*K.PK.Rint);
s = L.s;  Ic = I / K.PK.Np;
end

function [lap, soc, vmin] = march(K)
% The same lap-by-lap march acc_endurance makes, kept for plotting.
pl = K.R.per_lap;
f = @(fld, x) interp1([pl.soc], [pl.(fld)], x, 'linear', 'extrap');
lap = 0;  soc = K.PK.SoC0;  vmin = f('Vmin', soc);
while lap(end) < K.R.laps - 1e-9
    fr = min(1, K.R.laps - lap(end));
    soc(end+1)  = soc(end) - fr*f('Q_Ah', soc(end))/K.PK.CapacityAh; %#ok<AGROW>
    vmin(end+1) = f('Vmin', soc(end)); %#ok<AGROW>
    lap(end+1)  = lap(end) + fr; %#ok<AGROW>
end
end
