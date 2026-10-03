function pt_plots(E, Eb, labels, outdir)
%PT_PLOTS  The powertrain, as figures. Saved to pt/figures/.
%
%   PT_PLOTS()                    the car as specified
%   PT_PLOTS(E, Eb, labels, dir)  two pt_model results overlaid (pt_study)
%
%   1. force at the road against speed: what the motor could give, what the
%      tyres can take, what the car gets, and what it loses to the air and
%      the road. The shaded bands say which limit binds where -- which is
%      the whole department in one picture.
%   2. the quasi-steady acceleration run: speed against time and distance.
%   3. regen: the braking force the motor can give, against speed.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(E), E = pt_model(); end
if nargin < 2, Eb = []; end
if nargin < 3 || isempty(labels), labels = {'as specified', 'study'}; end
if nargin < 4 || isempty(outdir), outdir = fullfile(here, 'figures'); end
if ~isfolder(outdir), mkdir(outdir); end

vmax = min(E.v(end), 1.15 * max([E.v_top; field_or(Eb,'v_top',0)]));

% ---- 1. force against speed --------------------------------------------
f = figure('Visible','off','Position',[100 100 900 520]);
hold on; grid on; box on;
yl = [0, 1.1*max([E.F_motor(1); E.F_traction(1)])];
bands(E, yl, vmax);
plot(E.v, E.F_motor,    '-',  'Color',[0.35 0.35 0.35], 'LineWidth',1.2, 'DisplayName','motor + pack can give');
plot(E.v, E.F_traction, '--', 'Color',[0.35 0.35 0.35], 'LineWidth',1.2, 'DisplayName','rear tyres can take');
plot(E.v, E.F_drive,    '-',  'Color',[0.00 0.35 0.65], 'LineWidth',2.2, 'DisplayName',['drive, ' labels{1}]);
plot(E.v, E.F_resist,   '-',  'Color',[0.75 0.30 0.10], 'LineWidth',1.6, 'DisplayName','drag + rolling');
if ~isempty(Eb)
    plot(Eb.v, Eb.F_drive, '-', 'Color',[0.85 0.55 0.00], 'LineWidth',2.2, 'DisplayName',['drive, ' labels{2}]);
end
xlim([0 vmax]); ylim(yl);
xlabel('speed [m/s]'); ylabel('force at the road [N]');
title('Powertrain envelope: what binds, at every speed');
legend('Location','east');
exportgraphics(f, fullfile(outdir,'pt_envelope.png'), 'Resolution', 150); close(f);

% ---- 2. acceleration run ------------------------------------------------
f = figure('Visible','off','Position',[100 100 900 420]);
[t, s, v] = run_trace(E);
subplot(1,2,1); hold on; grid on; box on;
plot(t, 3.6*v, 'Color',[0 0.35 0.65], 'LineWidth',2, 'DisplayName',labels{1});
if ~isempty(Eb), [tb,~,vb] = run_trace(Eb); plot(tb, 3.6*vb, 'Color',[0.85 0.55 0], 'LineWidth',2, 'DisplayName',labels{2}); end
xline(E.t_accel, ':', sprintf('%.0f m: %.3f s', E.s_accel, E.t_accel), 'HandleVisibility','off');
xlabel('time [s]'); ylabel('speed [km/h]'); title('Acceleration, perfect traction control');
legend('Location','southeast');
subplot(1,2,2); hold on; grid on; box on;
plot(s, 3.6*v, 'Color',[0 0.35 0.65], 'LineWidth',2);
if ~isempty(Eb), [~,sb,vb] = run_trace(Eb); plot(sb, 3.6*vb, 'Color',[0.85 0.55 0], 'LineWidth',2); end
xline(E.s_accel, ':', 'HandleVisibility','off');
xlim([0 1.5*E.s_accel]);
xlabel('distance [m]'); ylabel('speed [km/h]'); title('Speed against distance');
exportgraphics(f, fullfile(outdir,'pt_accel.png'), 'Resolution', 150); close(f);

% ---- 3. regen -----------------------------------------------------------
f = figure('Visible','off','Position',[100 100 700 420]);
hold on; grid on; box on;
plot(E.v, E.F_regen, 'Color',[0 0.35 0.65], 'LineWidth',2, 'DisplayName',['regen, ' labels{1}]);
if ~isempty(Eb), plot(Eb.v, Eb.F_regen, 'Color',[0.85 0.55 0], 'LineWidth',2, 'DisplayName',['regen, ' labels{2}]); end
plot(E.v, E.F_resist, 'Color',[0.75 0.30 0.10], 'LineWidth',1.2, 'DisplayName','drag + rolling');
xlim([0 vmax]);
xlabel('speed [m/s]'); ylabel('braking force at the road [N]');
title('Regen: power-limited almost from rest');
legend('Location','northeast');
exportgraphics(f, fullfile(outdir,'pt_regen.png'), 'Resolution', 150); close(f);

fprintf('  figures written to %s\n', outdir);
end

% -------------------------------------------------------------------------
function bands(E, yl, vmax)
C = struct('traction',[0.90 0.93 0.97], 'torque',[0.93 0.97 0.92], ...
           'pack',[0.98 0.95 0.88], 'motor_power',[0.96 0.92 0.96]);
lim = E.limit;  v = E.v;
edges = [1; find(lim(2:end) ~= lim(1:end-1)) + 1; numel(v)+1];
for k = 1:numel(edges)-1
    a = edges(k); b = edges(k+1) - 1;
    key = strrep(char(lim(a)), ' ', '_');
    patch([v(a) v(b) v(b) v(a)], [yl(1) yl(1) yl(2) yl(2)], C.(key), ...
          'EdgeColor','none', 'HandleVisibility','off');
    if v(a) >= vmax, continue; end
    text(mean([v(a) min(v(b), vmax)]), yl(2)*0.97, char(lim(a)), ...
         'HorizontalAlignment','center', 'Color',[0.3 0.3 0.3]);
end
end

function [t, s, v] = run_trace(E)
ok = E.ax > 1e-6;
v = E.v(ok);  a = E.ax(ok);
t = cumtrapz(v, 1 ./ a);
s = cumtrapz(v, v ./ a);
end

function x = field_or(S, f, d)
if isempty(S), x = d; else, x = S.(f); end
end
