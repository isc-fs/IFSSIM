function R = car_figures(carA, carB, opts)
%CAR_FIGURES  The evidence, as figures: every claim the models make, for two cars.
%
%   R = CAR_FIGURES()                     IFS-08 against IFS-09
%   R = CAR_FIGURES('IFS-08','IFS-09')
%   R = CAR_FIGURES(a, b, opts)           opts.plant  run the full plant for the
%                                         acceleration figure (default true, ~1 min a car)
%                                         opts.outdir (default matlab/build/reports/<a>_vs_<b>)
%
%   ONE COMMAND, TWO CARS, EVERY FIGURE OVERLAID. Each figure is the evidence
%   behind a specific claim -- the camber gain the multibody corner computes,
%   the ground the roll centres are measured from, the workbook reproduced,
%   whether multibody fits the real-time plant, the powertrain and lap
%   envelopes -- drawn for both cars on the same axes, car A solid, car B
%   dashed. Today the IFS-09 inherits everything from the IFS-08, so its lines
%   sit on the IFS-08's; as the IFS-09's spec fills in, the same command shows
%   where they part.
%
%   Each car is computed WITH IT ACTIVE (ifssim_car), so every tool, including
%   the multibody sweep, the plant run and the generated battery pack, reads
%   that car's spec and builds into that car's folder.
%
%   R.files     the PNGs written
%   R.captions  one line per figure: the claim, and the number behind it
%   R.data      the per-car results, for anything that wants to go further

if nargin < 1 || isempty(carA), carA = 'IFS-08'; end
if nargin < 2 || isempty(carB), carB = 'IFS-09'; end
if nargin < 3, opts = struct(); end
if ~isfield(opts,'plant'), opts.plant = true; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
for sub = {'spec','plant','vd','pt','lap','aero','tyres','accumulator','sm'}
    addpath(fullfile(root, sub{1}));
end
if ~isfield(opts,'outdir') || isempty(opts.outdir)
    opts.outdir = fullfile(ifssim_workdir(), 'reports', sprintf('%s_vs_%s', carA, carB));
end
if ~isfolder(opts.outdir), mkdir(opts.outdir); end
was = ifssim_car();  restore = onCleanup(@() ifssim_car(was));
cars = {carA, carB};

% ---- the data, one car at a time, with that car active -------------------
for i = 1:2
    fprintf('\n--- %s ---\n', cars{i});
    d = struct();
    ifssim_car(cars{i});
    ifssim_load_workspace();
    d.name = cars{i};
    d.P = ifssim_params();
    evalc('d.Tf = sm_corner_sweep(''front'');');
    evalc('d.Tr = sm_corner_sweep(''rear'');');
    d.Hf = sm_hardpoints('front');   d.Hr = sm_hardpoints('rear');
    d.Gf = susp_geometry(d.Hf);      d.Gr = susp_geometry(d.Hr);
    d.E  = pt_model(d.P);
    d.A  = aero_kpis(d.P);
    d.Ty = tyre_kpis(d.P);
    d.Ac = acc_kpis(d.P);
    d.accel = [];
    if opts.plant
        try
            evalc('d.accel = accel_run(75);');
        catch ME
            fprintf('  plant acceleration run failed for %s: %s\n', cars{i}, ME.message);
        end
    end
    D(i) = d; %#ok<AGROW>
end
ifssim_car(carA);  ifssim_load_workspace();

% ---- the figures ----------------------------------------------------------
F = {};  C = {};
[F, C] = add(F, C, fig_kinematics(D, opts.outdir));
[F, C] = add(F, C, fig_motion_ratio(D, opts.outdir));
[F, C] = add(F, C, fig_front_view(D(1), opts.outdir));
[F, C] = add(F, C, fig_workbook(opts.outdir));
[F, C] = add(F, C, fig_multibody_fmu(root, opts.outdir));
[F, C] = add(F, C, reuse(@() pt_plots(D(1).E, D(2).E, cars, tmpd(opts.outdir)), opts.outdir, ...
    {'pt_envelope.png', '06_powertrain_envelope.png', sprintf(['Powertrain envelope: traction-limited to %.1f m/s, ' ...
      'then the pack (%.1f kW at the shaft). The motor''s own limits never bind.'], D(1).E.v_traction_end, D(1).E.shaft_kW)}));
if ~isempty(D(1).accel)
    [F, C] = add(F, C, fig_plant_accel(D, opts.outdir));
end
[F, C] = add(F, C, reuse(@() aero_plots(D(1).A, D(2).A, cars, tmpd(opts.outdir), true), opts.outdir, ...
    {'aero_lap.png',     '08_lap.png', sprintf('Lap on lap_track: %.2f s, mean %.1f km/h, from the g-g-V envelope.', D(1).A.lap, D(1).A.lap_mean_kmh); ...
     'aero_balance.png', '09_limit_balance.png', sprintf(['Cornering limit and limit balance against speed: the rear saturates first ' ...
      '(front/rear capacity %.3f at 22 m/s) and more so with speed.'], D(1).A.ratio_22); ...
     'aero_trade.png',   '10_aero_trade.png', 'Lap time over the aero package: contours run nearly vertical, so downforce buys far more than drag costs.'}));
[F, C] = add(F, C, reuse(@() tyre_plots(D(1).Ty, D(2).Ty, cars, tmpd(opts.outdir)), opts.outdir, ...
    {'tyre_fy.png',   '11_tyre_fy.png', sprintf('Lateral force against slip angle at four loads: peak at %.1f deg, %.0f%% left once sliding.', D(1).Ty.alpha_peak, 100*D(1).Ty.tail_lat); ...
     'tyre_load.png', '12_tyre_load.png', 'Peak friction and cornering stiffness against load: load sensitivity is what transfer costs.'}));
[F, C] = add(F, C, reuse(@() acc_plots(D(1).Ac, D(2).Ac, cars, tmpd(opts.outdir)), opts.outdir, ...
    {'acc_lap_current.png', '13_acc_lap_current.png', sprintf('One flat-out lap, current per cell: peaks at the %.0f A pack limit (%.1f A a cell).', D(1).Ac.I_limit, D(1).Ac.I_cell_at_limit); ...
     'acc_endurance.png',   '14_acc_endurance.png', sprintf('Charge and sag across a flat-out endurance: %.1f of %.1f laps.', D(1).Ac.laps, D(1).Ac.R.laps_needed)}));
[F, C] = add(F, C, fig_battery(D(1), opts.outdir));
[F, C] = add(F, C, fig_kpis(D, opts.outdir));

R.files = F;  R.captions = C;  R.data = D;  R.outdir = opts.outdir;
fid = fopen(fullfile(opts.outdir, 'captions.txt'), 'w');
for k = 1:numel(F), fprintf(fid, '%s\t%s\n', F{k}, C{k}); end
fclose(fid);
fprintf('\n%d figures in %s\n', numel(F), opts.outdir);
end

% =========================================================================
function [F, C] = add(F, C, fc)
for k = 1:size(fc,1), F{end+1} = fc{k,1}; C{end+1} = fc{k,2}; end %#ok<AGROW>
end

function d = tmpd(outdir)
d = fullfile(outdir, 'tmp');  if ~isfolder(d), mkdir(d); end
end

function fc = reuse(fn, outdir, map)
%REUSE  Run a department's plot function into a scratch folder and keep the
%   figures under report names.
evalc('fn();');
fc = cell(0,2);
for k = 1:size(map,1)
    src = fullfile(outdir, 'tmp', map{k,1});
    if isfile(src)
        copyfile(src, fullfile(outdir, map{k,2}));
        fc(end+1,:) = {map{k,2}, map{k,3}}; %#ok<AGROW>
    end
end
end

function [c, ls] = sty(i)
pal = {[0 0.35 0.65], [0.85 0.50 0]};  st = {'-', '--'};
c = pal{i};  ls = st{i};
end

function save_fig(f, outdir, name)
exportgraphics(f, fullfile(outdir, name), 'Resolution', 150);
close(f);
end

% ---- 01: camber and toe against travel ------------------------------------
function fc = fig_kinematics(D, outdir)
f = figure('Visible','off','Position',[100 100 1100 720]);
ax = {'front','rear'};  assumed = [0.80 0.60];
for a = 1:2
    subplot(2,2,a); hold on; grid on; box on;
    for i = 1:numel(D)
        T = D(i).(['T' ax{a}(1)]);  [c, ls] = sty(i);
        plot(T.travel_mm, T.camber_deg, ls, 'Color', c, 'LineWidth', 2, 'DisplayName', D(i).name);
    end
    ht = D(1).(['H' ax{a}(1)]).track/2;
    tr = linspace(-55, 55, 3);
    plot(tr, -assumed(a)/ht*180/pi*tr/1000, ':', 'Color', [0.7 0.2 0.2], 'LineWidth', 1.5, ...
         'DisplayName', sprintf('assumed until 2026-10 (gain %.2f)', assumed(a)));
    g = D(1).P.Susp.(['CamberGain' cap(ax{a})]);
    title(sprintf('%s camber: gain %.3f of body roll recovered', cap(ax{a}), g));
    xlabel('wheel travel [mm]  (+ = bump)'); ylabel('camber change [deg]  (SAE, + = top out)');
    legend('Location','northeast');
    subplot(2,2,2+a); hold on; grid on; box on;
    for i = 1:numel(D)
        T = D(i).(['T' ax{a}(1)]);  [c, ls] = sty(i);
        plot(T.travel_mm, -T.toe_deg, ls, 'Color', c, 'LineWidth', 2, 'DisplayName', D(i).name);
    end
    yline(0, ':', 'Color', [0.7 0.2 0.2], 'LineWidth', 1.5, 'DisplayName','assumed: zero');
    title(sprintf('%s toe: %+.2f deg/m of bump (toe-in +)', cap(ax{a}), D(1).P.Susp.(['BumpSteer' cap(ax{a})])));
    xlabel('wheel travel [mm]'); ylabel('toe change [deg]  (+ = toe-in)');
    legend('Location','best');
end
sgtitle('Suspension kinematics from the multibody corner on the real hardpoints');
save_fig(f, outdir, '01_kinematics.png');
fc = {'01_kinematics.png', sprintf(['Camber and toe against travel, from the multibody corner on the workbook''s ' ...
      'hardpoints: camber gain %.3f front and %.3f rear, against 0.80 and 0.60 assumed; front bump steer %+.2f deg/m, ' ...
      'against an assumed zero.'], D(1).P.Susp.CamberGainFront, D(1).P.Susp.CamberGainRear, D(1).P.Susp.BumpSteerFront)};
end

% ---- 02: motion ratio -----------------------------------------------------
function fc = fig_motion_ratio(D, outdir)
f = figure('Visible','off','Position',[100 100 1000 420]);
ax = {'front','rear'};
for a = 1:2
    subplot(1,2,a); hold on; grid on; box on;
    for i = 1:numel(D)
        mr = D(i).(['T' ax{a}(1)]).Properties.UserData.mrCurve;  [c, ls] = sty(i);
        plot(mr(:,1), mr(:,2), ls, 'Color', c, 'LineWidth', 2, 'DisplayName', D(i).name);
    end
    yline(0.70, ':', 'assumed 0.70', 'Color', [0.7 0.2 0.2], 'LineWidth', 1.5, 'HandleVisibility','off');
    xline(0, ':', 'HandleVisibility','off');
    title(sprintf('%s: %.3f at static, regressive', cap(ax{a}), D(1).P.Susp.(['MotionRatio' cap(ax{a})])));
    xlabel('wheel travel [mm]  (+ = bump)'); ylabel('motion ratio (damper / wheel travel)');
    legend('Location','northeast');
end
sgtitle('Motion ratio from the pushrod and rocker hardpoints');
save_fig(f, outdir, '02_motion_ratio.png');
fc = {'02_motion_ratio.png', sprintf(['Motion ratio against travel, from the pushrod, rocker and damper hardpoints: ' ...
      '%.3f front and %.3f rear at static, against 0.70 assumed (the workbook''s own spring sheet assumes 1.2). ' ...
      'Regressive: it falls into bump.'], D(1).P.Susp.MotionRatioFront, D(1).P.Susp.MotionRatioRear)};
end

% ---- 03: front view, the roll centres and the ground ----------------------
function fc = fig_front_view(d, outdir)
f = figure('Visible','off','Position',[100 100 1100 520]);
ax = {'front','rear'};  sheetRC = [28.2083 58.016];  monoRC = [12.8 31];
gz = d.Hf.groundZ_mm;
for a = 1:2
    subplot(1,2,a); hold on; grid on; box on; axis equal;
    H = d.(['H' ax{a}(1)]);  G = d.(['G' ax{a}(1)]);
    [lA, lB] = fv(H.lca_front, H.lca_rear, H.lca_outer);
    [uA, uB] = fv(H.uca_front, H.uca_rear, H.uca_outer);
    ic = G.icFV;  cp = H.contact([2 3]);
    plot([ic(1) lB(1)]*1e3, [ic(2) lB(2)]*1e3, ':', 'Color', [0.55 0.55 0.55], 'HandleVisibility','off');
    plot([ic(1) uB(1)]*1e3, [ic(2) uB(2)]*1e3, ':', 'Color', [0.55 0.55 0.55], 'HandleVisibility','off');
    plot([lA(1) lB(1)]*1e3, [lA(2) lB(2)]*1e3, '-', 'Color', [0 0.35 0.65], 'LineWidth', 3, 'DisplayName','lower wishbone');
    plot([uA(1) uB(1)]*1e3, [uA(2) uB(2)]*1e3, '-', 'Color', [0 0.55 0.85], 'LineWidth', 3, 'DisplayName','upper wishbone');
    plot([lB(1) uB(1)]*1e3, [lB(2) uB(2)]*1e3, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'DisplayName','upright');
    plot([cp(1) 0]*1e3, [cp(2) G.rc_m]*1e3, '-', 'Color', [0.2 0.6 0.3], 'LineWidth', 1.2, 'HandleVisibility','off');
    yline(0, '-', 'ground (wheel centre - tyre radius)', 'Color', [0.2 0.6 0.3], 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
    yline(-gz, '--', 'workbook Z = 0', 'Color', [0.7 0.2 0.2], 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
    plot(0, G.rc_m*1e3, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.2 0.6 0.3], 'MarkerEdgeColor','k', ...
         'DisplayName', sprintf('roll centre, this ground: %.1f mm', G.rc_m*1e3));
    plot(0, sheetRC(a) - gz, 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.85 0.4 0.4], 'MarkerEdgeColor','k', ...
         'DisplayName', sprintf('workbook FASE 2: %.1f mm above its Z = 0', sheetRC(a)));
    plot(0, monoRC(a), 'd', 'MarkerSize', 9, 'MarkerFaceColor', [0.95 0.75 0.2], 'MarkerEdgeColor','k', ...
         'DisplayName', sprintf('workbook MONO sheet: %.1f mm', monoRC(a)));
    xlim([-100 700]);  ylim([-140 480]);
    xlabel('y [mm]  (car centreline at 0)'); ylabel('z [mm] above the ground');
    title(sprintf('%s axle, front view (IC at y = %.0f mm, off the left)', cap(ax{a}), ic(1)*1e3));
    legend('Location','northwest', 'FontSize', 8);
end
sgtitle(sprintf('%s roll centres: which ground the workbook measured from', d.name));
save_fig(f, outdir, '03_roll_centres_datum.png');
fc = {'03_roll_centres_datum.png', sprintf(['Front view of each axle with the instant-centre construction. With the ground ' ...
      'at wheel centre minus tyre radius the roll centres are %.1f mm and %.1f mm, matching the MONO sheet''s independent ' ...
      '12.8 and 31 mm; the Susp_Geometry sheet''s 28.2 and 58.0 mm are measured from its own Z = 0, %.1f mm below the ground.'], ...
      d.Gf.rc_m*1e3, d.Gr.rc_m*1e3, gz)};
end

function [A, B] = fv(pf, pr, bj)
t = (bj(1) - pf(1)) / (pr(1) - pf(1));
in = pf + t*(pr - pf);
A = in([2 3]);  B = bj([2 3]);
end

% ---- 04: the workbook, reproduced -----------------------------------------
function fc = fig_workbook(outdir)
sheet = struct('GroundZ_mm', 0, 'car', 'IFS-08');
Hf = sm_hardpoints('front', sheet);  Hr = sm_hardpoints('rear', sheet);
Gf = susp_geometry(Hf, struct('x', Hf.wheel_centre(1) - 0.7057009, 'h', 0.3, 'brakeFront', 0.6));
Gr = susp_geometry(Hr, struct('x', Hr.wheel_centre(1) + 0.8657009, 'h', 0.3, 'brakeFront', 0.6));
rows = { 'front KPI [deg]', Gf.kpi_deg, 7.125;  'front caster [deg]', Gf.caster_deg, 6.2065
         'front scrub [mm]', Gf.scrub_m*1e3, 21.25;  'front trail [mm]', Gf.trail_m*1e3, 36.9125
         'front KP offset [mm]', Gf.kpOffset_m*1e3, 60;  'front spindle [mm]', Gf.spindle_m*1e3, 38
         'front IC y [mm]', Gf.icFV(1)*1e3, -4292.1831;  'front IC z [mm]', Gf.icFV(2)*1e3, 230
         'front roll centre [mm]', Gf.rc_m*1e3, 28.2083;  'front FVSA [mm]', Gf.fvsa_m*1e3, 4897.5867
         'front anti-dive [%]', Gf.antiDive_pct, 26.6509;  'rear KPI [deg]', Gr.kpi_deg, 3.5763
         'rear caster [deg]', Gr.caster_deg, 0;  'rear scrub [mm]', Gr.scrub_m*1e3, 35.625
         'rear KP offset [mm]', Gr.kpOffset_m*1e3, 55;  'rear spindle [mm]', Gr.spindle_m*1e3, 50
         'rear IC y [mm]', Gr.icFV(1)*1e3, -1778.656;  'rear roll centre [mm]', Gr.rc_m*1e3, 58.016
         'rear FVSA [mm]', Gr.fvsa_m*1e3, 2389.7498;  'rear anti-lift [%]', Gr.antiLift_pct, 22.1884 };
res = max(abs([rows{:,2}] - [rows{:,3}]), 1e-12);
f = figure('Visible','off','Position',[100 100 900 640]);
barh(res, 'FaceColor', [0 0.35 0.65]); hold on; grid on; box on;
set(gca, 'XScale','log', 'YTick', 1:size(rows,1), 'YTickLabel', ...
    cellfun(@(n,v) sprintf('%s  (sheet %g)', n, v), rows(:,1), rows(:,3), 'uni', 0), 'YDir','reverse', 'FontSize', 8);
xline(1e-3, '--', 'test tolerance', 'Color', [0.7 0.2 0.2]);
xlim([1e-13 1]);
xlabel('|computed - workbook|  (log scale)');
title(sprintf('All %d figures the Susp\\_Geometry sheet computes, reproduced from the imported hardpoints', size(rows,1)));
save_fig(f, outdir, '04_workbook_reproduced.png');
fc = {'04_workbook_reproduced.png', sprintf(['Every figure the workbook computes (%d of them: kingpin, caster, scrub, trail, ' ...
      'instant and roll centres, swing arms, anti-dive/lift), reproduced from the imported hardpoints with the sheet''s ' ...
      'own datum. Largest residual %.1e, against a 1e-3 test tolerance.'], size(rows,1), max(res))};
end

% ---- 05: multibody in the plant? -----------------------------------------
function fc = fig_multibody_fmu(root, outdir)
fc = cell(0,2);
pr = fullfile(ifssim_workdir(), 'sm_probe', 'probe_results.mat');
if ~isfile(pr), return; end
S = load(pr);  R = S.R;
ls = fullfile(ifssim_workdir(), 'sm_probe', 'load_sweep.mat');
rtf = timing(root);
f = figure('Visible','off','Position',[100 100 1500 460]);
subplot(1,3,1); hold on; grid on; box on;
cols = {[0.3 0.3 0.3], [0 0.35 0.65], [0.85 0.5 0], [0.6 0.2 0.6], [0.2 0.6 0.3]};
for k = 1:numel(R)
    if ~R(k).ok, continue; end
    plot(R(k).t, (R(k).z - R(k).z(1))*1000, '-', 'Color', cols{min(k,end)}, 'LineWidth', 1.4 + 1.6*(k==1), ...
         'DisplayName', sprintf('%s (max err %.3f mm)', R(k).name, R(k).err_mm));
end
xlabel('time [s]'); ylabel('wheel travel [mm]');
title('One closed-loop corner, 600 N stepped onto the contact patch');
legend('Location','southeast', 'FontSize', 7);
lim = NaN;
subplot(1,3,2); hold on; grid on; box on;
if isfile(ls)
    L = load(ls);  L = L.S;  ok = [L.ok];
    plot([L.F], [L.travel], '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility','off');
    plot([L(ok).F], [L(ok).travel], 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.6 0.3], 'MarkerEdgeColor','k', ...
         'DisplayName','plant''s solver holds the linkage');
    plot([L(~ok).F], [L(~ok).travel], 'x', 'MarkerSize', 11, 'LineWidth', 2, 'Color', [0.8 0.2 0.2], ...
         'DisplayName','plant''s solver loses it');
    lim = max([L(ok).travel]);
    yline(lim, '--', sprintf('valid to %.0f mm: bump stop needed below this', lim), 'Color', [0.8 0.2 0.2], ...
          'LabelHorizontalAlignment','left', 'HandleVisibility','off');
    yline(30, ':', 'travel the car uses (~30 mm)', 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
    xlabel('vertical load at the contact patch [N]'); ylabel('steady wheel travel, reference [mm]');
    title('Where the linkage snaps through');
    legend('Location','northwest', 'FontSize', 8);
end
subplot(1,3,3); hold on; grid on; box on;
if ~isempty(rtf)
    b = bar(categorical({'whole plant FMU','one multibody corner FMU'}, {'whole plant FMU','one multibody corner FMU'}), ...
            [rtf(1), rtf(2)], 'FaceColor', 'flat');
    b.CData = [0.2 0.6 0.3; 0 0.35 0.65];
    set(gca, 'YScale','log');  yline(1, '--', 'real time', 'Color', [0.7 0.2 0.2]);
    ylim([1 10^ceil(log10(max(rtf))+0.5)]);
    text(1:2, [rtf(1) rtf(2)]*1.25, compose('%.1fx', [rtf(1) rtf(2)]), 'HorizontalAlignment','center');
    ylabel('real-time factor, stepped as the simulator steps it'); title('Speed, at the plant''s own solver settings');
end
save_fig(f, outdir, '05_multibody_in_the_plant.png');
k4 = R(4);
fc = {'05_multibody_in_the_plant.png', sprintf(['Can multibody live in the real-time plant? Yes, within its travel. A ' ...
      'closed-loop corner (the real front hardpoints, spring and damper on the rocker) runs at the plant''s 1/960 s step ' ...
      'with the plant''s own solver settings, %.3f mm from a tight reference, and exported as an FMU it runs at %.1fx real ' ...
      'time against the whole plant''s %.1fx. The plant''s solver holds the linkage to %.0f mm of bump; past that the ' ...
      'linkage snaps through, so the model -- and the car -- needs a bump stop before it.'], k4.err_mm, rtf(2), rtf(1), lim)};
end

function rtf = timing(root)
rtf = [];
sh = fullfile(root, '..', 'tools', 'fmu', 'step_timing', 'run.sh');
plantF = fullfile(root, 'plant', 'fmu', 'IFSSIM_Plant.fmu');
cornerF = fullfile(ifssim_workdir(), 'sm_probe', 'fmu', 'IFSSIM_SM_Probe.fmu');
if ~isfile(sh) || ~isfile(plantF) || ~isfile(cornerF), return; end
[~, a] = system(sprintf('"%s" "%s" 10', sh, plantF));
[~, b] = system(sprintf('"%s" "%s" 10 0 600 0.05', sh, cornerF));
ra = regexp(a, 'real-time factor ([0-9.]+)x', 'tokens', 'once');
rb = regexp(b, 'real-time factor ([0-9.]+)x', 'tokens', 'once');
if ~isempty(ra) && ~isempty(rb), rtf = [str2double(ra{1}), str2double(rb{1})]; end
end

% ---- 07: the plant against the powertrain model --------------------------
function fc = fig_plant_accel(D, outdir)
f = figure('Visible','off','Position',[100 100 900 460]); hold on; grid on; box on;
for i = 1:numel(D)
    if isempty(D(i).accel), continue; end
    [c, ls] = sty(i);
    h = D(i).accel.hist;
    plot(h(:,1), 3.6*h(:,3), ls, 'Color', c, 'LineWidth', 2, 'DisplayName', [D(i).name ' plant, full throttle']);
    E = D(i).E;  ok = E.ax > 1e-6;  v = E.v(ok);  t = cumtrapz(v, 1./E.ax(ok));
    plot(t, 3.6*v, ':', 'Color', c, 'LineWidth', 2, 'DisplayName', [D(i).name ' pt\_model, perfect traction']);
end
xlim([0 6]); xlabel('time [s]'); ylabel('speed [km/h]');
title(sprintf('Acceleration: the plant against the powertrain model (75 m in %.2f s vs %.2f s)', ...
      D(1).accel.t_target, D(1).E.t_accel));
legend('Location','southeast');
save_fig(f, outdir, '07_plant_vs_powertrain_model.png');
fc = {'07_plant_vs_powertrain_model.png', sprintf(['The full plant at full throttle against pt_model: the same pack-limited ' ...
      'slope from 22 m/s up (test_pt_model, within 5%%); the plant is %.2f s slower over 75 m because it has no traction ' ...
      'control, and that gap is the most traction control could be worth.'], D(1).accel.t_target - D(1).E.t_accel)};
end

% ---- 15: the generated battery pack against the analytic answer -----------
function fc = fig_battery(d, outdir)
fc = cell(0,2);
P = d.P;  PK = pack_from_cells(P);
I = 100;  T = 30;
mdl = 'car_figures_battery';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'SolverType','Fixed-step','Solver','ode1','FixedStep','0.0010416666666666671','StopTime',num2str(T));
add_block('simulink/Sources/Constant', [mdl '/I'], 'Value', num2str(I), 'Position',[30 60 60 80]);
try
    evalc('info = build_battery_pack(mdl, ''Pack'', [150 40 300 200], P);');
catch ME
    fprintf('  battery figure skipped: %s\n', ME.message);  close_system(mdl,0);  return
end
add_line(mdl, 'I/1', 'Pack/1');
add_block('simulink/Sinks/To Workspace', [mdl '/v'], 'VariableName','CF_v', 'SaveFormat','Timeseries', 'Position',[400 40 460 60]);
add_line(mdl, 'Pack/1', 'v/1');
for m = 1:info.NumModules
    add_block('simulink/Sinks/Terminator', sprintf('%s/t%d', mdl, m), 'Position',[400 70+25*m 420 85+25*m]);
    add_line(mdl, sprintf('Pack/%d', m+1), sprintf('t%d/1', m));
end
ifssim_load_workspace();
r = sim(mdl);  v = r.get('CF_v');
close_system(mdl, 0);
soc0 = P.Assumed.BatteryInitialSoC;
t = v.Time;
va = PK.Ns * interp1(P.Cell.OCV_SoC, P.Cell.OCV_V, soc0 - I*t/(PK.CapacityAh*3600)) - I*PK.Rint;
f = figure('Visible','off','Position',[100 100 800 420]); hold on; grid on; box on;
plot(t, squeeze(v.Data), '-', 'Color', [0 0.35 0.65], 'LineWidth', 4, 'DisplayName', ...
     sprintf('Simscape Battery pack (%s)', info.Library));
plot(t, va, '--', 'Color', [0.85 0.5 0], 'LineWidth', 2, 'DisplayName', 'analytic: Ns OCV(soc) - I R');
xlabel('time [s]'); ylabel('pack terminal voltage [V]');
title(sprintf('%s pack at a constant %d A: max difference %.2g V', d.name, I, max(abs(squeeze(v.Data) - va))));
legend('Location','northeast', 'Interpreter','none');
save_fig(f, outdir, '15_battery_pack.png');
fc = {'15_battery_pack.png', sprintf(['The plant''s accumulator, generated by Simscape Battery from the spec''s ' ...
      'arrangement (%dp%ds, %d modules), against the closed form under %d A: they agree to %.2g V.'], ...
      info.Np, info.Ns, info.NumModules, I, max(abs(squeeze(v.Data) - va)))};
end

% ---- 16: the two cars, side by side --------------------------------------
function fc = fig_kpis(D, outdir)
k = { 'lap [s]',              @(d) d.A.lap
      '75 m [s]',             @(d) d.E.t_accel
      'skid pad [s]',         @(d) d.Ty.skidpad
      'top speed [m/s]',      @(d) d.E.v_top
      'endurance laps',       @(d) d.Ac.laps
      'limit @22 m/s [g]',    @(d) d.A.aylim_22
      'F/R capacity @22',     @(d) d.A.ratio_22
      'understeer K [deg/g]', @(d) d.Ty.understeer
      'camber gain F',        @(d) d.P.Susp.CamberGainFront
      'camber gain R',        @(d) d.P.Susp.CamberGainRear
      'roll centre F [mm]',   @(d) d.P.RollCenterFront*1e3
      'roll centre R [mm]',   @(d) d.P.RollCenterRear*1e3
      'motion ratio F',       @(d) d.P.Susp.MotionRatioFront
      'pack energy [kWh]',    @(d) d.Ac.E_kWh
      'shaft power [kW]',     @(d) d.E.shaft_kW };
n = size(k,1);  V = zeros(n, numel(D));
for i = 1:numel(D), for j = 1:n, V(j,i) = k{j,2}(D(i)); end, end
rel = 100 * V ./ V(:,1);
f = figure('Visible','off','Position',[100 100 1000 620]);
b = barh(rel, 'grouped'); hold on; grid on; box on;
b(1).FaceColor = [0 0.35 0.65];  if numel(b) > 1, b(2).FaceColor = [0.85 0.5 0]; end
set(gca, 'YTick', 1:n, 'YTickLabel', k(:,1), 'YDir','reverse');
xline(100, ':');
for j = 1:n
    text(max(rel(j,:)) + 1, j, sprintf('  %s %.4g   %s %.4g', D(1).name, V(j,1), D(end).name, V(j,end)), 'FontSize', 8);
end
xlim([0 160]);
xlabel(sprintf('percent of %s', D(1).name));
legend({D.name}, 'Location','southeast');
same = all(abs(V(:,end) - V(:,1)) < 1e-9);
title(sprintf('%s against %s: %s', D(1).name, D(end).name, ...
      tern(same, 'identical -- the second car still inherits everything', 'where they differ')));
save_fig(f, outdir, '16_car_comparison.png');
fc = {'16_car_comparison.png', sprintf(['The headline numbers of both cars, each as a percentage of %s. %s'], D(1).name, ...
      tern(same, sprintf('Today they are identical: %s inherits every value from %s, and the bars will part as its spec fills in.', D(end).name, D(1).name), ...
                 'Bars that differ from 100 are where the cars part.'))};
end

function s = cap(x), s = [upper(x(1)) x(2:end)]; end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
