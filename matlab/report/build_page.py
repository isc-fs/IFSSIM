#!/usr/bin/env python3
"""Build the evidence page from car_figures' output.

    python3 matlab/report/build_page.py <figures dir> <out dir> [--intro TEXT] [state|text ...]

<figures dir> is what car_figures wrote (PNGs + captions.txt), e.g.
matlab/build/reports/IFS-08_vs_IFS-09. Each "state|text" argument is a status
chip; state is "ok" or "warn". The captions -- and so every number on the
page -- come from captions.txt, which car_figures writes from the data.
"""
import datetime, html, pathlib, shutil, sys

args = sys.argv[1:]
intro = None
if '--intro' in args:
    i = args.index('--intro'); intro = args[i + 1]; del args[i:i + 2]
src, out = pathlib.Path(args[0]), pathlib.Path(args[1])
status = [s.split('|', 1) for s in args[2:]]
cars = src.name.split('_vs_') if '_vs_' in src.name else ['car A', 'car B']
out.mkdir(parents=True, exist_ok=True)
caps = {}
for line in (src / 'captions.txt').read_text().splitlines():
    if '\t' in line:
        f, c = line.split('\t', 1)
        caps[f] = c

SECTIONS = [
    ('geometry', 'Suspension geometry', 'From the workbook hardpoints, through the multibody corner, into the spec.',
     [('01_kinematics.png', 'Camber and toe against wheel travel', 'test_kinematics_feed'),
      ('02_motion_ratio.png', 'Motion ratio against wheel travel', 'test_kinematics_feed'),
      ('03_roll_centres_datum.png', 'Roll centres, and the ground they are measured from', 'test_susp_geometry'),
      ('04_workbook_reproduced.png', 'The workbook, reproduced', 'test_susp_geometry')]),
    ('multibody', 'Multibody in the real-time plant', 'Whether a closed-loop corner can run inside the simulator’s FMU.',
     [('05_multibody_in_the_plant.png', 'Accuracy, travel limit and speed of a multibody corner', 'sm_fixed_step_probe')]),
    ('powertrain', 'Powertrain', 'What reaches the road, and how the design model compares with the plant.',
     [('06_powertrain_envelope.png', 'Force at the road against speed', 'test_pt_model'),
      ('07_plant_vs_powertrain_model.png', 'Acceleration: plant against powertrain model', 'test_pt_model')]),
    ('lap', 'Lap, balance and aero', 'The quasi-steady lap on the car’s g-g-V envelope.',
     [('08_lap.png', 'Speed around the lap', 'test_lap_sim'),
      ('09_limit_balance.png', 'Cornering limit and limit balance against speed', 'test_lap_sim'),
      ('10_aero_trade.png', 'Lap time across the aero package', 'aero_report')]),
    ('tyres', 'Tyres', 'The Magic Formula curve both models run.',
     [('11_tyre_fy.png', 'Lateral force against slip angle', 'test_tyre_kpis'),
      ('12_tyre_load.png', 'Peak friction and cornering stiffness against load', 'test_tyre_kpis')]),
    ('accumulator', 'Accumulator', 'The Simscape Battery packs and a flat-out endurance.',
     [('13_acc_lap_current.png', 'Cell current around one lap', 'test_acc_endurance'),
      ('14_acc_endurance.png', 'Charge and voltage sag across the endurance', 'test_acc_endurance'),
      ('15_battery_pack.png', 'Each generated pack against the closed form', 'test_battery_pack')]),
    ('cars', 'The two cars', 'Every headline number, side by side.',
     [('16_car_comparison.png', f'{cars[0]} against {cars[1]}', 'car_figures')]),
]

def esc(s): return html.escape(s, quote=True)

if intro is None:
    intro = (f'Every claim the models make, as the figure that supports it and the test that keeps it true. '
             f'{cars[0]} is drawn solid and {cars[1]} dashed; where a value is still inherited, the two lines sit '
             f'on each other. Rerunning <code>car_figures</code> shows each new difference.')

nav = ''.join(f'<a href="#{sid}">{esc(t)}</a>' for sid, t, _, figs in SECTIONS
              if any((src / f).exists() for f, _, _ in figs))
chips = ''.join(f'<li class="chip {esc(st)}"><span class="dot" aria-hidden="true"></span>{esc(tx)}</li>'
                for st, tx in status)
body, n = [], 0
for sid, title, lede, figs in SECTIONS:
    present = [(f, t, chk) for f, t, chk in figs if (src / f).exists()]
    if not present:
        continue
    cards = []
    for f, t, chk in present:
        n += 1
        shutil.copy(src / f, out / f)
        cards.append(f'''
      <figure class="fig" id="f{esc(f[:2])}">
        <figcaption>
          <h3><span class="num">{esc(f[:2])}</span>{esc(t)}</h3>
          <p>{esc(caps.get(f, ''))}</p>
          <p class="checked">Checked by <code>{esc(chk)}</code></p>
        </figcaption>
        <a class="plate" href="{esc(f)}" target="_blank" rel="noopener"><img src="{esc(f)}" alt="{esc(t)}" loading="lazy"></a>
      </figure>''')
    body.append(f'''
    <section id="{sid}">
      <header class="sec"><h2>{esc(title)}</h2><p>{esc(lede)}</p></header>
      {''.join(cards)}
    </section>''')

page = f'''<title>{esc(cars[0])} vs {esc(cars[1])} Evidence</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600&family=Source+Sans+3:wght@400;600&family=JetBrains+Mono:wght@400;500&display=swap">
<style>
/* Layout: one reading column of evidence cards, a sticky section bar, figures on light plates. */
:root {{
  --bg: #f3f5f7; --surface: #ffffff; --fg: #17202a; --muted: #55606c; --rule: #d9dee4;
  --accent: #005a9e; --car-b: #c97400; --ok: #1f7a4a; --warn: #a15c00; --plate: #ffffff;
  --display: "Barlow Condensed", "Arial Narrow", system-ui, sans-serif;
  --body: "Source Sans 3", system-ui, -apple-system, "Segoe UI", sans-serif;
  --mono: "JetBrains Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #0f1418; --surface: #161d23; --fg: #e4e9ee; --muted: #9aa6b2; --rule: #2a343d;
  --accent: #5fa8e8; --car-b: #f0a640; --ok: #5cc28a; --warn: #f0b45a; --plate: #f7f9fb; color-scheme: dark; }} }}
:root[data-theme="dark"] {{
  --bg: #0f1418; --surface: #161d23; --fg: #e4e9ee; --muted: #9aa6b2; --rule: #2a343d;
  --accent: #5fa8e8; --car-b: #f0a640; --ok: #5cc28a; --warn: #f0b45a; --plate: #f7f9fb; color-scheme: dark; }}
* {{ box-sizing: border-box; }}
body {{ background: var(--bg); color: var(--fg); font: 16px/1.55 var(--body); margin: 0; }}
.wrap {{ max-width: 1120px; margin: 0 auto; padding-inline: 20px; padding-block: 32px 64px; }}
.top h1 {{ font: 600 clamp(2rem, 4vw, 2.9rem)/1.05 var(--display); letter-spacing: .01em; margin: 0 0 .35rem; text-wrap: balance; }}
.top h1 .a {{ color: var(--accent); }} .top h1 .b {{ color: var(--car-b); }}
.top p {{ color: var(--muted); max-width: 68ch; margin: 0 0 1rem; }}
.meta {{ font: 500 .78rem var(--mono); color: var(--muted); letter-spacing: .02em; }}
.chips {{ list-style: none; display: flex; flex-wrap: wrap; gap: 8px; padding: 0; margin: 1.1rem 0 0; }}
.chip {{ display: inline-flex; align-items: center; gap: 8px; background: var(--surface); border: 1px solid var(--rule);
  border-radius: 999px; padding: 5px 12px; font-size: .86rem; }}
.chip .dot {{ width: 8px; height: 8px; border-radius: 50%; background: var(--ok); flex: none; }}
.chip.warn .dot {{ background: var(--warn); }}
nav.bar {{ position: sticky; top: env(safe-area-inset-top, 0px); z-index: 2; background: var(--bg);
  border-bottom: 1px solid var(--rule); margin: 28px 0 8px; padding: 10px 0; display: flex; gap: 6px 18px; flex-wrap: wrap; }}
nav.bar a {{ font: 600 .82rem var(--body); letter-spacing: .06em; text-transform: uppercase; color: var(--muted); text-decoration: none; }}
nav.bar a:hover, nav.bar a:focus-visible {{ color: var(--accent); }}
a:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 3px; border-radius: 4px; }}
section {{ padding-top: 28px; scroll-margin-top: 56px; }}
.sec h2 {{ font: 600 1.7rem/1.1 var(--display); letter-spacing: .02em; margin: 0; }}
.sec p {{ color: var(--muted); margin: .25rem 0 1rem; }}
.fig {{ margin: 0 0 22px; background: var(--surface); border: 1px solid var(--rule); border-radius: 10px; overflow: hidden; scroll-margin-top: 56px; }}
.fig figcaption {{ padding: 16px 18px 12px; display: grid; gap: 6px; min-width: 0; }}
.fig h3 {{ font: 600 1.05rem/1.3 var(--body); margin: 0; display: flex; gap: 10px; align-items: baseline; }}
.fig .num {{ font: 500 .78rem var(--mono); color: var(--accent); }}
.fig p {{ margin: 0; max-width: 78ch; }}
.fig .checked {{ font-size: .84rem; color: var(--muted); }}
code {{ font: .82rem var(--mono); background: var(--bg); border: 1px solid var(--rule); border-radius: 4px; padding: 1px 6px; }}
.plate {{ display: block; background: var(--plate); padding: 10px; border-top: 1px solid var(--rule); }}
.plate img {{ display: block; width: 100%; height: auto; max-width: 100%; }}
.legend {{ display: inline-flex; gap: 14px; font-size: .86rem; color: var(--muted); flex-wrap: wrap; }}
.legend i {{ display: inline-block; width: 22px; height: 3px; vertical-align: middle; margin-right: 6px; background: var(--accent); }}
.legend i.b {{ background: repeating-linear-gradient(90deg, var(--car-b) 0 6px, transparent 6px 10px); }}
footer {{ margin-top: 36px; color: var(--muted); font-size: .86rem; border-top: 1px solid var(--rule); padding-top: 14px; }}
@media (max-width: 560px) {{ .wrap {{ padding-inline: 16px; }} .fig figcaption {{ padding: 14px; }} .plate {{ padding: 6px; }} }}
</style>
<div class="wrap">
  <header class="top">
    <p class="meta">IFSSIM · dev-manual · generated {datetime.date.today().isoformat()} by <code>car_figures</code></p>
    <h1><span class="a">{esc(cars[0])}</span> vs <span class="b">{esc(cars[1])}</span>: the evidence</h1>
    <p>{intro}</p>
    <span class="legend"><span><i></i>{esc(cars[0])}, solid</span><span><i class="b"></i>{esc(cars[1])}, dashed</span></span>
    <ul class="chips">{chips}</ul>
  </header>
  <nav class="bar" aria-label="Sections">{nav}</nav>
  {''.join(body)}
  <footer>{n} figures. Regenerate with <code>car_figures('{esc(cars[0])}','{esc(cars[1])}')</code> in MATLAB, then <code>python3 matlab/report/build_page.py</code>; figures land in <code>matlab/build/reports/</code>.</footer>
</div>
'''
(out / 'index.html').write_text(page)
print(f'{n} figures -> {out / "index.html"}')
