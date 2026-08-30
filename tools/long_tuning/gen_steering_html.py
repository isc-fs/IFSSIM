#!/usr/bin/env python3
"""Generate a self-contained steering-divergence diagnostic page from the CSV."""
import csv, os

CSV = "tools/long_tuning/hairpin_steering.csv"
OUT = "tools/long_tuning/steering_divergence.html"

rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        try:
            rows.append(dict(
                t=float(r["t"]), assi=int(r["assi"] or 0),
                cmd=float(r["cmd_az"] or 0), tgt=float(r["target_deg"] or 0),
                act=float(r["actual_deg"] or 0), mot=float(r["motor_deg"] or 0),
            ))
        except ValueError:
            pass

drive = [r["t"] for r in rows if r["assi"] == 3]
A, B = min(drive), max(drive)

# chart geometry
W, H = 920, 470
ML, MR, MT, MB = 58, 24, 28, 44
x0 = 4.0
x1 = 17.2
ymin, ymax = -72, 72

def sx(t): return ML + (t - x0) / (x1 - x0) * (W - ML - MR)
def sy(v): return MT + (ymax - v) / (ymax - ymin) * (H - MT - MB)

def poly(key):
    pts = [f"{sx(r['t']):.1f},{sy(r[key]):.1f}" for r in rows if x0 <= r["t"] <= x1]
    return " ".join(pts)

# axes ticks
yt = list(range(-60, 61, 30))
xt = list(range(4, 18, 2))
ygrid = "".join(f'<line class="grid" x1="{ML}" y1="{sy(v):.1f}" x2="{W-MR}" y2="{sy(v):.1f}"/>' for v in yt)
xgrid = "".join(f'<line class="grid" x1="{sx(t):.1f}" y1="{MT}" x2="{sx(t):.1f}" y2="{H-MB}"/>' for t in xt)
ylab = "".join(f'<text class="ax" x="{ML-8}" y="{sy(v)+3:.1f}" text-anchor="end">{v}</text>' for v in yt)
xlab = "".join(f'<text class="ax" x="{sx(t):.1f}" y="{H-MB+16}" text-anchor="middle">{t}</text>' for t in xt)

drive_x, drive_w = sx(A), sx(B) - sx(A)
emerg_x = sx(B)
# divergence onset marker (~ where actual first exceeds 40 deg)
onset = next((r["t"] for r in rows if r["t"] > A and abs(r["act"]) > 40), 12.0)
onset_x = sx(onset)
sat_y = sy(63.5)

html = f"""<title>Steering divergence — hairpin bench replay</title>
<style>
  :root {{
    --bg:#f4f6f8; --panel:#ffffff; --ink:#12161c; --muted:#5a6672; --faint:#8a95a1;
    --line:#dfe4ea; --grid:#eceff3; --accent:#0ca6c4;
    --good:#1f9d55; --bad:#e03131; --ref:#e8850c;
    --shade:rgba(12,166,196,.06);
  }}
  @media (prefers-color-scheme:dark) {{
    :root {{
      --bg:#0d1117; --panel:#141a22; --ink:#e6edf3; --muted:#9aa7b4; --faint:#6b7885;
      --line:#232c37; --grid:#1b222b; --accent:#38c6e0;
      --good:#38d17a; --bad:#ff5c5c; --ref:#f5a742;
      --shade:rgba(56,198,224,.08);
    }}
  }}
  :root[data-theme="dark"] {{
    --bg:#0d1117; --panel:#141a22; --ink:#e6edf3; --muted:#9aa7b4; --faint:#6b7885;
    --line:#232c37; --grid:#1b222b; --accent:#38c6e0;
    --good:#38d17a; --bad:#ff5c5c; --ref:#f5a742; --shade:rgba(56,198,224,.08);
  }}
  :root[data-theme="light"] {{
    --bg:#f4f6f8; --panel:#ffffff; --ink:#12161c; --muted:#5a6672; --faint:#8a95a1;
    --line:#dfe4ea; --grid:#eceff3; --accent:#0ca6c4;
    --good:#1f9d55; --bad:#e03131; --ref:#e8850c; --shade:rgba(12,166,196,.06);
  }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; background:var(--bg); color:var(--ink);
    font-family: system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; line-height:1.5; }}
  .wrap {{ max-width:980px; margin:0 auto; padding:32px 22px 56px; }}
  .eyebrow {{ font:600 12px/1 ui-monospace,SFMono-Regular,Menlo,monospace;
    letter-spacing:.14em; text-transform:uppercase; color:var(--accent); }}
  h1 {{ font-size:27px; font-weight:680; letter-spacing:-.01em; margin:12px 0 6px;
    text-wrap:balance; }}
  .sub {{ color:var(--muted); max-width:64ch; margin:0 0 24px; }}
  .stats {{ display:flex; flex-wrap:wrap; gap:12px; margin:0 0 22px; }}
  .stat {{ flex:1 1 150px; background:var(--panel); border:1px solid var(--line);
    border-radius:10px; padding:13px 15px; }}
  .stat .k {{ font:600 11px/1 ui-monospace,monospace; letter-spacing:.08em;
    text-transform:uppercase; color:var(--faint); }}
  .stat .v {{ font:700 23px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;
    margin-top:7px; font-variant-numeric:tabular-nums; }}
  .stat .d {{ font-size:12.5px; color:var(--muted); margin-top:4px; }}
  .stat.good .v {{ color:var(--good); }} .stat.bad .v {{ color:var(--bad); }}
  .card {{ background:var(--panel); border:1px solid var(--line); border-radius:14px;
    padding:18px 18px 10px; }}
  .chart-wrap {{ overflow-x:auto; }}
  svg {{ display:block; width:100%; height:auto; min-width:640px; }}
  .grid {{ stroke:var(--grid); stroke-width:1; }}
  .ax {{ fill:var(--faint); font:500 11px ui-monospace,monospace;
    font-variant-numeric:tabular-nums; }}
  .axttl {{ fill:var(--muted); font:600 11px ui-monospace,monospace;
    letter-spacing:.06em; text-transform:uppercase; }}
  .lgd {{ display:flex; flex-wrap:wrap; gap:18px; padding:12px 4px 6px; }}
  .lgd span {{ display:inline-flex; align-items:center; gap:8px; font-size:13px;
    color:var(--muted); }}
  .sw {{ width:20px; height:0; border-top-width:3px; border-top-style:solid; border-radius:2px; }}
  .read {{ margin-top:26px; display:grid; gap:14px; }}
  .read p {{ margin:0; max-width:70ch; }}
  .read b {{ color:var(--ink); }}
  .tag {{ font:600 12px ui-monospace,monospace; padding:2px 7px; border-radius:5px;
    background:var(--shade); color:var(--accent); }}
  .foot {{ margin-top:28px; color:var(--faint); font:500 12px ui-monospace,monospace; }}
</style>

<div class="wrap">
  <div class="eyebrow">Bench replay · 9 m left hairpin · {B-A:.1f}s driving window</div>
  <h1>The stepper obeys the command. The wheels don&rsquo;t.</h1>
  <p class="sub">Three steering channels from the uDV, one run. The commanded angle and the
  stepper position stay locked together the whole corner — but the physical wheel angle from
  the LWS sensor peels off and pins at the mechanical limit. The controller is not the problem.</p>

  <div class="stats">
    <div class="stat good"><div class="k">Stepper vs command</div>
      <div class="v">3.7&deg;</div><div class="d">mean |motor &minus; target| — tracks faithfully</div></div>
    <div class="stat bad"><div class="k">Physical vs command</div>
      <div class="v">22.9&deg;</div><div class="d">mean |actual &minus; target| — decoupled</div></div>
    <div class="stat bad"><div class="k">LWS saturates at</div>
      <div class="v">+63.5&deg;</div><div class="d">pinned from ~t=12s to emergency</div></div>
  </div>

  <div class="card">
    <div class="chart-wrap">
    <svg viewBox="0 0 {W} {H}" role="img" aria-label="Steering angle vs time: target, stepper, and physical LWS angle">
      <rect x="{drive_x:.1f}" y="{MT}" width="{drive_w:.1f}" height="{H-MT-MB}" fill="var(--shade)"/>
      {xgrid}{ygrid}
      <line class="grid" x1="{ML}" y1="{sy(0):.1f}" x2="{W-MR}" y2="{sy(0):.1f}" style="stroke:var(--line)"/>
      <line x1="{ML}" y1="{sat_y:.1f}" x2="{W-MR}" y2="{sat_y:.1f}"
        stroke="var(--bad)" stroke-width="1" stroke-dasharray="3 4" opacity=".55"/>
      <text class="ax" x="{W-MR-4}" y="{sat_y-6:.1f}" text-anchor="end" style="fill:var(--bad)">LWS limit +63.5&deg;</text>
      <line x1="{onset_x:.1f}" y1="{MT}" x2="{onset_x:.1f}" y2="{H-MB}"
        stroke="var(--bad)" stroke-width="1" stroke-dasharray="2 3" opacity=".5"/>
      <text class="ax" x="{onset_x+5:.1f}" y="{MT+13}" style="fill:var(--bad)">runaway begins ~{onset:.0f}s</text>
      <line x1="{emerg_x:.1f}" y1="{MT}" x2="{emerg_x:.1f}" y2="{H-MB}" stroke="var(--muted)" stroke-width="1"/>
      <text class="ax" x="{emerg_x-5:.1f}" y="{MT+13}" text-anchor="end" style="fill:var(--muted)">EMERGENCY</text>
      <polyline fill="none" stroke="var(--ref)" stroke-width="2.4" points="{poly('tgt')}"/>
      <polyline fill="none" stroke="var(--good)" stroke-width="2" stroke-dasharray="6 3" points="{poly('mot')}"/>
      <polyline fill="none" stroke="var(--bad)" stroke-width="2.6" points="{poly('act')}"/>
      {ylab}{xlab}
      <text class="axttl" x="{ML-44}" y="{MT-12}">deg</text>
      <text class="axttl" x="{(W)/2:.0f}" y="{H-6}" text-anchor="middle">time — seconds</text>
    </svg>
    </div>
    <div class="lgd">
      <span><i class="sw" style="border-color:var(--ref)"></i>target — commanded angle</span>
      <span><i class="sw" style="border-color:var(--good);border-top-style:dashed"></i>motor — stepper position</span>
      <span><i class="sw" style="border-color:var(--bad)"></i>actual — LWS physical angle</span>
    </div>
  </div>

  <div class="read">
    <p><b>Read it left to right.</b> Through the first seconds the <span class="tag">target</span> and
    <span class="tag">motor</span> lines sit on top of each other — the stepper goes exactly where the
    controller asks (mean error 3.7&deg;). Around <b>t&nbsp;&asymp;&nbsp;12&nbsp;s</b> the
    <span class="tag">actual</span> line breaks upward on its own and slams into the LWS saturation rail,
    staying there while the command swings &plusmn;30&deg; underneath it.</p>
    <p><b>What it means.</b> The command chain and the stepper are healthy — so is the Stanley loop
    (the steering command showed zero limit-cycle sign-flips). The fault is downstream of the stepper:
    the <b>physical wheel angle is decoupled from the stepper count</b>. Either the LWS is mis-reading
    (sensor) or the stepper slipped relative to the rack (mechanical). The bench discriminator is visual —
    watch whether the front wheels are actually at full lock or at the small commanded angle.</p>
  </div>
  <div class="foot">bench_&hellip;hairpin&hellip;231526 &middot; /steering/feedback [actual, target, motor] &middot; dv_age 147 ms (heartbeat healthy) &middot; ended on ts_loss</div>
</div>
"""
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write(html)
print(f"wrote {OUT}  ({len(rows)} rows, driving {A:.1f}-{B:.1f}s, onset~{onset:.1f}s)")
