#!/usr/bin/env python3
"""Offline longitudinal-control tuning harness.

Drives the REAL PIVelocity controller (pipeline/control) closed-loop against a
longitudinal plant built from the sim's own vehicle parameters (settings.json).
No ROS, no hardware — deterministic, runs anywhere Python does.

Purpose: tune the throttle/regen loop for smooth tracking WITHOUT the
"spikes / intermittent throttle" the real car shows. The controller's own
docstring pins that failure mode on a spurious single-tick v_meas glitch fed
raw into kp; this harness reproduces it and scores how well the conditioning
(v_meas_tau, v_meas_max_accel) + gains (kp, ki, deadband) reject it.

Two things it measures:
  A. Step tracking   — accel 0→v and decel v→0: rise time, overshoot, ss error.
  B. Glitch rejection — inject a single-tick v_meas dropout mid-cruise and count
     the resulting throttle spike. This is the real-car symptom.

Plant is a transparent first-order longitudinal model (motor force − drag −
roll, grip-limited). Absolute dynamics are approximate until the bench
throttle→rpm sweep pins them; RELATIVE gain comparisons (which config is
smoother) are valid regardless.
"""
from __future__ import annotations
import sys, os, math, itertools

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "pipeline", "control"))
from control.controllers.pi_velocity import PIVelocity  # noqa: E402

DT = 1.0 / 40.0  # matches PIVelocity._DT


# ------------------------------------------------------------------ plant
class LongPlant:
    """First-order longitudinal plant from settings.json vehicle params.

    v_dot = a_drive·throttle − a_regen·regen − (F_drag + F_roll)/m
    Traction-limited so |a_drive·throttle| ≤ TireMu·g.
    """
    def __init__(self,
                 mass=275.0, motor_torque_nm=230.0, gear=2.909, wheel_r=0.202,
                 drivetrain_eff=0.92, cda=0.9, rho=1.225, crr=0.02,
                 tire_mu=1.4, g=9.81,
                 regen_torque_nm=110.0):   # EMRAX battery-limited regen (approx)
        f_drive_max = motor_torque_nm * gear * drivetrain_eff / wheel_r
        self.a_drive_max = min(f_drive_max / mass, tire_mu * g)
        f_regen_max = regen_torque_nm * gear * drivetrain_eff / wheel_r
        self.a_regen_max = min(f_regen_max / mass, tire_mu * g)
        self.mass, self.cda, self.rho, self.crr, self.g = mass, cda, rho, crr, g
        self.v = 0.0

    def step(self, throttle, regen):
        drag = 0.5 * self.rho * self.cda * self.v * self.v
        roll = self.crr * self.mass * self.g if self.v > 0.01 else 0.0
        a = (self.a_drive_max * throttle
             - self.a_regen_max * regen
             - (drag + roll) / self.mass)
        self.v = max(0.0, self.v + a * DT)   # no reverse on the bench scenarios
        return self.v


# ------------------------------------------------------------- scenarios
def run_step(pi, plant, v_set_fn, T=6.0, glitch_t=None, glitch_v=0.0):
    """Closed loop: plant.v → real conditioning+track → (throttle,regen) → plant.
    Optionally inject a single-tick raw-v_meas glitch at glitch_t."""
    n = int(T / DT)
    ts, vs, sets, us, thr, reg = [], [], [], [], [], []
    for i in range(n):
        t = i * DT
        v_set = v_set_fn(t)
        v_raw = plant.v
        if glitch_t is not None and abs(t - glitch_t) < DT / 2:
            v_raw = glitch_v                      # spurious single-tick sample
        v_meas = pi._condition_velocity(v_raw)    # REAL conditioning
        throttle, regen = pi._track(v_meas, v_set)  # REAL track stage
        plant.step(throttle, regen)
        u = throttle - regen
        ts.append(t); vs.append(plant.v); sets.append(v_set)
        us.append(u); thr.append(throttle); reg.append(regen)
    return dict(t=ts, v=vs, v_set=sets, u=us, thr=thr, reg=reg)


# --------------------------------------------------------------- metrics
def spikes(u, thresh=0.10):
    """Tick-to-tick command jumps > thresh — the 'spikes/intermittent' symptom."""
    return sum(1 for i in range(1, len(u)) if abs(u[i] - u[i-1]) > thresh)

def rise_time(tr):
    target = tr["v_set"][-1]
    if target <= 0: return float("nan")
    for t, v in zip(tr["t"], tr["v"]):
        if v >= 0.9 * target:
            return t
    return float("nan")

def overshoot(tr):
    target = max(tr["v_set"])
    if target <= 0: return 0.0
    return max(0.0, (max(tr["v"]) - target) / target)

def ss_err(tr, tail_s=1.0):
    k = int(tail_s / DT)
    seg = list(zip(tr["v"][-k:], tr["v_set"][-k:]))
    return sum(abs(v - s) for v, s in seg) / len(seg)


def make_pi(**kw):
    base = dict(v_max=3.0, kp=0.5, ki=0.05, deadband=0.2, throttle_max=0.2,
                v_meas_tau=0.15, v_meas_max_accel=20.0)
    base.update(kw)
    return PIVelocity(**base)


def evaluate(cfg):
    """Score one config across accel, decel, and a glitch scenario."""
    accel = run_step(make_pi(**cfg), LongPlant(), lambda t: 3.0, T=6.0)
    dec_pi, dec_plant = make_pi(**cfg), LongPlant(); dec_plant.v = 3.0; dec_pi._v_filt = 3.0
    decel = run_step(dec_pi, dec_plant, lambda t: 0.0 if t > 0.5 else 3.0, T=6.0)
    # cruise at 3 with a single-tick v_meas dropout to 0 at t=2s
    gl_pi, gl_plant = make_pi(**cfg), LongPlant(); gl_plant.v = 3.0; gl_pi._v_filt = 3.0
    glitch = run_step(gl_pi, gl_plant, lambda t: 3.0, T=4.0, glitch_t=2.0, glitch_v=0.0)
    return dict(
        rise=rise_time(accel), overshoot=overshoot(accel),
        ss=ss_err(accel), spikes_accel=spikes(accel["u"]),
        glitch_spike=max(glitch["u"][75:85]) - min(glitch["u"][75:85]),  # around t=2s
        glitch_spikes=spikes(glitch["u"]),
    )


def score(m):
    # lower is better: tracking error, spikes, overshoot, rise, glitch response
    return (2.0 * m["ss"] + 0.5 * m["overshoot"] + 0.2 * m["rise"]
            + 0.3 * m["spikes_accel"] + 3.0 * m["glitch_spike"] + 0.3 * m["glitch_spikes"])


if __name__ == "__main__":
    plant0 = LongPlant()
    print(f"# plant: a_drive_max={plant0.a_drive_max:.2f} m/s²  a_regen_max={plant0.a_regen_max:.2f} m/s²  (throttle_max=0.2 → {0.2*plant0.a_drive_max:.2f} m/s²)\n")

    base = dict(kp=0.5, ki=0.05, deadband=0.2, v_meas_tau=0.15, v_meas_max_accel=20.0)
    m = evaluate(base)
    print("=== BASELINE (current gains) ===")
    print(f"  rise90={m['rise']:.2f}s  overshoot={100*m['overshoot']:.1f}%  ss_err={m['ss']:.3f} m/s")
    print(f"  accel spikes={m['spikes_accel']}   glitch throttle jump={m['glitch_spike']:.3f}  glitch spikes={m['glitch_spikes']}")
    print(f"  score={score(m):.3f}\n")

    # ---- sweep ----
    grid = dict(
        kp=[0.3, 0.5, 0.8],
        ki=[0.05, 0.15, 0.3],
        deadband=[0.05, 0.1, 0.2],
        v_meas_tau=[0.10, 0.15, 0.25],
        v_meas_max_accel=[8.0, 12.0, 20.0],
    )
    keys = list(grid)
    results = []
    for combo in itertools.product(*grid.values()):
        cfg = dict(zip(keys, combo))
        try:
            m = evaluate(cfg); results.append((score(m), cfg, m))
        except Exception:
            pass
    results.sort(key=lambda r: r[0])
    print(f"=== SWEEP: {len(results)} configs, top 5 by score ===")
    for s, cfg, m in results[:5]:
        print(f"  score={s:.3f}  kp={cfg['kp']} ki={cfg['ki']} db={cfg['deadband']} "
              f"tau={cfg['v_meas_tau']} maxacc={cfg['v_meas_max_accel']}  "
              f"| rise={m['rise']:.2f} os={100*m['overshoot']:.0f}% ss={m['ss']:.3f} "
              f"glitch={m['glitch_spike']:.3f} spk={m['glitch_spikes']}")
    print("\n=== worst 3 (for contrast) ===")
    for s, cfg, m in results[-3:]:
        print(f"  score={s:.3f}  kp={cfg['kp']} ki={cfg['ki']} db={cfg['deadband']} "
              f"tau={cfg['v_meas_tau']} maxacc={cfg['v_meas_max_accel']}  glitch={m['glitch_spike']:.3f}")
