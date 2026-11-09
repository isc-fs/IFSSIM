"""What the car should do (from settings.json) and what it did (from traces).

Everything here is a pure function over plain dicts and lists, standard
library only: the GPU box's Python has no numpy, and the unit tests
(tools/tests/test_characterise.py) drive these with synthetic traces.

Frames and reference points
---------------------------
A trace sample is ``{"t", "x", "y", "z", "vx", "vy", "wz", "yaw"}``: sim time
(s), the body origin's position and velocity in the world ENU frame (m, m/s),
the yaw rate about ENU z (rad/s, + = left) and the heading (rad, ENU, from the
ground-truth quaternion). Since #757 the body origin IS the centre of mass:
the front axle sits ``L*(1-WeightDistFront)`` ahead of it and the rear axle
``b = L*WeightDistFront`` behind it.

* Radii are of the body origin (centre of mass). The kinematic bicycle's
  rear-axle radius is ``L/tan(delta)``; the CoM sits ``b`` ahead of the rear
  axle on that radius's normal, so its radius is ``hypot(L/tan(delta), b)``.
  That is the expected radius.
* Curvature and yaw gain use ``u``, the velocity along the heading. ``u`` is
  the same at every point on the car's centreline (the rigid body's
  velocities differ only across it), and the kinematic bicycle says
  ``r = u*tan(delta)/L`` exactly, whatever the reference point.
"""

from __future__ import annotations

import math
import re
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

G = 9.81          # m/s^2
RHO_AIR = 1.225   # kg/m^3, sea-level ISA: a physical constant, not a car number

# Tolerances of the hard checks. Kinematic quantities only: the two bugs this
# exists for were a 1.38x radius error (wheelbase) and a 1.23x one (lock).
TOL_RADIUS = 0.03          # relative, full-lock radius
TOL_STEER_EXPONENT = 0.10  # absolute, on n in curvature ~ tan(delta)^n (squared input: n ~ 2)
TOL_YAW_GAIN = 0.05        # relative, r / (u * tan(delta)) against 1/L
TOL_WHEELBASE_LOG = 0.005  # relative, the solver's wheelbase from the sim log


# ------------------------------------------------------------------ settings -> expectations
def vehicle_physics(settings: dict, vehicle: str = "FSCar") -> dict:
    """The ``VehiclePhysics`` block of a settings.json dict (or the block itself)."""
    if "Vehicles" in settings:
        vehicles = settings["Vehicles"] or {}
        v = vehicles.get(vehicle) or next(
            (x for x in vehicles.values() if isinstance(x, dict) and "VehiclePhysics" in x), None)
        if not v or "VehiclePhysics" not in v:
            raise KeyError(f"no Vehicles.{vehicle}.VehiclePhysics in settings")
        return v["VehiclePhysics"]
    return settings


def expectations(settings: dict, vehicle: str = "FSCar") -> Dict[str, Optional[float]]:
    """Every number a check compares with, derived from settings.json.

    Wheelbase, WeightDistFront, MaxSteerAngle and Mass are required (the hard
    checks need them); the rest only feed the reported (info) checks and are
    None when absent.
    """
    p = vehicle_physics(settings, vehicle)
    missing = [k for k in ("Wheelbase", "WeightDistFront", "MaxSteerAngle", "Mass") if k not in p]
    if missing:
        raise KeyError(f"settings.json VehiclePhysics lacks {', '.join(missing)}")
    L = float(p["Wheelbase"])
    wdf = float(p["WeightDistFront"])
    dmax = math.radians(float(p["MaxSteerAngle"]))
    m = float(p["Mass"])
    b = L * wdf            # CoM to rear axle
    lf = L - b             # CoM to front axle
    mu, h = p.get("TireMu"), p.get("CoGHeight")
    crr, cda, cla = p.get("RollingResistance"), p.get("CdA"), p.get("ClA")

    out: Dict[str, Optional[float]] = {
        "wheelbase_m": L,
        "front_weight": wdf,
        "max_steer_deg": float(p["MaxSteerAngle"]),
        "mass_kg": m,
        "cog_to_rear_m": b,
        "cog_to_front_m": lf,
        "full_lock_radius_rear_m": L / math.tan(dmax),
        "full_lock_radius_cog_m": turn_radius_cog(L, b, dmax),
        "yaw_gain_per_m": 1.0 / L,
        "steer_exponent": 1.0,
        "coast_a0": None, "coast_b": None, "ebs_decel": None, "launch_accel_max": None,
    }
    if crr is not None:
        out["coast_a0"] = float(crr) * G
        if cda is not None:
            # Drag, plus rolling resistance on the downforce (Crr acts on Fz).
            out["coast_b"] = RHO_AIR / (2 * m) * (float(cda) + float(crr) * float(cla or 0.0))
    if mu is not None and h is not None:
        mu, h = float(mu), float(h)
        # EBS clamps the rear axle only; braking moves load off it:
        #   a = mu * N_r / m,  N_r = m*(g*lf - a*h)/L  ->  a = mu*g*lf / (L + mu*h)
        out["ebs_decel"] = mu * G * lf / (L + mu * h)
        # Rear-wheel drive, launch moves load onto it: a = mu*g*lf / (L - mu*h)
        if L - mu * h > 0:
            out["launch_accel_max"] = mu * G * lf / (L - mu * h)
    return out


def turn_radius_cog(L: float, b: float, delta_rad: float) -> float:
    """Kinematic radius of the point ``b`` ahead of the rear axle, steer ``delta``."""
    return math.hypot(L / math.tan(abs(delta_rad)), b)


# ------------------------------------------------------------------ small numerics
def mean(xs: Sequence[float]) -> float:
    return sum(xs) / len(xs)


def linfit(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    """Least-squares line: (slope, intercept)."""
    n = len(xs)
    if n < 2:
        raise ValueError("linfit needs two points")
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        raise ValueError("linfit: all x equal")
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope, my - slope * mx


def _solve3(a: List[List[float]], v: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting, 3x3."""
    m = [row[:] + [v[i]] for i, row in enumerate(a)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("singular system (points on a line?)")
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, 3):
            f = m[r][c] / m[c][c]
            for k in range(c, 4):
                m[r][k] -= f * m[c][k]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][k] * x[k] for k in range(r + 1, 3))) / m[r][r]
    return x


def fit_circle(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float, float]:
    """Algebraic (Kasa) circle fit: (cx, cy, radius, rms radial residual).

    Centred first, so a circle 50 m from the origin fits as well as one at it.
    Good on half a turn or more; on short arcs it is biased small under noise.
    """
    if len(xs) < 3:
        raise ValueError("fit_circle needs three points")
    mx, my = mean(xs), mean(ys)
    us = [x - mx for x in xs]
    vs = [y - my for y in ys]
    # u^2 + v^2 = 2*cu*u + 2*cv*v + c
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for u, v in zip(us, vs):
        row = (2 * u, 2 * v, 1.0)
        rhs = u * u + v * v
        for i in range(3):
            atb[i] += row[i] * rhs
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    cu, cv, c = _solve3(ata, atb)
    r = math.sqrt(max(c + cu * cu + cv * cv, 0.0))
    rms = math.sqrt(mean([(math.hypot(u - cu, v - cv) - r) ** 2 for u, v in zip(us, vs)]))
    return cu + mx, cv + my, r, rms


def unwrap(angles: Sequence[float]) -> List[float]:
    out: List[float] = []
    for a in angles:
        if out:
            d = a - out[-1]
            d = (d + math.pi) % (2 * math.pi) - math.pi
            out.append(out[-1] + d)
        else:
            out.append(a)
    return out


def yaw_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
    """Heading about ENU z of a body-FLU-to-ENU quaternion."""
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def speed(s: dict) -> float:
    return math.hypot(s["vx"], s["vy"])


def body_u(s: dict) -> float:
    """Velocity along the heading (same at every point on the centreline)."""
    return s["vx"] * math.cos(s["yaw"]) + s["vy"] * math.sin(s["yaw"])


def path_length(samples: Sequence[dict]) -> List[float]:
    out = [0.0]
    for a, b in zip(samples, samples[1:]):
        out.append(out[-1] + math.hypot(b["x"] - a["x"], b["y"] - a["y"]))
    return out


def _integrate(ts: Sequence[float], ys: Sequence[float]) -> float:
    return sum(0.5 * (ys[i] + ys[i + 1]) * (ts[i + 1] - ts[i]) for i in range(len(ts) - 1))


# ------------------------------------------------------------------ manoeuvre summaries
def turn_summary(samples: Sequence[dict], steer: float, max_steer_rad: float) -> dict:
    """A steady turn at a fixed steer command.

    kappa: heading change over distance along the heading (dpsi / integral of u dt),
    signed (+ = left). gyro_gain: mean of r/(u*tan(delta)) from the yaw-rate channel,
    sample by sample. radius_m: circle fit of the body origin's path.
    """
    if len(samples) < 10:
        raise ValueError(f"turn at steer {steer:+.2f}: only {len(samples)} samples")
    ts = [s["t"] for s in samples]
    yaw = unwrap([s["yaw"] for s in samples])
    us = [body_u(s) for s in samples]
    dist_u = _integrate(ts, us)
    dpsi = yaw[-1] - yaw[0]
    delta = abs(steer) * max_steer_rad
    tan_d = math.tan(delta)
    gains = [abs(s["wz"]) / (u * tan_d) for s, u in zip(samples, us) if u > 0.3 and tan_d > 0]
    out = {
        "steer": steer,
        "delta_deg": math.degrees(delta),
        "n": len(samples),
        "duration_s": ts[-1] - ts[0],
        "arc_deg": math.degrees(dpsi),
        "turn": "left" if dpsi > 0 else "right",
        "v_mean": mean([speed(s) for s in samples]),
        "u_mean": mean(us),
        "kappa": dpsi / dist_u if dist_u > 0 else None,
        "gyro_gain": mean(gains) if gains else None,
        "radius_m": None, "radius_rms_m": None,
    }
    if abs(dpsi) >= math.radians(90):  # a circle fit needs a real arc
        _, _, r, rms = fit_circle([s["x"] for s in samples], [s["y"] for s in samples])
        out["radius_m"], out["radius_rms_m"] = r, rms
        out["a_lat"] = out["v_mean"] ** 2 / r if r > 0 else None
    return out


def steer_exponent(points: Iterable[Tuple[float, float]], max_steer_rad: float) -> Optional[float]:
    """n in |kappa| = k * tan(delta)^n, from (steer command, kappa) points.

    1 for a car whose road-wheel angle is linear in the command; ~2 when the
    input is squared on the way (Chaos's SquaredFunction).
    """
    pts = [(abs(s), abs(k)) for s, k in points if s and k]
    if len(pts) < 2 or len({s for s, _ in pts}) < 2:
        return None
    xs = [math.log(math.tan(s * max_steer_rad)) for s, _ in pts]
    ys = [math.log(k) for _, k in pts]
    return linfit(xs, ys)[0]


def time_to_distance(samples: Sequence[dict], distance_m: float) -> Optional[dict]:
    """From the first sample (the launch), when the path first reaches ``distance_m``."""
    cum = path_length(samples)
    for i in range(1, len(samples)):
        if cum[i] >= distance_m:
            f = (distance_m - cum[i - 1]) / (cum[i] - cum[i - 1]) if cum[i] > cum[i - 1] else 1.0
            t = samples[i - 1]["t"] + f * (samples[i]["t"] - samples[i - 1]["t"])
            v = speed(samples[i - 1]) + f * (speed(samples[i]) - speed(samples[i - 1]))
            return {"time_s": t - samples[0]["t"], "speed_mps": v, "distance_m": distance_m}
    return None


def coast_fit(samples: Sequence[dict], chord_s: float = 0.5) -> Optional[dict]:
    """Deceleration while coasting, fitted as a(v) = a0 + b*v^2 over chords of ``chord_s``."""
    if len(samples) < 3:
        return None
    ts = [s["t"] for s in samples]
    vs = [speed(s) for s in samples]
    v2, acc = [], []
    j = 0
    for i in range(len(samples)):
        while j < len(samples) and ts[j] - ts[i] < chord_s:
            j += 1
        if j >= len(samples):
            break
        v2.append(((vs[i] + vs[j]) / 2) ** 2)
        acc.append(-(vs[j] - vs[i]) / (ts[j] - ts[i]))
    if len(acc) < 2:
        return None
    out = {
        "v_start": vs[0], "v_end": vs[-1], "duration_s": ts[-1] - ts[0],
        "decel_mean": (vs[0] - vs[-1]) / (ts[-1] - ts[0]),
        "v2_mean": mean([v * v for v in vs]),
        "a0": None, "b": None,
    }
    try:
        b, a0 = linfit(v2, acc)
        out["a0"], out["b"] = a0, b
    except ValueError:
        pass
    return out


def stop_summary(samples: Sequence[dict], v_stop: float = 0.1) -> Optional[dict]:
    """From the trigger (first sample) to standstill: distance, time, mean decel."""
    if len(samples) < 2:
        return None
    cum = path_length(samples)
    v0 = speed(samples[0])
    for i, s in enumerate(samples):
        if i and speed(s) < v_stop:
            t = s["t"] - samples[0]["t"]
            return {"v0": v0, "distance_m": cum[i], "time_s": t,
                    "decel_mean": v0 / t if t > 0 else None, "stopped": True}
    return {"v0": v0, "distance_m": cum[-1], "time_s": samples[-1]["t"] - samples[0]["t"],
            "decel_mean": None, "stopped": False}


def heading_error_deg(samples: Sequence[dict], v_min: float = 2.0) -> Optional[float]:
    """Mean angle between the velocity and the heading while driving straight."""
    errs = []
    for s in samples:
        if speed(s) >= v_min:
            d = math.atan2(s["vy"], s["vx"]) - s["yaw"]
            errs.append(abs((d + math.pi) % (2 * math.pi) - math.pi))
    return math.degrees(mean(errs)) if errs else None


_WB_OK = re.compile(r"FSDS: physics wheelbase ([0-9.]+) cm, ([0-9.]+)% of the weight on the front axle")
_WB_BAD = re.compile(r"FSDS: physics wheels are NOT where settings\.json puts them.*?wheelbase ([0-9.]+) cm")


def parse_wheelbase_log(text: str) -> Optional[dict]:
    """The last wheel-placement line the sim logged (VerifyWheelPlacement), or None."""
    last = None
    for m in _WB_OK.finditer(text):
        last = (m.start(), {"wheelbase_m": float(m.group(1)) / 100.0,
                            "front_weight": float(m.group(2)) / 100.0, "placed_ok": True})
    for m in _WB_BAD.finditer(text):
        if last is None or m.start() > last[0]:
            last = (m.start(), {"wheelbase_m": float(m.group(1)) / 100.0,
                                "front_weight": None, "placed_ok": False})
    return last[1] if last else None


# ------------------------------------------------------------------ checks
def _check(cid: str, name: str, unit: str, expected: Optional[float], measured: Optional[float],
           tol: Optional[float] = None, rel: bool = True, hard: bool = False, digits: int = 2,
           band: str = "", note: str = "") -> dict:
    if not band and tol is not None:
        band = f"±{tol * 100:g}%" if rel else f"±{tol:g}"
    c = {"id": cid, "name": name, "unit": unit, "expected": expected, "measured": measured,
         "tolerance": tol, "relative": rel, "band": band, "hard": hard, "digits": digits,
         "note": note}
    if measured is None:
        c["status"] = "error" if hard else "info"
        c["note"] = note or "not measured"
    elif not hard or expected is None or tol is None:
        c["status"] = "info"
    else:
        err = (measured - expected) / expected if rel else measured - expected
        c["error"] = err
        c["status"] = "pass" if abs(err) <= tol else "fail"
    return c


def evaluate(exp: dict, meas: dict) -> dict:
    """Checks and summaries from the expectations and the raw measurements.

    ``meas`` (written by the driver):
      turns:    [{"steer": s, "samples": [...]}]   steady turns, fixed steer, ~2 m/s
      launch:   {"throttle", "distance_m", "samples"}  from rest
      coast:    {"samples"}                            throttle and regen 0
      ebs:      {"samples"}                            from the activateEbs call
      wheelbase_log: parse_wheelbase_log(sim log) or None
    """
    dmax = math.radians(exp["max_steer_deg"])
    L = exp["wheelbase_m"]
    checks: List[dict] = []
    summary: Dict[str, object] = {}

    wb = meas.get("wheelbase_log")
    checks.append(_check(
        "physics_wheelbase", "Physics wheelbase (sim log)", "m", L,
        wb["wheelbase_m"] if wb else None, TOL_WHEELBASE_LOG, hard=wb is not None, digits=3,
        note="" if wb else "no wheel-placement line in the sim log"))
    if wb and wb.get("front_weight") is not None:
        checks.append(_check("front_weight", "Front weight share (sim log)", "", exp["front_weight"],
                             wb["front_weight"], digits=3))

    turns = []
    for tr in meas.get("turns") or []:
        try:
            turns.append(turn_summary(tr["samples"], tr["steer"], dmax))
        except ValueError as e:
            turns.append({"steer": tr["steer"], "error": str(e)})
    summary["turns"] = turns
    ok_turns = [t for t in turns if "error" not in t]

    locks = [t for t in ok_turns if abs(t["steer"]) >= 0.999]
    for t in locks:
        checks.append(_check(
            f"full_lock_radius_{t['turn']}", f"Full-lock radius, {t['turn']} (steer {t['steer']:+.0f})",
            "m", exp["full_lock_radius_cog_m"], t["radius_m"], TOL_RADIUS, hard=True,
            note=f"centre of mass; rear axle {math.sqrt(max(t['radius_m'] ** 2 - exp['cog_to_rear_m'] ** 2, 0)):.2f} m"
                 if t["radius_m"] else "arc too short to fit"))
    for side in ("left", "right"):
        if not any(t["turn"] == side for t in locks):
            checks.append(_check(f"full_lock_radius_{side}", f"Full-lock radius, {side}", "m",
                                 exp["full_lock_radius_cog_m"], None, TOL_RADIUS, hard=True,
                                 note="no full-lock turn to this side"))

    n = steer_exponent([(t["steer"], t["kappa"]) for t in ok_turns if t["kappa"]], dmax)
    levels = sorted({abs(t["steer"]) for t in ok_turns if t["kappa"]})
    checks.append(_check(
        "steer_linearity", "Steering linearity: n in κ ∝ tan(δ)ⁿ", "", 1.0, n,
        TOL_STEER_EXPONENT, rel=False, hard=True, digits=2,
        note=f"steer {', '.join(f'{s:g}' for s in levels)}; a squared input gives ~2"))

    gains = [t["gyro_gain"] for t in ok_turns if t["gyro_gain"]]
    checks.append(_check(
        "yaw_gain", "Low-speed yaw gain r/(u·tanδ)", "1/m", exp["yaw_gain_per_m"],
        mean(gains) if gains else None, TOL_YAW_GAIN, hard=True, digits=3,
        note=f"1/L; mean of {len(gains)} turns, from the yaw-rate channel"))

    pos = [t for t in ok_turns if t["steer"] > 0]
    if pos:
        checks.append({"id": "steer_sign", "name": "Steering sign", "unit": "", "expected": None,
                       "measured": None, "hard": False, "status": "info", "band": "",
                       "text": f"+ turns {pos[0]['turn']}", "note": "ENU yaw from the ground truth"})

    launch = meas.get("launch") or {}
    ttd = time_to_distance(launch.get("samples") or [], launch.get("distance_m", 30.0))
    summary["launch"] = ttd
    d = launch.get("distance_m", 30.0)
    a_max = exp.get("launch_accel_max")
    checks.append(_check(
        "launch_time", f"Launch, {d:g} m at throttle {launch.get('throttle', 1.0):g}", "s",
        math.sqrt(2 * d / a_max) if a_max else None, ttd["time_s"] if ttd else None, band="≥",
        note=(f"{ttd['speed_mps']:.1f} m/s at {d:g} m; expected is the traction limit"
              if ttd else "never covered the distance")))
    he = heading_error_deg(launch.get("samples") or [])
    if he is not None:
        checks.append(_check("heading_vs_velocity", "Heading vs velocity, straight", "deg", 0.0, he,
                             digits=1, note="ground-truth quaternion against velocity"))

    coast = coast_fit((meas.get("coast") or {}).get("samples") or [])
    summary["coast"] = coast
    exp_coast = None
    if coast and exp.get("coast_a0") is not None and exp.get("coast_b") is not None:
        exp_coast = exp["coast_a0"] + exp["coast_b"] * coast["v2_mean"]
    checks.append(_check(
        "coast_decel", "Coast-down deceleration", "m/s²", exp_coast,
        coast["decel_mean"] if coast else None, digits=3,
        note=(f"{coast['v_start']:.1f}→{coast['v_end']:.1f} m/s; fit a0 "
              + (f"{coast['a0']:.3f}, b {coast['b']:.5f}" if coast["a0"] is not None else "–")
              + (f" (settings {exp['coast_a0']:.3f}, {exp['coast_b']:.5f})" if exp.get("coast_b") else ""))
        if coast else "no coast trace"))

    ebs = stop_summary((meas.get("ebs") or {}).get("samples") or [])
    summary["ebs"] = ebs
    exp_ebs = (ebs["v0"] ** 2 / (2 * exp["ebs_decel"])) if ebs and exp.get("ebs_decel") else None
    checks.append(_check(
        "ebs_distance", "EBS stopping distance", "m", exp_ebs,
        ebs["distance_m"] if ebs and ebs["stopped"] else None,
        note=(f"from {ebs['v0']:.1f} m/s in {ebs['time_s']:.2f} s" + ("" if ebs["stopped"] else ", never stopped"))
        if ebs else "no EBS trace"))

    return {"checks": checks, "summary": summary, "status": overall(checks)}


def overall(checks: Sequence[dict]) -> str:
    """fail: a hard check failed. error: none failed, but one couldn't be measured. Else pass."""
    hard = [c for c in checks if c.get("hard")]
    if any(c["status"] == "fail" for c in hard):
        return "fail"
    if any(c["status"] == "error" for c in hard):
        return "error"
    return "pass"


# ------------------------------------------------------------------ markdown
STATUS_CELL = {"pass": "✅ pass", "fail": "❌ **fail**", "error": "⚠️ not measured", "info": "info"}
HEADLINE = {"pass": "passed", "fail": "failed", "error": "incomplete"}


def _fmt(v: Optional[float], digits: int, unit: str) -> str:
    if v is None:
        return "–"
    return f"{v:.{digits}f} {unit}".strip()


def markdown(result: dict) -> str:
    """The "Car characterisation" section of the PR comment."""
    status = result.get("status", "error")
    head = f"#### Car characterisation · {HEADLINE.get(status, status)}"
    if result.get("error"):
        return f"{head}\n\n> {result['error']}\n"
    rows = ["| Check | Expected | Measured | Result |", "|---|---|---|---|"]
    for c in result.get("checks", []):
        if c.get("text"):
            exp_cell, meas_cell = "–", c["text"]
        else:
            exp_cell = _fmt(c.get("expected"), c.get("digits", 3), c.get("unit", ""))
            band = c.get("band") or ""
            if exp_cell != "–" and band:
                exp_cell = f"{exp_cell} {band}" if band.startswith("±") else f"{band} {exp_cell}"
            meas_cell = _fmt(c.get("measured"), c.get("digits", 3), c.get("unit", ""))
        note = f" — {c['note']}" if c.get("note") else ""
        name = c["name"] if c.get("hard") else f"{c['name']} *(info)*"
        rows.append(f"| {name}{note} | {exp_cell} | {meas_cell} | {STATUS_CELL.get(c['status'], c['status'])} |")
    clock = result.get("clock", {})
    foot = ("Scripted inputs over RPC, no pipeline; expectations derived from `settings.json`. "
            "Radii are of the body origin (the centre of mass), against √((L/tanδ)² + b²). ")
    if clock:
        foot += f"Clock: {clock.get('mode')}"
        if clock.get("rtf") is not None:
            foot += f", {clock['rtf']:.2f}× real time"
        if clock.get("sim_s") is not None:
            foot += f", {clock['sim_s']:.0f} s of sim time"
        foot += "."
    return "\n".join([head, "", *rows, "", foot, ""])
