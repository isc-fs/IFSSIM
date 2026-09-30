"""Capture what each perception stage did to one LiDAR scan, for the sanity plots.

Runs the production ``RealtimeConeDetector.detect`` once with the stage functions
it calls wrapped, so every intermediate result is recorded exactly as production
computed it (nothing is re-implemented):

* ``ransac2``                -> which points were ground (RANSAC inliers)
* ``_tall_column_veto_mask`` -> which above-ground points the veto removed
* ``_bound_dbscan_input``    -> which the DBSCAN memory guard dropped
* ``clustering_separation_rt`` -> DBSCAN labels, and the ground plane
* ``_fit_cluster``           -> every cone fit, with the points it used

The per-cluster gates of ``detect`` are replayed to say why each cluster was kept
or dropped; if the replay ever disagrees with ``detect`` it raises instead of
drawing something wrong. Every point is expressed in the frame the fit runs in:
sensor xy origin, rotated so the fitted ground plane is horizontal and shifted so
it is z = 0 (needs the IFS09 pipeline with the ground-at-z=0 fix).

Used by ``run_perception_benchmark.py`` (writes ``perception_sanity.json`` next
to its results, which IFS-DV-BENCHWEB shows in the perception section) and by
``perception_sanity_plots.py`` (a standalone HTML page, e.g. for real bags).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

FILENAME = "perception_sanity.json"
FORMAT_VERSION = 1

# Ground returns dominate a scan (~94k of 95k in the sim); keep every
# above-ground point but thin the ground to this many for display.
MAX_GROUND_POINTS = 30_000


@dataclass
class ClusterInfo:
    label: int
    points: object  # (N, 3) ground frame, before the floor cull
    fate: str
    accepted: bool = False
    fit: dict | None = None


@dataclass
class StageCapture:
    cropped: object = None  # (N, 3) sensor frame: what RANSAC saw
    ground_mask: object = None  # (N,) bool over ``cropped``
    rotation: object = None  # 3x3
    plane: object = None
    floor_z: float = 0.0  # ground z in the rotated SENSOR frame (-sensor height)
    vetoed: object = None  # (M, 3) ground frame, removed by the tall-column veto
    guard_dropped: object = None  # (K, 3) ground frame, removed by the DBSCAN guard
    labels: object = None
    clustered: object = None  # ground-frame points DBSCAN labelled
    fits: list = field(default_factory=list)  # [(clean_cone, result tuple)]
    clusters: list = field(default_factory=list)
    cones: list = field(default_factory=list)
    n_raw: int = 0
    n_nonfinite: int = 0


def rows_not_in(big, small):
    """Rows of ``big`` absent from ``small`` (exact float match)."""
    import numpy as np

    if len(small) == len(big):
        return big[:0]
    small_set = {r.tobytes() for r in np.ascontiguousarray(small)}
    keep = [r.tobytes() not in small_set for r in np.ascontiguousarray(big)]
    return big[np.asarray(keep, dtype=bool)]


def to_ground_frame(cap: StageCapture, pts):
    """Sensor-frame points -> the frame clustering runs in (ground plane = z=0)."""
    out = pts @ cap.rotation
    out[:, 2] -= cap.floor_z
    return out


def capture(xyz, config=None) -> StageCapture:
    """Run ``detect`` on ``xyz`` with ``config`` (default: the pipeline defaults), recording every stage."""
    import numpy as np

    import cone_detection.cone_detection as cd

    cfg = config or cd.ConeDetectionConfig()
    cap = StageCapture(n_raw=len(xyz), n_nonfinite=int((~np.isfinite(xyz).all(axis=1)).sum()))
    orig = {
        name: getattr(cd, name)
        for name in (
            "ransac2",
            "_tall_column_veto_mask",
            "_bound_dbscan_input",
            "_fit_cluster",
            "clustering_separation_rt",
        )
    }
    veto_in: dict = {}

    def ransac2(A, **kw):
        inliers, coefs = orig["ransac2"](A, **kw)
        mask = np.zeros(len(A), dtype=bool)
        mask[inliers] = True
        cap.cropped = np.asarray(A[:, 1:4])
        cap.ground_mask = mask
        return inliers, coefs

    def veto(xy, height, c):
        keep = orig["_tall_column_veto_mask"](xy, height, c)
        veto_in["keep"] = keep
        return keep

    def guard(data, c):
        out = orig["_bound_dbscan_input"](data, c)
        cap.guard_dropped = rows_not_in(data, out)
        return out

    def fit(clean_cone, c, **kw):
        res = orig["_fit_cluster"](clean_cone, c, **kw)
        cap.fits.append((np.array(clean_cone), res))
        return res

    def sep(data, config=None, **kw):
        labels, clean, coefs = orig["clustering_separation_rt"](data, config, **kw)
        cap.labels, cap.clustered, cap.plane = np.asarray(labels), np.asarray(clean), coefs
        return labels, clean, coefs

    cd.ransac2, cd._tall_column_veto_mask = ransac2, veto
    cd._bound_dbscan_input, cd._fit_cluster = guard, fit
    cd.clustering_separation_rt = sep
    try:
        cap.cones = cd.RealtimeConeDetector(cfg).detect(xyz)
    finally:
        for name, fn in orig.items():
            setattr(cd, name, fn)

    if cap.cropped is None:
        return cap  # nothing left after the range crop
    cap.rotation = cd.ground_rotation_matrix(cap.plane)
    w = np.asarray(cap.plane[1:], dtype=np.float64)
    cap.floor_z = float(np.dot([0.0, 0.0, -cap.plane[0]], w) / np.linalg.norm(w))
    outliers = to_ground_frame(cap, cap.cropped[~cap.ground_mask])
    keep = veto_in.get("keep")
    cap.vetoed = outliers[~keep] if keep is not None else outliers[:0]
    if cap.guard_dropped is None:
        cap.guard_dropped = outliers[:0]
    classify_clusters(cap, cfg)
    return cap


def classify_clusters(cap: StageCapture, cfg) -> None:
    """Replay detect()'s per-cluster gates to label each cluster's fate.

    Fits are matched to clusters in loop order; the count check fails loudly if
    this replay ever drifts from ``RealtimeConeDetector.detect``.
    """
    import numpy as np

    if cap.labels is None or len(cap.labels) == 0:
        return
    fits = iter(cap.fits)
    n_fitted = 0
    accepted_xy = {(round(c[0], 6), round(c[1], 6)) for c in cap.cones}
    for label in np.unique(cap.labels):
        pts = cap.clustered[cap.labels == label]
        info = ClusterInfo(label=int(label), points=pts, fate="")
        clean = pts[pts[:, 2] > cfg.floor_margin_m]
        if len(pts) < cfg.min_cluster_points:
            info.fate = f"dropped: {len(pts)} pts < min {cfg.min_cluster_points}"
        elif len(clean) == 0:
            info.fate = "dropped: all points below floor margin"
        else:
            height = float(clean[:, 2].max() - clean[:, 2].min())
            rng = float(np.hypot(clean[:, 0].mean(), clean[:, 1].mean()))
            if not cfg.cluster_height_min_m <= height <= cfg.cluster_height_max_m:
                info.fate = (
                    f"dropped: height {height:.2f} m outside "
                    f"[{cfg.cluster_height_min_m}, {cfg.cluster_height_max_m}]"
                )
            elif rng > cfg.range_gate_max_m:
                info.fate = f"dropped: range {rng:.1f} m > {cfg.range_gate_max_m}"
            else:
                clean_cone, (a, b, c, d, res_min, res_other, is_big) = next(fits)
                n_fitted += 1
                info.accepted = (round(a, 6), round(b, 6)) in accepted_xy
                info.fit = {
                    "a": a, "b": b, "c": c, "d": d, "res_min": res_min,
                    "res_other": res_other, "is_big": is_big, "clean": clean_cone,
                    "range": rng, "height": height,
                }
                rmse_mm = 1000.0 * float(np.sqrt(res_min)) if np.isfinite(res_min) else float("inf")
                info.fate = (
                    f"accepted ({'big' if is_big else 'small'}, RMSE {rmse_mm:.0f} mm)"
                    if info.accepted
                    else f"dropped: residual RMSE {rmse_mm:.0f} mm > gate "
                    f"{1000 * cfg.residual_gate_mse ** 0.5:.0f} mm"
                )
        cap.clusters.append(info)
    if n_fitted != len(cap.fits):
        raise RuntimeError(
            f"gate replay drifted from detect(): matched {n_fitted} fits, detect ran {len(cap.fits)}"
        )


def template_fits(clean, config=None) -> list[dict]:
    """Fit BOTH FSAE templates to ``clean`` with the pipeline's solver settings (for display)."""
    import cone_detection.cone_fit as cf
    from cone_detection.cone_detection import ConeDetectionConfig

    cfg = config or ConeDetectionConfig()
    out = []
    for kind, c_fix, d_fix in (
        ("small", cf._CONE_SMALL_C, cf._CONE_SMALL_D),
        ("big", cf._CONE_BIG_C, cf._CONE_BIG_D),
    ):
        a, b, c, d, mse = cf.cone_fit_template(
            clean, c_fix, d_fix, solver=cfg.cone_fit_solver, maxiter=cfg.template_fit_maxiter
        )
        out.append({"kind": kind, "a": a, "b": b, "c": c, "d": d, "mse": mse})
    return out


def pipeline_path(fit: dict) -> str:
    """Which branch of ``_fit_cluster`` produced the pipeline's result."""
    if math.isnan(fit["res_other"]):
        return "collinear"
    if math.isinf(fit["res_other"]):
        return "one template (height early exit)"
    return "both templates"


# ---------------------------------------------------------------------------
# serialisation
# ---------------------------------------------------------------------------


def _num(v, nd: int = 4):
    """Finite float rounded to ``nd`` decimals, else None (JSON has no NaN)."""
    v = float(v)
    return round(v, nd) if math.isfinite(v) else None


def _rmse_mm(mse) -> float | None:
    mse = float(mse)
    return round(1000.0 * math.sqrt(mse), 1) if math.isfinite(mse) and mse >= 0 else None


def to_record(cap: StageCapture, config=None, *, meta: dict | None = None,
              max_ground_points: int = MAX_GROUND_POINTS) -> dict:
    """JSON-ready record: one row per point (role, cluster, used by the fit) and one per cluster."""
    import numpy as np

    from cone_detection.cone_detection import ConeDetectionConfig

    cfg = config or ConeDetectionConfig()
    rec: dict = {"version": FORMAT_VERSION, "scan": dict(meta or {})}
    if cap.cropped is None:
        rec["scan"].update(n_raw=cap.n_raw, n_nonfinite=cap.n_nonfinite, n_input=0)
        rec["points"] = {k: [] for k in ("x", "y", "z", "role", "cluster", "fit_used")}
        rec["clusters"] = []
        return rec

    ground = to_ground_frame(cap, cap.cropped[cap.ground_mask])
    stride = max(1, math.ceil(len(ground) / max_ground_points)) if max_ground_points > 0 else 1
    parts = [(ground[::stride], "ground", -1, False)]
    for info in cap.clusters:
        pts = np.asarray(info.points)
        used = (pts[:, 2] > cfg.floor_margin_m) if info.fit is not None else np.zeros(len(pts), bool)
        parts.append((pts[used], "above", info.label, True))
        parts.append((pts[~used], "above", info.label, False))
    parts.append((cap.vetoed, "veto", -1, False))
    parts.append((cap.guard_dropped, "guard", -1, False))

    x, y, z, role, cluster, fit_used = [], [], [], [], [], []
    for pts, r, lab, f in parts:
        pts = np.round(np.asarray(pts, dtype=np.float64).reshape(-1, 3), 3)
        x += pts[:, 0].tolist(); y += pts[:, 1].tolist(); z += pts[:, 2].tolist()
        role += [r] * len(pts); cluster += [lab] * len(pts); fit_used += [f] * len(pts)
    rec["points"] = {"x": x, "y": y, "z": z, "role": role, "cluster": cluster, "fit_used": fit_used}

    clusters = []
    for info in cap.clusters:
        row: dict = {
            "label": info.label, "n_points": int(len(info.points)), "fate": info.fate,
            "accepted": bool(info.accepted), "fitted": info.fit is not None,
        }
        if info.fit is not None:
            f = info.fit
            row.update(
                range_m=_num(f["range"], 3), height_m=_num(f["height"], 3),
                path=pipeline_path(f), chosen="big" if f["is_big"] else "small",
                a=_num(f["a"]), b=_num(f["b"]), c=_num(f["c"]), d=_num(f["d"]),
                rmse_mm=_rmse_mm(f["res_min"]),
                templates={
                    t["kind"]: {
                        "a": _num(t["a"]), "b": _num(t["b"]), "c": _num(t["c"]),
                        "d": _num(t["d"]), "rmse_mm": _rmse_mm(t["mse"]),
                    }
                    for t in template_fits(f["clean"], cfg)
                },
            )
        clusters.append(row)
    rec["clusters"] = clusters

    rec["scan"].update(
        n_raw=cap.n_raw,
        n_nonfinite=cap.n_nonfinite,
        n_input=int(len(cap.cropped)),
        n_ground=int(cap.ground_mask.sum()),
        ground_stride=stride,
        n_clustered=int(len(cap.clustered)) if cap.clustered is not None else 0,
        n_vetoed=int(len(cap.vetoed)),
        n_guard_dropped=int(len(cap.guard_dropped)),
        n_clusters=sum(1 for c in cap.clusters if c.label != -1),
        n_fitted=len(cap.fits),
        n_accepted=len(cap.cones),
        sensor_height_m=_num(-cap.floor_z, 3),
        floor_margin_m=cfg.floor_margin_m,
        residual_gate_rmse_mm=round(1000.0 * math.sqrt(cfg.residual_gate_mse), 1),
    )
    return rec


def write(run_dir: Path, xyz, config=None, *, meta: dict | None = None) -> Path:
    """Capture ``xyz`` and write ``perception_sanity.json`` into ``run_dir``."""
    path = Path(run_dir) / FILENAME
    rec = to_record(capture(xyz, config), config, meta=meta)
    path.write_text(json.dumps(rec, separators=(",", ":")))
    return path
