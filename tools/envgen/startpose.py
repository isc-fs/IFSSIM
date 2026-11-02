"""Where the sim will spawn the car on a track, computed without the sim.

A line-for-line port of FSDSConeSpawner::ComputeStartGatePose
(Plugins/FSDSPlugin/Source/FSDSPlugin/Private/FSDSConeSpawner.cpp). The CSV
has no start-pose field: the sim derives the pose from the big orange cones.
On a one-gate track it puts blue on the car's left, from the blue and yellow
cones nearest the gate, and only without both colours there falls back to
the four track cones nearest the gate. This port lets the track creator, and
CI, check the pose before the sim ever sees it.

Works in the track CSV's frame (metres; right-handed, +y left of +x). The sim
works in UE centimetres with y mirrored (left-handed, +Y right of +X). PCA and
distances are mirror-invariant and "left" is mirrored with the frame, so the
answer is the same, expressed in the CSV's frame. Keep it in step with the C++.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

GATE_CLUSTER_M = 10.0     # GateClusterDistSq = 1000 cm, squared
K_DIRECTION = 4           # nearest track cones used for the sign (fallback)
K_SIDE = 2                # nearest cones of each colour for the side test
SIDE_RADIUS_M = 8.0       # side-test cones within this of the gate centroid
MIN_SIDE_OFFSET_M = 1.0   # yellow->blue offset needed to trust the side test
BACKUP_M = 3.0            # loadTrack calls ComputeStartGatePose(300 cm)

Cone = Tuple[str, float, float]


@dataclass
class StartPose:
    anchor: Tuple[float, float]     # the gate point the car backs up from
    forward: Tuple[float, float]    # unit vector, the direction the car faces
    spawn: Tuple[float, float]      # anchor - forward * BACKUP_M
    method: str                     # "multi-gate", "pca-sides" or "pca"
    gates: int
    side_offset: Optional[Tuple[float, float]] = None   # yellow->blue, "pca-sides" only


def _unit(x: float, y: float) -> Optional[Tuple[float, float]]:
    n = math.hypot(x, y)
    return None if n < 1e-9 else (x / n, y / n)


def compute_start_pose(cones: Sequence[Cone], backup_m: float = BACKUP_M) -> Optional[StartPose]:
    """The pose the sim would spawn the car at, or None where the sim fails.

    `cones` are (type, x, y) rows in CSV order: the order matters, because a
    multi-gate track's start gate is the one nearest the first blue/yellow row.
    """
    big = [(x, y) for t, x, y in cones if t == "big_orange"]
    track = [(x, y) for t, x, y in cones if t in ("blue", "yellow")]
    if len(big) < 4 or len(track) < 1:
        return None

    cx = sum(p[0] for p in big) / len(big)
    cy = sum(p[1] for p in big) / len(big)

    # Multi-gate (acceleration, skidpad, open tracks): cluster within 10 m.
    cluster = [-1] * len(big)
    centroids: List[Tuple[float, float]] = []
    for i in range(len(big)):
        if cluster[i] != -1:
            continue
        g = len(centroids)
        stack, members = [i], []
        cluster[i] = g
        while stack:
            c = stack.pop()
            members.append(c)
            for j in range(len(big)):
                if cluster[j] == -1 and (big[c][0] - big[j][0]) ** 2 + \
                        (big[c][1] - big[j][1]) ** 2 <= GATE_CLUSTER_M ** 2:
                    cluster[j] = g
                    stack.append(j)
        centroids.append((sum(big[m][0] for m in members) / len(members),
                          sum(big[m][1] for m in members) / len(members)))

    if len(centroids) >= 2:
        ts = track[0]
        start = min(range(len(centroids)),
                    key=lambda g: (centroids[g][0] - ts[0]) ** 2 + (centroids[g][1] - ts[1]) ** 2)
        others = [centroids[g] for g in range(len(centroids)) if g != start]
        ox = sum(p[0] for p in others) / len(others)
        oy = sum(p[1] for p in others) / len(others)
        f = _unit(ox - centroids[start][0], oy - centroids[start][1])
        if f is not None:
            a = centroids[start]
            return StartPose(anchor=a, forward=f,
                             spawn=(a[0] - f[0] * backup_m, a[1] - f[1] * backup_m),
                             method="multi-gate", gates=len(centroids))

    # One gate: PCA, smaller-variance axis = forward (the side test below can
    # swap it for the other axis).
    cxx = sum((x - cx) ** 2 for x, _ in big)
    cyy = sum((y - cy) ** 2 for _, y in big)
    cxy = sum((x - cx) * (y - cy) for x, y in big)
    half_trace = 0.5 * (cxx + cyy)
    det = cxx * cyy - cxy * cxy
    lam_small = half_trace - math.sqrt(max(0.0, half_trace * half_trace - det))
    # The C++ threshold is 1e-3 in cm^2; in m^2 that is 1e-7.
    if abs(cxy) > 1e-7:
        fx, fy = cxy, lam_small - cxx
    else:
        fx, fy = (1.0, 0.0) if cxx <= cyy else (0.0, 1.0)
    f = _unit(fx, fy)
    if f is None:
        return None

    # Side test: blue on the car's left. Of the two gate axes, forward is the
    # one more perpendicular to nearest-yellow -> nearest-blue.
    def mean_nearest(colour: str) -> Optional[Tuple[float, float]]:
        pts = sorted(((x - cx) ** 2 + (y - cy) ** 2, x, y) for t, x, y in cones
                     if t == colour and (x - cx) ** 2 + (y - cy) ** 2 <= SIDE_RADIUS_M ** 2)[:K_SIDE]
        if not pts:
            return None
        return (sum(p[1] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts))

    b, y_ = mean_nearest("blue"), mean_nearest("yellow")
    side = (b[0] - y_[0], b[1] - y_[1]) if b and y_ else None
    if side is not None and math.hypot(*side) >= MIN_SIDE_OFFSET_M:
        across = (-f[1], f[0])
        if abs(across[0] * side[0] + across[1] * side[1]) < abs(f[0] * side[0] + f[1] * side[1]):
            f = across
        # CSV frame is right-handed: the left of F is (-F.y, F.x).
        if side[0] * -f[1] + side[1] * f[0] < 0.0:
            f = (-f[0], -f[1])
        return StartPose(anchor=(cx, cy), forward=f,
                         spawn=(cx - f[0] * backup_m, cy - f[1] * backup_m),
                         method="pca-sides", gates=len(centroids), side_offset=side)

    near = sorted(track, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)[:K_DIRECTION]
    nx = sum(p[0] for p in near) / len(near) - cx
    ny = sum(p[1] for p in near) / len(near) - cy
    if nx * f[0] + ny * f[1] < 0.0:
        f = (-f[0], -f[1])
    return StartPose(anchor=(cx, cy), forward=f,
                     spawn=(cx - f[0] * backup_m, cy - f[1] * backup_m),
                     method="pca", gates=len(centroids))


def heading_error_deg(pose: StartPose, intended: Tuple[float, float]) -> float:
    """Angle between where the sim points the car and where the track goes."""
    u = _unit(*intended)
    if u is None:
        return 0.0
    dot = max(-1.0, min(1.0, pose.forward[0] * u[0] + pose.forward[1] * u[1]))
    return math.degrees(math.acos(dot))
