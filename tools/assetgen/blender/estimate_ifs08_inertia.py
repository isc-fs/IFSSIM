"""Estimate the IFS-08's body inertia from its CAD, for settings.json VehiclePhysics.Inertia.

    blender -b <IFS08_KIN>\\cad\\blender\\IFS08_livery.blend --python tools\\assetgen\\blender\\estimate_ifs08_inertia.py -- [--cells-kg 38]

An ESTIMATE, not a measurement. Each CAD part gives its own volume (closed
meshes) or surface (open meshes, thin panels) moments; its mass comes from a
density picked by its material name. The CAD is missing mass (the accumulator
is an empty container, and there is no wiring, DV kit or fluids), so the
total is brought to settings.json Mass: --cells-kg as a uniform block filling
the accumulator container, the rest spread like the CAD parts. The cells'
mass drives the result most (40 / 50 / 60 kg -> yaw 154 / 148 / 142 kg*m^2).

Check: the estimate's own centre of mass is not fitted to WeightDistFront or
CoGHeight, so how close it lands to them says something about the densities.

Prints roll / pitch / yaw about the centre of mass in kg*m^2. Reads the .blend
only; never saves it.
"""
import argparse
import json
import os
import re
import runpy
import sys
import tempfile

import bmesh
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--cells-kg", type=float, default=38.0,
                help="accumulator contents: cells + modules, the CAD has only the empty case (12.5 kg). AS_BT_1000_v2 rebuilt is 50.6 kg")
ap.add_argument("--total-kg", type=float, default=None, help="default: settings.json Mass")
args = ap.parse_args(argv)
with open(os.path.join(HERE, "..", "..", "..", "settings.json"), encoding="utf-8") as f:
    VP = next(iter(json.load(f)["Vehicles"].values()))["VehiclePhysics"]
TOTAL = args.total_kg or float(VP["Mass"])

# prepare() / find_wheels() from the build script, without running its export.
sys.argv = [sys.argv[0], "--", "--out", os.path.join(tempfile.gettempdir(), "ifs08_inertia")]
B = runpy.run_path(os.path.join(HERE, "build_ifs08_vehicle.py"), run_name="ifs08_build")

STEEL, ALU, CU, PLAST, FOAM = 7850, 2700, 8900, 1200, 100
CFRP_AREAL = 2.2      # kg per m^2 of panel (sandwich); panel area = surface area / 2
OPEN_T = 0.0015       # wall assumed for open meshes that are not CFRP, m
RULES = [  # regex on "<first material> | <object name>", how, value
    (r"Rear alternativo-1\.027$", "fixed", 1.0),   # impact attenuator: a solid block in CAD, honeycomb on the car
    (r"Tyre rubber", "fixed", 4.3),                # Hoosier 16x7.5-10, each
    (r"EMRAX", "share", 12.3),                     # EMRAX 228 datasheet, over its parts by volume
    (r"Livery|Carbon fibre|SP panel carbon", "areal", CFRP_AREAL),
    (r"Headrest cushion|foam|Roll bar pad|Harness pad|Seat edge", "vol", FOAM),
    (r"Harness webbing|Harness patch|Harness cover|Harness pull", "vol", 600),
    (r"HV lug|TSAL copper|copper", "vol", CU),
    (r"Radox", "vol", 3500),
    (r"FR4|board edge|Brake light", "vol", 1850),
    (r"acrylic|DISPLAY_GLASS", "vol", 1190),
    (r"Plastic|Rubber|rubber|LED|Cable tie|nylon|CV boot|Hose|black plastic|Fan blades|Fan frame|Fan hub|Grip tape|tape|"
     r"sticker|Label|label|Wheel button|Wheel display|Wheel black plastic|Indicator|XB4|Bleed cap|SP terminal|SP measuring",
     "vol", PLAST),
    (r"Painted #00402A|Steel|steel|Black oxide|Bolt|Fastener|Screw|Zinc|Chrome|chrome|Cardan|Drive shaft|Chain|Safety clip|"
     r"Polished|Damper spring|Brake disc|Hub nut|Mono nut|Sprocket steel|stainless|nickel|Powder coat|Brake pad|Bleed nipple|"
     r"Harness buckle|Harness tongue|Harness adjuster|Harness hardware|Wheel black-oxide|SP button|SP red key|"
     r"Painted #0F0F0F|Painted #E86A10|HVD screw", "vol", STEEL),
    (r"Anodised|anodised|Machined|aluminium|Hub worn|Rim black|Sprocket alu|Damper|Radiator|Cooling|sprayed|Caliper logo|"
     r"Wheel badge|Rim lettering|SP aluminium", "vol", ALU),
]


def rule(o):
    mats = [s.material.name for s in o.material_slots if s.material]
    t = (mats[0] if mats else "") + " | " + o.name
    for rx, how, val in RULES:
        if re.search(rx, t):
            return how, val
    return "vol", ALU


def moments(o):
    """World-space integrals: V, int x dV, int x x^T dV, A, int x dA, int x x^T dA, and whether the mesh is closed."""
    me = o.data
    me.calc_loop_triangles()
    n = len(me.loop_triangles)
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    mw = np.array(o.matrix_world)
    co = co.reshape(-1, 3) @ mw[:3, :3].T + mw[:3, 3]
    tri = np.empty(n * 3, np.int64)
    me.loop_triangles.foreach_get("vertices", tri)
    tri = tri.reshape(-1, 3)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    s = a + b + c
    d = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0           # signed tetra volumes from the origin
    M2 = sum(np.einsum("t,ti,tj->ij", d, x, x) for x in (a, b, c, s)) / 20.0
    w = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    bm = bmesh.new()
    bm.from_mesh(me)
    closed = all(e.is_manifold for e in bm.edges)
    bm.free()
    return (d.sum(), (d[:, None] * s).sum(0) / 4.0, M2,
            w.sum(), (w[:, None] * s / 3).sum(0), np.einsum("t,ti,tj->ij", w, s / 3, s / 3), closed)


objs = B["prepare"]()
wheels, ground_z = B["find_wheels"](objs)
rows = []        # [mass, centroid, second moment per unit mass]
share = []
for o in objs:
    if not o.data.polygons:
        continue
    V, C1, M2, A, S1, S2, closed = moments(o)
    how, val = rule(o)
    by_volume = closed and abs(V) > 0 and how in ("vol", "share", "fixed")
    if how == "fixed":
        m = val
    elif how == "areal":
        m = A / 2 * val
    elif how == "share":
        m = abs(V) if closed else 0.0           # scaled to the datasheet mass below
    else:
        m = abs(V) * val if by_volume else A / 2 * OPEN_T * val
    if m <= 0 or A <= 0:
        continue
    if how == "share":
        share.append(len(rows))
    if by_volume:
        sg = 1.0 if V > 0 else -1.0
        rows.append([m, sg * C1 / abs(V), sg * M2 / abs(V)])
    else:
        rows.append([m, S1 / A, S2 / A])
emrax = next(val for _, how, val in RULES if how == "share")
vol = sum(rows[i][0] for i in share)
for i in share:
    rows[i][0] *= emrax / vol
cad = sum(r[0] for r in rows)

# The accumulator container's extent, from its base, walls and lids.
pts = []
for o in objs:
    if re.match(r"PA_BT_100(03|1|4|5|6|12)", o.name):
        _, _, _, A, S1, S2, _ = moments(o)
        c = S1 / A
        half = np.sqrt(np.maximum(np.diag(S2 / A - np.outer(c, c)), 0) * 3)
        pts += [c - half, c + half]
lo, hi = np.min(pts, 0), np.max(pts, 0)
box_c, box_e = (lo + hi) / 2, hi - lo
cells = min(args.cells_kg, TOTAL - cad)
spread = TOTAL - cad - cells
rows = [[m * (1 + spread / cad), c, s] for m, c, s in rows]
rows.append([cells, box_c, np.outer(box_c, box_c) + np.diag(box_e ** 2 / 12)])

M = sum(r[0] for r in rows)
cg = sum(r[0] * r[1] for r in rows) / M
S = sum(r[0] * r[2] for r in rows) - M * np.outer(cg, cg)
I = np.trace(S) * np.eye(3) - S          # scene frame: x left, y rear, z up
fy = (wheels["FL"]["centre"][1] + wheels["FR"]["centre"][1]) / 2
ry = (wheels["RL"]["centre"][1] + wheels["RR"]["centre"][1]) / 2
print(f"[inertia] CAD parts {cad:.1f} kg; cells {cells:.1f} kg in the accumulator box "
      f"{np.round(box_e, 2)} m; {spread:.1f} kg more spread like the CAD; total {M:.1f} kg")
print(f"[inertia] centre of mass: {(ry - cg[1]) / (ry - fy) * 100:.1f} % front "
      f"(settings {VP['WeightDistFront'] * 100:.1f} %), {cg[2] - ground_z:.3f} m up (settings {VP['CoGHeight']:.3f})")
print(f"[inertia] about it, kg*m^2: roll {I[1, 1]:.1f}, pitch {I[0, 0]:.1f}, yaw {I[2, 2]:.1f}"
      f"  ->  \"Inertia\": [{I[1, 1]:.1f}, {I[0, 0]:.1f}, {I[2, 2]:.1f}]")
