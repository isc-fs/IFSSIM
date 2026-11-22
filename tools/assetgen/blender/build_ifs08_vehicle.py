"""Game-ready IFS-08 for IFSSIM, from the team's Blender livery scene.

    blender -b <IFS08_livery.blend> --python tools/assetgen/blender/build_ifs08_vehicle.py -- \
        --out <dir> [--weight-front 0.481] [--budget 450000] [--tex 4096] [--no-bake] [--preview]

Reads the livery scene (SolidWorks export + procedural Cycles materials, ~10 M
triangles) and writes, to --out:

    SK_IFS08.fbx              body: skeletal mesh, bones root + WheelFL/FR/RL/RR at the hub centres
    SM_IFS08_Wheel_<c>.fbx    the parts that spin (tyre, rim, disc, hub), pivot at the hub centre
    SM_IFS08_Corner_<c>.fbx   the parts that steer and travel but do not spin (upright, caliper)
    T_IFS08_<atlas>_{BC,N,ORM}.png   baked textures; ORM = AO, roughness, metallic
    manifest.json             geometry (UE cm), material slots, triangle counts

The .blend is never saved: everything happens on the loaded copy.

Frames. The scene is x = car left, y = rearward, z = up, metres, ground at the
lowest tyre point. The export frame is x forward, y left, z up, metres, origin
on the ground under the centre of mass (Wheelbase x (1 - weight-front) behind the
front axle, on the centreline): the sim puts the centre of mass at the body
origin and places the wheels around it (FSDSWheeledVehicleMovementComponent::
PlaceWheelsFromSettings), so the art has to agree. The FBX exporter's axis
conversion then gives UE's x forward, y right.

Steps:
 1. Drop the scene (ground, Jarama, cameras, lights), apply modifiers,
    convert curves and text to meshes.
 2. Split each object between body, a wheel (spins) and a corner (steers).
 3. Cull what no camera can see: rays from ~150 viewpoints around the car and
    a panorama from the cockpit. A part no ray reaches is dropped.
 4. Decimate to --budget triangles, shared by how much of each part is seen.
    Tyre and rim lettering only feed the bake (it is a few mm proud of the tyre).
 5. Atlas UVs and bake from the full-resolution parts (Cycles): base colour,
    roughness, tangent normal, metallic and AO.
    LED dies, the TSAL and the dash screen keep their own material slots so the
    sim can light them; glass is dropped.
 6. Export.
"""

import argparse
import json
import math
import os
import re
import sys
import time

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

# --- arguments ------------------------------------------------------------------

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--weight-front", type=float, default=0.481,
                help="front axle share of the weight, places the centre of mass (KIN CAD: 0.481)")
ap.add_argument("--budget", type=int, default=450_000, help="LOD0 triangles, all meshes")
ap.add_argument("--tex", type=int, default=4096, help="atlas size, pixels")
ap.add_argument("--samples", type=int, default=8, help="bake samples (AO takes 4x)")
ap.add_argument("--rays", type=int, default=256, help="visibility render size / 4, pixels")
ap.add_argument("--no-bake", action="store_true", help="geometry only: flat grey materials")
ap.add_argument("--preview", action="store_true", help="render preview PNGs of the result")
ap.add_argument("--cpu", action="store_true", help="bake on the CPU")
args = ap.parse_args(argv)
OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)

T0 = time.time()


def log(msg):
    print(f"[ifs08 {time.time() - T0:7.1f}s] {msg}", flush=True)


CORNERS = ("FL", "FR", "RL", "RR")

# What spins with the wheel, and what steers with it without spinning. Matched
# against the object name, its collections and its SolidWorks component path.
ROT_RE = re.compile(r"Tyre|_RIM|PA_SP_3035|lettering|Brake Rotor|Wheel Hub|Hub Nut|Hub Locking|"
                    r"03033|3033_Mono|Safety clip|Wear frame", re.I)
CORNER_RE = re.compile(r"Upright|Brake Caliper", re.I)
# Lettering sits a few mm proud of the tyre: it goes into the tyre's texture,
# not into the mesh.
BAKE_ONLY_RE = re.compile(r"lettering", re.I)
# Kept as their own slots, not baked: the sim lights them.
FX_MAT_RE = re.compile(r"LED die|LED silicone|DISPLAY_SCREEN", re.I)
GLASS_MAT_RE = re.compile(r"glass|lens", re.I)
DROP_COLL = {"Environment", "Env Jarama"}
ALWAYS_KEEP_RE = re.compile(r"DISPLAY_SCREEN|TSAL|Shift LED|Brake light|Indicator", re.I)

scene = bpy.context.scene
vl = bpy.context.view_layer


def tag(o):
    p = o.get("sw_path", "") or ""
    c = o.get("sw_component", "") or ""
    return " ".join([o.name, c, p] + [k.name for k in o.users_collection])


def world_bbox(o):
    pts = [o.matrix_world @ Vector(c) for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def renderable(o):
    if o.hide_render:
        return False
    for c in o.users_collection:
        if c.hide_render:
            return False
    return True


# --- 1. scene to plain meshes -----------------------------------------------------

def prepare():
    drop = []
    for o in list(bpy.data.objects):
        colls = {c.name for c in o.users_collection}
        if colls & DROP_COLL or o.type in {"CAMERA", "LIGHT", "LIGHT_PROBE", "SPEAKER"}:
            drop.append(o)
    for o in drop:
        bpy.data.objects.remove(o, do_unlink=True)

    dg = bpy.context.evaluated_depsgraph_get()
    meshes = []
    for o in list(bpy.data.objects):
        if o.type not in {"MESH", "CURVE", "FONT", "SURFACE", "META"}:
            continue
        if not renderable(o):
            bpy.data.objects.remove(o, do_unlink=True)
            continue
        ev = o.evaluated_get(dg)
        me = bpy.data.meshes.new_from_object(ev, preserve_all_data_layers=True, depsgraph=dg)
        if len(me.polygons) == 0:
            bpy.data.objects.remove(o, do_unlink=True)
            continue
        mw = o.matrix_world.copy()
        props = {k: o[k] for k in ("sw_component", "sw_path") if k in o}
        name = o.name
        colls = list(o.users_collection)
        if o.type == "MESH":
            o.modifiers.clear()
            o.data = me
            o.parent = None
            o.matrix_world = mw
            meshes.append(o)
        else:
            n = bpy.data.objects.new(name + " mesh", me)
            for c in colls:
                c.objects.link(n)
            n.matrix_world = mw
            for k, v in props.items():
                n[k] = v
            bpy.data.objects.remove(o, do_unlink=True)
            meshes.append(n)
    # Empties (parents, wear frames) are no longer needed: transforms are baked
    # into matrix_world.
    for o in list(bpy.data.objects):
        if o.type == "EMPTY":
            bpy.data.objects.remove(o, do_unlink=True)
    log(f"prepared {len(meshes)} meshes, "
        f"{sum(len(o.data.polygons) for o in meshes) / 1e6:.2f} M faces")
    return meshes


# --- 2. classify ---------------------------------------------------------------

def find_wheels(objs):
    """Hub centre, radius and ground from the four tyres."""
    tyres = [o for o in objs if re.search(r"Tyre", tag(o)) and not re.search(r"lettering", o.name, re.I)
             and len(o.data.polygons) > 10000]
    if len(tyres) != 4:
        raise RuntimeError(f"expected 4 tyres, found {[o.name for o in tyres]}")
    wheels = {}
    zmin = 1e9
    for t in tyres:
        co = np.empty(len(t.data.vertices) * 3)
        t.data.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3)
        mw = np.array(t.matrix_world)
        w = co @ mw[:3, :3].T + mw[:3, 3]
        lo, hi = w.min(0), w.max(0)
        c = (lo + hi) / 2
        r = float(np.sqrt(((w[:, 1] - c[1]) ** 2 + (w[:, 2] - c[2]) ** 2)).max())
        zmin = min(zmin, float(lo[2]))
        key = ("F" if c[1] < 0 else "R") + ("L" if c[0] > 0 else "R")   # -y forward, +x left
        wheels[key] = {"centre": [float(v) for v in c], "radius": r, "width": float(hi[0] - lo[0])}
    if sorted(wheels) != sorted(CORNERS):
        raise RuntimeError(f"tyres do not make four corners: {sorted(wheels)}")
    return wheels, zmin


def classify(objs, wheels):
    cls = {}
    for o in objs:
        lo, hi = world_bbox(o)
        c = (lo + hi) / 2
        t = tag(o)
        best, bd = None, 1e9
        for k, w in wheels.items():
            h = Vector(w["centre"])
            if abs(c.x - h.x) < 0.25:
                d = math.hypot(c.y - h.y, c.z - h.z)
                if d < bd:
                    best, bd = k, d
        kind = "body"
        if best and bd < 0.30:
            if ROT_RE.search(t):
                kind = "wheel"
            elif CORNER_RE.search(t):
                kind = "corner"
        cls[o.name] = (kind, best if kind != "body" else None)
    counts = {}
    for kind, k in cls.values():
        counts[(kind, k)] = counts.get((kind, k), 0) + 1
    log("classified: " + ", ".join(f"{a}{'/' + b if b else ''}={n}" for (a, b), n in sorted(
        counts.items(), key=lambda x: (x[0][0], x[0][1] or ""))))
    return cls


# --- 3. visibility ---------------------------------------------------------------

def fibonacci_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


def srgb_to_linear(v):
    return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4


def visibility(objs, ground_z):
    """Pixels each part covers, summed over the viewpoints: an object-ID render.

    Workbench, flat object colours, no anti-aliasing and no dither, so every
    pixel is exactly one part's colour: 16 levels per channel encode the part
    (4095 ids, 0 = background). scene.ray_cast walks every object's BVH per ray
    (~1 ms with 2,500 parts); this is one GPU render per view."""
    ids = {}
    for i, o in enumerate(objs):
        code = i + 1
        if code > 4095:
            raise RuntimeError("more than 4095 parts: widen the id encoding")
        lv = ((code >> 8) & 15, (code >> 4) & 15, code & 15)
        o.color = (*[srgb_to_linear(l / 15) for l in lv], 1.0)
        ids[code] = o.name
    r = scene.render
    saved = (r.engine, r.resolution_x, r.resolution_y, r.resolution_percentage, r.dither_intensity,
             scene.view_settings.view_transform, scene.view_settings.look, scene.camera, r.filepath)
    r.engine = "BLENDER_WORKBENCH"
    r.resolution_x = r.resolution_y = args.rays * 4
    r.resolution_percentage = 100
    r.dither_intensity = 0.0
    r.film_transparent = False
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGB"
    r.image_settings.color_depth = "8"
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = 0
    scene.view_settings.gamma = 1
    scene.display.render_aa = "OFF"
    sh = scene.display.shading
    sh.light = "FLAT"
    sh.color_type = "OBJECT"
    sh.background_type = "WORLD"
    for flag in ("show_object_outline", "show_cavity", "show_shadows", "show_specular_highlight", "show_xray",
                 "use_dof", "show_backface_culling"):
        if hasattr(sh, flag):
            setattr(sh, flag, False)
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.color = (0, 0, 0)

    lo = Vector((min(world_bbox(o)[0].x for o in objs), min(world_bbox(o)[0].y for o in objs), ground_z))
    hi = Vector((max(world_bbox(o)[1].x for o in objs), max(world_bbox(o)[1].y for o in objs),
                 max(world_bbox(o)[1].z for o in objs)))
    centre = (lo + hi) / 2
    radius = (hi - lo).length / 2
    cam = bpy.data.objects.new("VIS_CAM", bpy.data.cameras.new("VIS_CAM"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.data.clip_start = 0.01
    cam.data.clip_end = 50
    counts = np.zeros(4096, np.int64)
    path = os.path.join(OUT, "_vis.png")
    table = np.arange(16) / 15.0   # replaced by the calibration below

    def render_px():
        r.filepath = path
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(path, check_existing=False)
        px = np.empty(img.size[0] * img.size[1] * 4, np.float32)
        img.pixels.foreach_get(px)
        w, h = img.size
        bpy.data.images.remove(img)
        return px.reshape(h, w, 4)[..., :3]

    # Calibrate: what each of the 16 levels comes out as, after whatever colour
    # management Workbench applies, from 16 grey quads far above the car.
    quads = []
    for lv in range(16):
        me = bpy.data.meshes.new(f"VIS_Q{lv}")
        x0 = lv * 1.0
        me.from_pydata([(x0, 0, 0), (x0 + 1, 0, 0), (x0 + 1, 1, 0), (x0, 1, 0)], [], [(0, 1, 2, 3)])
        q = bpy.data.objects.new(f"VIS_Q{lv}", me)
        q.location = (0, 0, 500)
        c = srgb_to_linear(lv / 15)
        q.color = (c, c, c, 1)
        scene.collection.objects.link(q)
        quads.append(q)
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = 16
    cam.location = (8, 0.5, 520)
    cam.rotation_euler = (0, 0, 0)
    rx, ry = r.resolution_x, r.resolution_y
    r.resolution_x, r.resolution_y = 320, 20
    px = render_px()
    table = np.array([px[10, int((lv + 0.5) * 20), 0] for lv in range(16)])
    r.resolution_x, r.resolution_y = rx, ry
    cam.data.type = "PERSP"
    for q in quads:
        me = q.data
        bpy.data.objects.remove(q, do_unlink=True)
        bpy.data.meshes.remove(me)
    gaps = np.diff(table)
    log(f"visibility: level calibration {np.round(table * 255).astype(int).tolist()}")
    if gaps.min() < 1.5 / 255:
        raise RuntimeError("visibility: object-colour levels are not distinguishable after colour management")
    mids = (table[1:] + table[:-1]) / 2

    def shoot(pos, look_at=None, quat=None, fov=None):
        cam.location = pos
        if quat is None:
            quat = (look_at - pos).to_track_quat("-Z", "Y")
        cam.rotation_euler = quat.to_euler()
        cam.data.angle = fov
        lv = np.searchsorted(mids, render_px().reshape(-1, 3)).astype(np.int64)
        code = (lv[:, 0] << 8) | (lv[:, 1] << 4) | lv[:, 2]
        counts[:] += np.bincount(code, minlength=4096)[:4096]

    # Exterior: viewpoints on a sphere, none lower than 4 cm (a cone's eye on
    # the ground is the lowest anyone looks from), framing the bounding sphere.
    t = time.time()
    dirs = fibonacci_sphere(220)
    for v, d in enumerate(dirs):
        p = centre + Vector(d) * radius * 2.2
        if p.z < ground_z + 0.04:
            p.z = ground_z + 0.04
        dist = (centre - p).length
        shoot(p, look_at=centre, fov=2 * math.asin(min(radius / dist, 0.99)))
        if v % 55 == 0:
            log(f"visibility: view {v}/{len(dirs)} ({time.time() - t:.0f}s)")
    # Cockpit: a cube of 90 degree views from a head above the seat.
    seat = [o for o in objs if re.search(r"Seat", tag(o))]
    if seat:
        slo, shi = world_bbox(seat[0])
        head = Vector(((slo.x + shi.x) / 2, (slo.y + shi.y) / 2 - 0.1, shi.z + 0.25))
        for axis in (Vector((1, 0, 0)), Vector((-1, 0, 0)), Vector((0, 1, 0)), Vector((0, -1, 0)),
                     Vector((0, 0, -1))):
            shoot(head, look_at=head + axis, fov=math.radians(92))
    (r.engine, r.resolution_x, r.resolution_y, r.resolution_percentage, r.dither_intensity,
     scene.view_settings.view_transform, scene.view_settings.look, scene.camera, r.filepath) = saved
    bpy.data.objects.remove(cam, do_unlink=True)
    if os.path.exists(path):
        os.remove(path)
    hits = {name: int(counts[code]) for code, name in ids.items()}
    unknown = int(counts[len(objs) + 1:].sum())
    seen = sum(1 for v in hits.values() if v)
    log(f"visibility: {seen}/{len(hits)} parts seen in {time.time() - t:.0f}s"
        + (f" ({unknown} pixels decoded to no part)" if unknown else ""))
    return hits


# --- 4. decimate -----------------------------------------------------------------

def allocate(tris, hits, budget, alpha=0.6, floor=24):
    w = {k: hits[k] ** alpha for k in tris if hits[k] > 0}
    target = {}
    pool = budget
    free = dict(w)
    for _ in range(6):
        tot = sum(free.values())
        if tot <= 0:
            break
        nxt = {}
        spent = 0
        for k, wk in free.items():
            t = max(floor, pool * wk / tot)
            if t >= tris[k]:
                target[k] = tris[k]
                spent += tris[k]
            else:
                nxt[k] = wk
        pool -= spent
        if len(nxt) == len(free):
            for k, wk in free.items():
                target[k] = int(max(floor, pool * wk / tot))
            break
        free = nxt
    return target


def tri_count(me):
    return sum(len(p.vertices) - 2 for p in me.polygons)


def decimate(lows, target):
    for o in lows:
        t = target[o["src"]]
        n = tri_count(o.data)
        o.modifiers.clear()
        tri = o.modifiers.new("tri", "TRIANGULATE")
        if t < n * 0.98:
            m = o.modifiers.new("dec", "DECIMATE")
            m.decimate_type = "COLLAPSE"
            m.ratio = max(t / n, 0.002)
            m.use_collapse_triangulate = True
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    for o in lows:
        me = bpy.data.meshes.new_from_object(o.evaluated_get(dg), depsgraph=dg)
        old = o.data
        o.modifiers.clear()
        o.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
        # CAD custom normals do not survive decimation: shade by angle instead.
        try:
            me.set_sharp_from_angle(angle=math.radians(35))
        except Exception:
            pass
        for p in me.polygons:
            p.use_smooth = True


# --- 5. atlas and bake -------------------------------------------------------------

def split_fx(o):
    """Move faces with FX materials (LEDs, screen) and glass out of o. Returns the FX object or None."""
    mats = [s.material for s in o.material_slots]
    fx_idx = {i for i, m in enumerate(mats) if m and FX_MAT_RE.search(m.name)}
    glass_idx = {i for i, m in enumerate(mats) if m and GLASS_MAT_RE.search(m.name)}
    if not fx_idx and not glass_idx:
        return None
    bm = bmesh.new()
    bm.from_mesh(o.data)
    glass = [f for f in bm.faces if f.material_index in glass_idx]
    bmesh.ops.delete(bm, geom=glass, context="FACES")
    fx_faces = [f for f in bm.faces if f.material_index in fx_idx]
    fx = None
    if fx_faces:
        bm_fx = bm.copy()
        bmesh.ops.delete(bm_fx, geom=[f for f in bm_fx.faces if f.material_index not in fx_idx], context="FACES")
        me_fx = bpy.data.meshes.new(o.name + " fx")
        bm_fx.to_mesh(me_fx)
        bm_fx.free()
        fx = bpy.data.objects.new(o.name + " fx", me_fx)
        scene.collection.objects.link(fx)
        fx.matrix_world = o.matrix_world
        for m in mats:
            fx.data.materials.append(m)
        fx["src"] = o["src"]
        fx["part"] = o["part"]
        bmesh.ops.delete(bm, geom=fx_faces, context="FACES")
    bm.to_mesh(o.data)
    bm.free()
    return fx


def select_only(objs, active=None):
    for o in vl.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    vl.objects.active = active or (objs[0] if objs else None)


def atlas_uv(objs):
    for o in objs:
        me = o.data
        while me.uv_layers:
            me.uv_layers.remove(me.uv_layers[0])
        me.uv_layers.new(name="UVMap")
    select_only(objs)
    with bpy.context.temp_override(active_object=objs[0], selected_objects=objs,
                                   selected_editable_objects=objs, object=objs[0]):
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.uv.smart_project(angle_limit=math.radians(60), island_margin=0.0015,
                                 area_weight=0.0, correct_aspect=True, scale_to_bounds=False)
        bpy.ops.object.mode_set(mode="OBJECT")
    area = 0.0
    for o in objs:
        uv = o.data.uv_layers.active.data
        for p in o.data.polygons:
            a = [uv[i].uv for i in p.loop_indices]
            area += abs(sum(a[k].x * a[k - 1].y - a[k - 1].x * a[k].y for k in range(len(a)))) / 2
    log(f"  UV: {len(objs)} meshes, {area:.2f} of the atlas covered")
    if area < 0.2:
        raise RuntimeError("smart UV project did not unwrap the atlas")


def prune_slots(o):
    """Drop material slots no face uses (an FX split keeps its source's slots)."""
    me = o.data
    used = sorted({p.material_index for p in me.polygons})
    if len(used) == len(me.materials):
        return
    remap = {old: new for new, old in enumerate(used)}
    mats = [me.materials[i] for i in used]
    idx = [remap[p.material_index] for p in me.polygons]
    me.materials.clear()
    for m in mats:
        me.materials.append(m)
    me.polygons.foreach_set("material_index", idx)


def joined_copy(objs, name):
    copies = []
    for o in objs:
        c = o.copy()
        c.data = o.data.copy()
        scene.collection.objects.link(c)
        copies.append(c)
    with bpy.context.temp_override(active_object=copies[0], selected_objects=copies,
                                   selected_editable_objects=copies, object=copies[0]):
        bpy.ops.object.join()
    j = copies[0]
    j.name = name
    return j


def new_image(name, size, color, non_color):
    img = bpy.data.images.new(name, size, size, alpha=False, float_buffer=False)
    img.generated_color = color
    if non_color:
        img.colorspace_settings.name = "Non-Color"
    return img


def bake_target_material(img):
    m = bpy.data.materials.new("BAKE_TARGET")
    m.use_nodes = True
    nt = m.node_tree
    n = nt.nodes.new("ShaderNodeTexImage")
    n.image = img
    nt.nodes.active = n
    return m


def set_bake_target(obj, img):
    obj.data.materials.clear()
    obj.data.materials.append(bake_target_material(img))


def metallic_of(mat):
    """Metallic from the material's Principled nodes, or its name."""
    best = None
    if mat and mat.node_tree:
        stack = [mat.node_tree]
        seen = set()
        while stack:
            nt = stack.pop()
            if nt.name in seen:
                continue
            seen.add(nt.name)
            for n in nt.nodes:
                if n.bl_idname == "ShaderNodeBsdfPrincipled":
                    s = n.inputs.get("Metallic")
                    if s is not None and not s.is_linked:
                        best = max(best or 0.0, float(s.default_value))
                elif n.bl_idname == "ShaderNodeGroup" and n.node_tree:
                    stack.append(n.node_tree)
    if best is not None:
        return best
    name = mat.name if mat else ""
    if re.search(r"steel|zinc|machined|anodis|alumin|chrome|brushed|alloy|hub|disc|cardan|emrax|copper|"
                 r"tinned|lead frame|spring", name, re.I):
        return 1.0
    return 0.0


def emit_material(v, cache={}):
    key = round(v, 2)
    if key not in cache:
        m = bpy.data.materials.new(f"EMIT_{key}")
        m.use_nodes = True
        nt = m.node_tree
        for n in list(nt.nodes):
            nt.nodes.remove(n)
        e = nt.nodes.new("ShaderNodeEmission")
        e.inputs["Color"].default_value = (key, key, key, 1)
        e.inputs["Strength"].default_value = 1.0
        out = nt.nodes.new("ShaderNodeOutputMaterial")
        nt.links.new(e.outputs[0], out.inputs[0])
        cache[key] = m
    return cache[key]


def pixels(img):
    a = np.empty(img.size[0] * img.size[1] * 4, np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(img.size[1], img.size[0], 4)


def save_png(arr, path, non_color):
    h, w = arr.shape[:2]
    img = bpy.data.images.new(os.path.basename(path), w, h, alpha=False, float_buffer=False)
    if non_color:
        img.colorspace_settings.name = "Non-Color"
    img.pixels.foreach_set(arr.astype(np.float32).ravel())
    img.filepath_raw = path
    img.file_format = "PNG"
    img.save()
    bpy.data.images.remove(img)


def setup_cycles():
    scene.render.engine = "CYCLES"
    if not args.cpu:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for dev_type in ("OPTIX", "CUDA", "HIP", "ONEAPI"):
            try:
                prefs.compute_device_type = dev_type
                prefs.get_devices()
                gpus = [d for d in prefs.devices if d.type == dev_type]
                if gpus:
                    for d in prefs.devices:
                        d.use = d.type == dev_type
                    scene.cycles.device = "GPU"
                    log(f"bake on {dev_type}: {[d.name for d in gpus]}")
                    break
            except TypeError:
                continue
    scene.cycles.samples = args.samples
    scene.cycles.use_denoising = False
    b = scene.render.bake
    b.margin = 8
    b.margin_type = "EXTEND"
    b.target = "IMAGE_TEXTURES"
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.light_settings.distance = 0.12   # AO reach, m


def principled_nodes(mat):
    out, stack, seen = [], [mat.node_tree] if mat and mat.node_tree else [], set()
    while stack:
        nt = stack.pop()
        if nt.name in seen:
            continue
        seen.add(nt.name)
        for n in nt.nodes:
            if n.bl_idname == "ShaderNodeBsdfPrincipled":
                out.append((nt, n))
            elif n.bl_idname == "ShaderNodeGroup" and n.node_tree:
                stack.append(n.node_tree)
    return out


def flat_diffuse(color, cache={}):
    key = tuple(round(c, 3) for c in color[:3])
    if key not in cache:
        m = bpy.data.materials.new("FLAT_" + "_".join(map(str, key)))
        m.use_nodes = True
        nt = m.node_tree
        nt.nodes.clear()
        d = nt.nodes.new("ShaderNodeBsdfDiffuse")
        d.inputs["Color"].default_value = (*key, 1)
        o = nt.nodes.new("ShaderNodeOutputMaterial")
        nt.links.new(d.outputs[0], o.inputs[0])
        cache[key] = m
    return cache[key]


def hex_colour(name):
    m = re.search(r"#([0-9A-Fa-f]{6})", name or "")
    if not m:
        return None
    srgb = [int(m.group(1)[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb]


class AlbedoPass:
    """Make Cycles' diffuse-colour pass return the base colour of metals too.

    The pass leaves out the metallic lobe, so a steel part bakes black. For the
    bake, Principled Metallic is 0 (links into it cut), and a material with no
    Principled node at all (a bare glossy chrome) is swapped for a flat diffuse
    of its colour. Everything is put back afterwards: materials are shared
    between atlases."""

    def __init__(self, highs):
        self.highs = highs

    def __enter__(self):
        self.links, self.values, self.slots = [], [], []
        done = set()
        for h in self.highs:
            for i, s in enumerate(h.material_slots):
                m = s.material
                if not m:
                    continue
                pn = principled_nodes(m)
                if not pn:
                    col = None
                    if m.node_tree:
                        for n in m.node_tree.nodes:
                            for key in ("Color", "Base Color"):
                                inp = n.inputs.get(key) if n.type.startswith("BSDF") else None
                                if inp is not None and not inp.is_linked:
                                    col = list(inp.default_value)[:3]
                    col = col or hex_colour(m.name) or [0.6, 0.6, 0.6]
                    self.slots.append((h, i, m))
                    s.material = flat_diffuse(col)
                    continue
                if m.name in done:
                    continue
                done.add(m.name)
                for nt, n in pn:
                    sock = n.inputs.get("Metallic")
                    if sock is None:
                        continue
                    for l in list(sock.links):
                        self.links.append((nt, l.from_socket, sock))
                        nt.links.remove(l)
                    self.values.append((sock, sock.default_value))
                    sock.default_value = 0.0
        return self

    def __exit__(self, *exc):
        for nt, a, b in self.links:
            nt.links.new(a, b)
        for sock, v in self.values:
            sock.default_value = v
        for h, i, m in self.slots:
            h.material_slots[i].material = m


def bake_atlas(atlas, lows, highs, all_lows, size):
    """Bake one atlas: lows share its UV space, highs are the full-resolution sources."""
    log(f"atlas {atlas}: {len(lows)} meshes, {sum(tri_count(o.data) for o in lows) / 1e3:.0f}k tris, "
        f"{len(highs)} sources")
    j = joined_copy(lows, f"BAKE_{atlas}")
    for o in all_lows:
        o.hide_render = True
    j.hide_render = False
    for h in highs:
        h.hide_render = False
    b = scene.render.bake
    res = {}

    def run(kind, img, s2a=True, **kw):
        set_bake_target(j, img)
        select_only(highs + [j] if s2a else [j], active=j)
        b.use_selected_to_active = s2a
        b.cage_extrusion = 0.004
        b.max_ray_distance = 0.012
        t = time.time()
        with bpy.context.temp_override(active_object=j, object=j,
                                       selected_objects=highs + [j] if s2a else [j]):
            bpy.ops.object.bake(type=kind, use_selected_to_active=s2a, **kw)
        log(f"  {atlas} {kind} {time.time() - t:.0f}s")
        return pixels(img)

    scene.cycles.samples = args.samples
    with AlbedoPass(highs):
        res["bc"] = run("DIFFUSE", new_image(f"{atlas}_bc", size, (0.5, 0.5, 0.5, 1), False),
                        pass_filter={"COLOR"})
    res["rough"] = run("ROUGHNESS", new_image(f"{atlas}_r", size, (0.6, 0.6, 0.6, 1), True))
    res["n"] = run("NORMAL", new_image(f"{atlas}_n", size, (0.5, 0.5, 1, 1), True), normal_space="TANGENT")
    # Metallic: swap every source material for an emission of its metallic
    # value, then put them back (materials and meshes are shared).
    saved = [(h, i, s.material) for h in highs for i, s in enumerate(h.material_slots)]
    for h, i, m in saved:
        h.material_slots[i].material = emit_material(metallic_of(m))
    scene.cycles.samples = 1
    res["metal"] = run("EMIT", new_image(f"{atlas}_m", size, (0, 0, 0, 1), True))
    for h, i, m in saved:
        h.material_slots[i].material = m
    # AO on the low-poly car itself: every other low visible, the sources and
    # this atlas's own lows hidden (the joined copy stands in for them; both
    # would occlude each other completely).
    for h in highs:
        h.hide_render = True
    own = set(lows)
    for o in all_lows:
        o.hide_render = o in own
    scene.cycles.samples = args.samples * 4
    res["ao"] = run("AO", new_image(f"{atlas}_ao", size, (1, 1, 1, 1), True), s2a=False)
    for o in all_lows:
        o.hide_render = True

    bc = res["bc"].copy()
    bc[..., 3] = 1
    save_png(bc, os.path.join(OUT, f"T_IFS08_{atlas}_BC.png"), False)
    n = res["n"].copy()
    n[..., 3] = 1
    save_png(n, os.path.join(OUT, f"T_IFS08_{atlas}_N.png"), True)
    orm = np.ones_like(bc)
    orm[..., 0] = res["ao"][..., 0]
    orm[..., 1] = res["rough"][..., 0]
    orm[..., 2] = res["metal"][..., 0]
    save_png(orm, os.path.join(OUT, f"T_IFS08_{atlas}_ORM.png"), True)
    bpy.data.objects.remove(j, do_unlink=True)


# --- 6. export --------------------------------------------------------------------

def to_export_frame(origin):
    """Scene (x left, y rear, z up) -> export (x forward, y left, z up), origin at `origin`."""
    rot = Matrix.Rotation(math.radians(90), 4, "Z")   # (x, y) -> (-y, x): -y (forward) -> +x
    return rot @ Matrix.Translation(-origin)


def join_into(objs, name, xf):
    j = joined_copy(objs, name)
    j.matrix_world = xf @ j.matrix_world
    with bpy.context.temp_override(active_object=j, selected_editable_objects=[j], object=j):
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    prune_slots(j)
    return j


def slot_material(name, cache={}):
    if name not in cache:
        cache[name] = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    return cache[name]


def assign_slot(o, slot):
    o.data.materials.clear()
    o.data.materials.append(slot_material(slot))


def export_fbx(path, objs, armature=None):
    sel = list(objs) + ([armature] if armature else [])
    select_only(sel)
    with bpy.context.temp_override(selected_objects=sel, active_object=sel[0]):
        bpy.ops.export_scene.fbx(
            filepath=path, use_selection=True, object_types={"MESH", "ARMATURE"},
            apply_unit_scale=True, apply_scale_options="FBX_SCALE_NONE", axis_forward="-Z", axis_up="Y",
            use_mesh_modifiers=True, mesh_smooth_type="FACE", use_tspace=False, add_leaf_bones=False,
            bake_anim=False, path_mode="STRIP", use_armature_deform_only=False)


def main():
    objs = prepare()
    wheels, ground_z = find_wheels(objs)
    for k, w in wheels.items():
        log(f"wheel {k}: centre {[round(v, 4) for v in w['centre']]} r {w['radius']:.4f} w {w['width']:.3f}")
    front_y = (wheels["FL"]["centre"][1] + wheels["FR"]["centre"][1]) / 2
    rear_y = (wheels["RL"]["centre"][1] + wheels["RR"]["centre"][1]) / 2
    wheelbase = rear_y - front_y
    cog_y = front_y + wheelbase * (1 - args.weight_front)
    origin = Vector((0.0, cog_y, ground_z))
    xf = to_export_frame(origin)
    log(f"ground z {ground_z:.4f}, wheelbase {wheelbase:.4f} m, CoG {cog_y - front_y:.4f} m behind the front axle")

    cls = classify(objs, wheels)
    hits = visibility(objs, ground_z)
    json.dump(hits, open(os.path.join(OUT, "visibility.json"), "w"), indent=0)

    # Highs: the parts some ray reached (plus the lit parts, always). Lows: copies to decimate.
    keep = [o for o in objs if hits[o.name] >= 3 or ALWAYS_KEEP_RE.search(o.name)]
    dropped = [o for o in objs if o not in keep]
    log(f"cull: keep {len(keep)} parts ({sum(tri_count(o.data) for o in keep) / 1e6:.2f} M tris), "
        f"drop {len(dropped)} ({sum(tri_count(o.data) for o in dropped) / 1e6:.2f} M tris)")
    for o in dropped:
        bpy.data.objects.remove(o, do_unlink=True)
    geo = [o for o in keep if not BAKE_ONLY_RE.search(tag(o))]
    tris = {o.name: tri_count(o.data) for o in geo}
    target = allocate(tris, {k: max(hits[k], 2) for k in tris}, args.budget)
    lows = []
    for o in geo:
        c = o.copy()
        c.data = o.data.copy()
        scene.collection.objects.link(c)
        c["src"] = o.name
        c["part"] = "/".join(x for x in cls[o.name] if x)
        lows.append(c)
    decimate(lows, target)
    fx = [f for f in (split_fx(o) for o in lows) if f]
    lows = [o for o in lows if len(o.data.polygons)]
    log(f"decimated: {sum(tri_count(o.data) for o in lows) / 1e3:.0f}k tris + "
        f"{sum(tri_count(o.data) for o in fx) / 1e3:.0f}k FX tris")

    # Atlases: livery (bodywork panels), mechanical (rest of body + corners), wheels.
    def atlas_of(src_name):
        if cls[src_name][0] == "wheel":
            return "Wheels"
        src = bpy.data.objects[src_name]
        if any(c.name == "Bodywork" or c.name.startswith(("Morro", "Suelo", "Frontwing", "Rear Wing"))
               for c in src.users_collection) or any(
                s.material and re.search(r"Livery|Carbon", s.material.name) for s in src.material_slots):
            return "Livery"
        return "Mech"
    groups = {}
    for o in lows:
        groups.setdefault(atlas_of(o["src"]), []).append(o)
    sources = {}
    for o in keep:
        sources.setdefault(atlas_of(o.name), []).append(o)

    if not args.no_bake:
        setup_cycles()
    for a, objs_a in groups.items():
        log(f"atlas {a}: UV")
        atlas_uv(objs_a)
        if not args.no_bake:
            bake_atlas(a, objs_a, sources.get(a, []), lows, args.tex)
        for o in objs_a:
            assign_slot(o, f"MI_IFS08_{a}")

    # FX slots keep the source material's name.
    for f in fx:
        for i, s in enumerate(f.material_slots):
            if s.material:
                nm = "FX_" + re.sub(r"[^A-Za-z0-9]+", "_", s.material.name).strip("_")
                f.data.materials[i] = slot_material(nm)
    for o in list(scene.objects):
        if o in keep:
            o.hide_render = True

    # --- export -------------------------------------------------------------
    manifest = {"source": os.path.basename(bpy.data.filepath), "frame": "UE cm, x forward, y right, z up",
                "weight_front": args.weight_front, "wheelbase_cm": wheelbase * 100,
                "ground_z_scene_m": ground_z, "wheels": {}, "meshes": {}}
    body = [o for o in lows + fx if cls[o["src"]][0] == "body"]
    for k in CORNERS:
        w = wheels[k]
        hub = xf @ Vector(w["centre"])
        manifest["wheels"][k] = {"hub_cm": [hub.x * 100, -hub.y * 100, hub.z * 100],
                                 "radius_cm": w["radius"] * 100, "width_cm": w["width"] * 100}
        hub_xf = Matrix.Translation(-hub) @ xf
        for kind in ("wheel", "corner"):
            parts = [o for o in lows + fx if cls[o["src"]] == (kind, k)]
            if not parts:
                continue
            name = f"SM_IFS08_{kind.capitalize()}_{k}"
            j = join_into(parts, name, hub_xf)
            export_fbx(os.path.join(OUT, name + ".fbx"), [j])
            manifest["meshes"][name] = {"tris": tri_count(j.data), "slots": [m.name for m in j.data.materials]}
            log(f"exported {name}: {tri_count(j.data) / 1e3:.1f}k tris")

    arm_data = bpy.data.armatures.new("IFS08_Skeleton")
    arm = bpy.data.objects.new("Armature", arm_data)
    scene.collection.objects.link(arm)
    select_only([arm])
    with bpy.context.temp_override(active_object=arm, object=arm, selected_editable_objects=[arm]):
        bpy.ops.object.mode_set(mode="EDIT")
        root = arm_data.edit_bones.new("root")
        root.head, root.tail = (0, 0, 0), (0, 0, 0.3)
        for k in CORNERS:
            hub = xf @ Vector(wheels[k]["centre"])
            b = arm_data.edit_bones.new("Wheel" + k)
            b.head, b.tail = hub, hub + Vector((0, 0, 0.15))
            b.parent = root
        bpy.ops.object.mode_set(mode="OBJECT")
    j = join_into(body, "SK_IFS08_Body", xf)
    vg = j.vertex_groups.new(name="root")
    vg.add(list(range(len(j.data.vertices))), 1.0, "REPLACE")
    j.parent = arm
    mod = j.modifiers.new("arm", "ARMATURE")
    mod.object = arm
    export_fbx(os.path.join(OUT, "SK_IFS08.fbx"), [j], armature=arm)
    lo = Vector([min(v.co[i] for v in j.data.vertices) for i in range(3)])
    hi = Vector([max(v.co[i] for v in j.data.vertices) for i in range(3)])
    manifest["meshes"]["SK_IFS08"] = {"tris": tri_count(j.data), "slots": [m.name for m in j.data.materials],
                                      "bounds_cm": [[lo.x * 100, -hi.y * 100, lo.z * 100],
                                                    [hi.x * 100, -lo.y * 100, hi.z * 100]]}
    log(f"exported SK_IFS08: {tri_count(j.data) / 1e3:.1f}k tris, slots {[m.name for m in j.data.materials]}")
    json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)

    if args.preview:
        preview([o for o in scene.objects if o.name.startswith(("SK_IFS08_Body", "SM_IFS08_"))], wheels, xf)
    log("done")


def preview(objs, wheels, xf):
    """EEVEE stills of the exported meshes with the baked textures."""
    for k in CORNERS:
        hub = xf @ Vector(wheels[k]["centre"])
        for o in objs:
            if o.name.endswith("_" + k):
                o.location = hub
    for o in scene.objects:
        o.hide_render = o not in objs
    for m in {s.material for o in objs for s in o.material_slots if s.material}:
        m.use_nodes = True
        nt = m.node_tree
        nt.nodes.clear()
        bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
        out = nt.nodes.new("ShaderNodeOutputMaterial")
        nt.links.new(bsdf.outputs[0], out.inputs[0])
        a = m.name.replace("MI_IFS08_", "")
        bc = os.path.join(OUT, f"T_IFS08_{a}_BC.png")
        if os.path.exists(bc):
            t = nt.nodes.new("ShaderNodeTexImage")
            t.image = bpy.data.images.load(bc)
            nt.links.new(t.outputs[0], bsdf.inputs["Base Color"])
            orm = nt.nodes.new("ShaderNodeTexImage")
            orm.image = bpy.data.images.load(os.path.join(OUT, f"T_IFS08_{a}_ORM.png"))
            orm.image.colorspace_settings.name = "Non-Color"
            sep = nt.nodes.new("ShaderNodeSeparateColor")
            nt.links.new(orm.outputs[0], sep.inputs[0])
            nt.links.new(sep.outputs[1], bsdf.inputs["Roughness"])
            nt.links.new(sep.outputs[2], bsdf.inputs["Metallic"])
            nm = nt.nodes.new("ShaderNodeTexImage")
            nm.image = bpy.data.images.load(os.path.join(OUT, f"T_IFS08_{a}_N.png"))
            nm.image.colorspace_settings.name = "Non-Color"
            nmap = nt.nodes.new("ShaderNodeNormalMap")
            nt.links.new(nm.outputs[0], nmap.inputs["Color"])
            nt.links.new(nmap.outputs[0], bsdf.inputs["Normal"])
        elif a.startswith("FX_"):
            bsdf.inputs["Base Color"].default_value = (0.9, 0.1, 0.1, 1) if "red" in a.lower() else (0.8, 0.8, 0.8, 1)
        else:
            bsdf.inputs["Base Color"].default_value = (0.5, 0.5, 0.5, 1)
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x, scene.render.resolution_y = 1600, 900
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.use_nodes = True
    bg = scene.world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.6, 0.65, 0.7, 1)
        bg.inputs["Strength"].default_value = 1.0
    sun = bpy.data.objects.new("PreviewSun", bpy.data.lights.new("PreviewSun", "SUN"))
    sun.data.energy = 4
    sun.rotation_euler = (math.radians(40), 0, math.radians(30))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("PreviewCam", bpy.data.cameras.new("PreviewCam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    for name, pos in {"front34": (3.2, -2.6, 1.4), "side": (0.0, -4.2, 0.6), "rear34": (-3.0, 2.4, 1.6),
                      "top": (0.01, 0.0, 5.0)}.items():
        cam.location = pos
        d = Vector((0, 0, 0.35)) - Vector(pos)
        cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = os.path.join(OUT, f"preview_{name}.png")
        bpy.ops.render.render(write_still=True)
        log(f"preview {name}")


main()
