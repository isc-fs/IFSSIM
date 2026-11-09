"""Model the world props in Blender and export them for tools/assetgen/ue_import.py.

Run headless (docs/world_assets.md):

    Blender -b --factory-startup --python tools/assetgen/blender/build_assets.py

Every prop is modelled at its real size, in metres, in the world frame
(docs/worlds.md): +x along the track, +y to the left, +z up. Each material
slot is named after an entry in tools/assetgen/materials.json; the importer
assigns MI_<slot>. UVs are box-projected in metres, so a material's tile_m is
the real size of one texture repeat on every prop.

Writes Saved/AssetGen/fbx/<Name>.fbx, Saved/AssetGen/fbx/manifest.json and
Saved/AssetGen/chainlink_opacity.png.
"""

import json
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
GEN_DIR = os.path.join(PROJECT, "Saved", "AssetGen")
FBX_DIR = os.path.join(GEN_DIR, "fbx")
MATERIALS = json.load(open(os.path.join(HERE, "..", "materials.json")))
SLOTS = set(MATERIALS["materials"]) | set(MATERIALS["flat"]) | {"ChainLink"}


# --- mesh building ----------------------------------------------------------------

class Part:
    """One bmesh per prop; every primitive carries its material slot."""

    def __init__(self, name):
        self.name = name
        self.bm = bmesh.new()
        self.slots = []

    def _slot(self, mat):
        if mat not in SLOTS:
            raise ValueError(f"{self.name}: unknown material {mat}")
        if mat not in self.slots:
            self.slots.append(mat)
        return self.slots.index(mat)

    def _tag(self, before, mat, smooth=False):
        idx = self._slot(mat)
        for f in set(self.bm.faces) - before:
            f.material_index = idx
            f.smooth = smooth

    def box(self, lo, hi, mat):
        """Axis-aligned box from corner lo to corner hi."""
        before = set(self.bm.faces)
        (x0, y0, z0), (x1, y1, z1) = lo, hi
        v = [self.bm.verts.new(p) for p in (
            (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))]
        for q in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self.bm.faces.new([v[i] for i in q])
        self._tag(before, mat)

    def tube(self, a, b, radius, mat, segments=8, caps=True):
        """Round tube from point a to point b."""
        a, b = Vector(a), Vector(b)
        axis = b - a
        rot = Vector((0, 0, 1)).rotation_difference(axis.normalized()).to_matrix().to_4x4()
        m = Matrix.Translation((a + b) / 2) @ rot
        before = set(self.bm.faces)
        bmesh.ops.create_cone(self.bm, cap_ends=caps, cap_tris=False, segments=segments,
                              radius1=radius, radius2=radius, depth=axis.length, matrix=m)
        self._tag(before, mat, smooth=True)
        # Caps stay flat.
        for f in set(self.bm.faces) - before:
            if abs(f.normal.normalized().dot(axis.normalized())) > 0.99:
                f.smooth = False

    def beam(self, a, b, w, h, mat, up=(0, 0, 1)):
        """Rectangular section w x h from a to b (a steel tube or timber)."""
        a, b = Vector(a), Vector(b)
        d = (b - a).normalized()
        up = Vector(up)
        if abs(d.dot(up)) > 0.99:
            up = Vector((1, 0, 0))
        s = d.cross(up).normalized() * (w / 2)
        u = s.cross(d).normalized() * (h / 2)
        before = set(self.bm.faces)
        corners = [-s - u, s - u, s + u, -s + u]
        va = [self.bm.verts.new(a + c) for c in corners]
        vb = [self.bm.verts.new(b + c) for c in corners]
        self.bm.faces.new(va[::-1])
        self.bm.faces.new(vb)
        for i in range(4):
            j = (i + 1) % 4
            self.bm.faces.new((va[i], va[j], vb[j], vb[i]))
        self._tag(before, mat)

    def sheet(self, rows, mat, thickness, smooth=True):
        """A shell through a grid of points rows[i][j], given the thickness."""
        before = set(self.bm.faces)
        n = len(rows[0])
        top = [[self.bm.verts.new(p) for p in r] for r in rows]
        # Offset along the local normal for the underside.
        bot = []
        for i, r in enumerate(rows):
            br = []
            for j, p in enumerate(r):
                pi = Vector(rows[min(i + 1, len(rows) - 1)][j]) - Vector(rows[max(i - 1, 0)][j])
                pj = Vector(r[min(j + 1, n - 1)]) - Vector(r[max(j - 1, 0)])
                nrm = pj.cross(pi).normalized()
                br.append(self.bm.verts.new(Vector(p) - nrm * thickness))
            bot.append(br)
        for i in range(len(rows) - 1):
            for j in range(n - 1):
                self.bm.faces.new((top[i][j], top[i][j + 1], top[i + 1][j + 1], top[i + 1][j]))
                self.bm.faces.new((bot[i][j], bot[i + 1][j], bot[i + 1][j + 1], bot[i][j + 1]))
        rim = ([(0, j) for j in range(n)] + [(i, n - 1) for i in range(1, len(rows))]
               + [(len(rows) - 1, j) for j in range(n - 2, -1, -1)] + [(i, 0) for i in range(len(rows) - 2, 0, -1)])
        for k in range(len(rim)):
            (i0, j0), (i1, j1) = rim[k], rim[(k + 1) % len(rim)]
            self.bm.faces.new((top[i0][j0], bot[i0][j0], bot[i1][j1], top[i1][j1]))
        self._tag(before, mat, smooth=smooth)

    def finish(self, bevel=0.0):
        """Weld, fix normals, project metre UVs, return a Blender object."""
        bmesh.ops.remove_doubles(self.bm, verts=self.bm.verts, dist=1e-5)
        bmesh.ops.recalc_face_normals(self.bm, faces=self.bm.faces)
        mesh = bpy.data.meshes.new(self.name)
        self.bm.to_mesh(mesh)
        self.bm.free()
        obj = bpy.data.objects.new(self.name, mesh)
        bpy.context.scene.collection.objects.link(obj)
        for s in self.slots:
            mat = bpy.data.materials.get(s) or bpy.data.materials.new(s)
            obj.data.materials.append(mat)
        if bevel > 0:
            # A few millimetres of bevel catch the light on every hard edge.
            mod = obj.modifiers.new("bevel", "BEVEL")
            mod.width = bevel
            mod.segments = 1
            mod.limit_method = "ANGLE"
            mod.angle_limit = math.radians(40)
            mod.harden_normals = True
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.modifier_apply(modifier=mod.name)
        metre_uvs(obj)
        return obj


def metre_uvs(obj):
    """Box projection: each face gets the two coordinates across its normal, in metres."""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    uv = bm.loops.layers.uv.new("UVMap")
    for f in bm.faces:
        n = f.normal
        ax = max(range(3), key=lambda k: abs(n[k]))
        u, v = {0: (1, 2), 1: (0, 2), 2: (0, 1)}[ax]
        sign = 1 if n[ax] >= 0 else -1
        for loop in f.loops:
            co = loop.vert.co
            # Mirror u on back faces so text-like detail never reads backwards.
            loop[uv].uv = (co[u] * (sign if ax != 2 else 1), co[v])
    bm.to_mesh(obj.data)
    bm.free()


# --- the props ------------------------------------------------------------------

def seat_block():
    """Ten stadium bucket seats at 0.5 m pitch on one 0.85 m tread.

    Origin: the tread's front edge (towards the track) at x = 0; the block runs
    to x = 5. Seats face -y (the track), set at the back of the tread with
    legroom in front, on a steel pedestal.
    """
    p = Part("SeatBlock")
    for k in range(10):
        cx = 0.25 + 0.5 * k
        # Shell: pan then back, as rows of points across the seat (x) and
        # along its profile (y, z); the sides lift to make the bucket.
        profile = [(0.36, 0.43), (0.44, 0.445), (0.60, 0.43), (0.70, 0.44),
                   (0.75, 0.53), (0.78, 0.67), (0.80, 0.80)]
        xs = [-0.22, -0.18, 0.0, 0.18, 0.22]
        rows = []
        for (y, z) in profile:
            r = []
            for x in xs:
                edge = abs(x) / 0.22
                lift = 0.035 * edge ** 3
                back = z > 0.47
                r.append((cx + x * (0.94 if back else 1.0),
                          y - (lift if back else 0.0),
                          z + (0.0 if back else lift)))
            rows.append(r)
        p.sheet(rows, "SeatPlastic", 0.012)
        # Pedestal: a square steel post on a floor plate.
        p.box((cx - 0.03, 0.52, 0.0), (cx + 0.03, 0.58, 0.425), "SteelGrey")
        p.box((cx - 0.08, 0.47, 0.0), (cx + 0.08, 0.63, 0.008), "SteelGrey")
        p.box((cx - 0.12, 0.48, 0.405), (cx + 0.12, 0.60, 0.425), "SteelGrey")
    # No bevel: 1,550 blocks, and a 3 mm edge is invisible beyond a few metres.
    return p.finish(bevel=0.0)


def fence_post():
    """Catch-fence post: 76 mm steel tube, 1 m tall, scaled to the fence height.

    Origin: post centre on the wall top.
    """
    p = Part("FencePost")
    p.tube((0, 0, 0), (0, 0, 1.0), 0.038, "SteelGalv", segments=10)
    p.box((-0.09, -0.09, 0.0), (0.09, 0.09, 0.01), "SteelGalv")
    return p.finish(bevel=0.002)


def sign_board():
    """Barrier advertising board, 3.0 x 0.9 m, generic two-tone panel.

    Origin: bottom centre of the board's back face; the face looks along -y.
    """
    p = Part("SignBoard")
    p.box((-1.5, -0.03, 0.0), (1.5, 0.0, 0.9), "SignPanel")
    p.box((-1.48, -0.032, 0.62), (1.48, -0.03, 0.88), "SignBlue")
    p.box((-1.48, -0.032, 0.02), (-0.9, -0.03, 0.6), "SignBlue")
    for x in (-1.5, 1.5):
        p.box((x - 0.02, -0.035, -0.0), (x + 0.02, 0.0, 0.9), "SteelGrey")
    p.box((-1.5, -0.035, 0.88), (1.5, 0.0, 0.9), "SteelGrey")
    p.box((-1.5, -0.035, 0.0), (1.5, 0.0, 0.02), "SteelGrey")
    return p.finish(bevel=0.003)


def garage_bay():
    """One 6 m pit-garage front: roller door, frame, glazed upper floor.

    Origin: the bay's left end at the facade, ground level; it runs to
    x = 6 and its front faces +y (the pit lane). The building box behind it
    stays the world's; this is the face you see from the track.
    """
    p = Part("GarageBay")
    w, door_w, door_h = 6.0, 5.0, 4.2
    # Pillars, lintel and floor slab edge.
    p.box((0.0, 0.0, 0.0), (0.5, 0.45, 8.25), "ConcreteCream")
    p.box((5.5, 0.0, 0.0), (6.0, 0.45, 8.25), "ConcreteCream")
    p.box((0.5, 0.0, door_h), (5.5, 0.45, 5.0), "ConcreteCream")
    p.box((0.0, 0.0, 7.4), (w, 0.5, 8.25), "ConcreteCream")
    # Roller door, recessed, with its slat ribs and bottom rail.
    p.box((0.5, 0.0, 0.0), (5.5, 0.15, door_h), "DoorPanel")
    for k in range(1, 14):
        z = k * door_h / 14
        p.box((0.5, 0.15, z - 0.012), (5.5, 0.165, z + 0.012), "DoorPanel")
    p.box((0.5, 0.15, 0.0), (5.5, 0.19, 0.08), "Rubber")
    p.box((0.5, 0.15, door_h - 0.25), (5.5, 0.30, door_h), "SteelGrey")  # door box
    # Upper floor glazing with mullions and a transom.
    p.box((0.5, 0.05, 5.0), (5.5, 0.10, 7.4), "Glass")
    for x in (0.5, 1.75, 3.0, 4.25, 5.5):
        p.box((x - 0.04, 0.10, 5.0), (x + 0.04, 0.20, 7.4), "SteelGrey")
    p.box((0.5, 0.10, 5.0), (5.5, 0.20, 5.08), "SteelGrey")
    p.box((0.5, 0.10, 6.95), (5.5, 0.20, 7.02), "SteelGrey")
    # Balcony slab and rail in front of the glazing.
    p.box((0.0, 0.45, 4.85), (w, 1.6, 5.0), "ConcreteCream")
    for x in (0.1, 1.5, 3.0, 4.5, 5.9):
        p.box((x - 0.025, 1.5, 5.0), (x + 0.025, 1.55, 6.05), "SteelWhite")
    p.box((0.0, 1.49, 6.0), (w, 1.56, 6.06), "SteelWhite")
    p.box((0.0, 1.5, 5.45), (w, 1.54, 5.48), "SteelWhite")
    return p.finish(bevel=0.004)


def start_gantry():
    """Start-line gantry: two columns and a box truss over the track.

    Origin: the start line on the track axis, at ground level. Columns stand
    at y = -12.0 (pit side, behind the pit wall) and y = +11.5 (behind the
    barrier); the truss's underside is at 6.0 m.
    """
    p = Part("StartGantry")
    y0, y1, z0, d, h = -12.0, 11.5, 6.0, 0.7, 0.9
    for y in (y0, y1):
        p.box((-0.25, y - 0.25, 0.0), (0.25, y + 0.25, z0 + h + 0.3), "SteelWhite")
        p.box((-0.4, y - 0.4, 0.0), (0.4, y + 0.4, 0.3), "ConcreteWall")
    chords = [(-d / 2, z0), (d / 2, z0), (-d / 2, z0 + h), (d / 2, z0 + h)]
    for (x, z) in chords:
        p.tube((x, y0, z), (x, y1, z), 0.05, "SteelWhite", segments=8)
    n = 16
    for k in range(n):
        ya, yb = y0 + (y1 - y0) * k / n, y0 + (y1 - y0) * (k + 1) / n
        for x in (-d / 2, d / 2):
            p.tube((x, ya, z0), (x, yb, z0 + h), 0.025, "SteelWhite", segments=6, caps=False)
        for z in (z0, z0 + h):
            p.tube((-d / 2, ya, z), (d / 2, yb, z), 0.025, "SteelWhite", segments=6, caps=False)
        p.tube((-d / 2, ya, z0), (-d / 2, ya, z0 + h), 0.025, "SteelWhite", segments=6, caps=False)
    # Start lights under the truss, facing the grid (-x): a housing and five
    # modules, each with two red lenses under visors.
    bx = -d / 2 - 0.05
    p.box((bx - 0.30, -1.75, z0 - 1.05), (bx, 1.75, z0 + 0.02), "Rubber")   # black housing
    for k in range(5):
        y = -1.36 + 0.68 * k
        p.box((bx - 0.38, y - 0.26, z0 - 1.0), (bx - 0.30, y + 0.26, z0 - 0.05), "Rubber")
        for z in (z0 - 0.33, z0 - 0.73):
            p.tube((bx - 0.38, y, z), (bx - 0.42, y, z), 0.15, "LampRed", segments=16)
            # Visor: a hood over the top of the lens.
            p.box((bx - 0.62, y - 0.17, z + 0.15), (bx - 0.38, y + 0.17, z + 0.18), "Rubber")
            p.box((bx - 0.62, y - 0.18, z - 0.02), (bx - 0.38, y - 0.15, z + 0.18), "Rubber")
            p.box((bx - 0.62, y + 0.15, z - 0.02), (bx - 0.38, y + 0.18, z + 0.18), "Rubber")
    return p.finish(bevel=0.004)


def handrail():
    """2.5 m of steel handrail: posts, top rail at 1.1 m, mid rail.

    Origin: one end at floor level; it runs to x = 2.5.
    """
    p = Part("Handrail")
    p.tube((0, 0, 1.1), (2.5, 0, 1.1), 0.024, "SteelGalv", segments=8)
    p.tube((0, 0, 0.55), (2.5, 0, 0.55), 0.016, "SteelGalv", segments=6)
    p.tube((0.05, 0, 0.0), (0.05, 0, 1.1), 0.024, "SteelGalv", segments=8)
    p.box((-0.03, -0.06, 0.0), (0.13, 0.06, 0.01), "SteelGalv")
    return p.finish(bevel=0.0)


def marshal_post():
    """Marshal post: a 2.4 x 1.6 m steel booth with a glazed front and a flag box.

    Origin: floor centre; the window faces -y.
    """
    p = Part("MarshalPost")
    p.box((-1.2, -0.8, 0.0), (1.2, 0.8, 0.15), "ConcreteWall")
    # Walls with a window band on the front and sides.
    p.box((-1.2, 0.72, 0.15), (1.2, 0.8, 2.3), "SteelWhite")
    for x in (-1.2, 1.12):
        p.box((x, -0.8, 0.15), (x + 0.08, 0.8, 1.0), "SteelWhite")
        p.box((x, -0.8, 1.9), (x + 0.08, 0.8, 2.3), "SteelWhite")
        p.box((x + 0.02, -0.8, 1.0), (x + 0.06, 0.72, 1.9), "Glass")
    p.box((-1.2, -0.8, 0.15), (1.2, -0.72, 1.0), "SteelWhite")
    p.box((-1.2, -0.8, 1.9), (1.2, -0.72, 2.3), "SteelWhite")
    p.box((-1.12, -0.78, 1.0), (1.12, -0.74, 1.9), "Glass")
    for x in (-1.2, -0.04, 1.12):
        p.box((x, -0.8, 1.0), (x + 0.08, -0.72, 1.9), "SteelGrey")
    # Roof with overhang, and the flag box under the window.
    p.box((-1.35, -1.0, 2.3), (1.35, 0.95, 2.42), "SteelGrey")
    p.box((-0.6, -0.95, 0.6), (0.6, -0.8, 0.95), "SteelGrey")
    return p.finish(bevel=0.004)


def roof_column():
    """Grandstand roof column: a 400 x 300 mm I-section, 1 m, scaled to height.

    Origin: base centre.
    """
    p = Part("RoofColumn")
    p.box((-0.15, -0.2, 0.0), (0.15, -0.18, 1.0), "SteelWhite")
    p.box((-0.15, 0.18, 0.0), (0.15, 0.2, 1.0), "SteelWhite")
    p.box((-0.006, -0.18, 0.0), (0.006, 0.18, 1.0), "SteelWhite")
    return p.finish(bevel=0.0)


# LOD screen sizes: [triangle fraction, screen size] after LOD0.
PROPS = [
    (seat_block, [[0.35, 0.25], [0.08, 0.08]], None),
    (fence_post, [[0.5, 0.05]], None),
    (sign_board, [[0.3, 0.1]], None),
    (garage_bay, [[0.4, 0.3], [0.1, 0.1]], None),
    (start_gantry, [[0.4, 0.3], [0.15, 0.1]], None),
    (handrail, [[0.4, 0.08]], None),
    (marshal_post, [[0.4, 0.15]], None),
    (roof_column, [], None),
]


# --- chain-link mask ------------------------------------------------------------

def chainlink_mask(path, size=256):
    """Opacity for 50 mm diamond chain-link, 3 mm wire; the texture spans 0.1 m."""
    wire = 0.003 / 0.1 * size
    img = bpy.data.images.new("chainlink", size, size, alpha=False, float_buffer=False)
    px = [0.0] * (size * size * 4)
    for j in range(size):
        for i in range(size):
            # Two diamond lattices of lines at +-45 degrees, period size/2.
            per = size / 2
            a = (i + j) % per
            b = (i - j) % per
            d = min(a, per - a, b, per - b) / math.sqrt(2)
            v = 1.0 if d < wire / 2 else 0.0
            k = 4 * (j * size + i)
            px[k:k + 4] = (v, v, v, 1.0)
    img.pixels = px
    img.filepath_raw = path
    img.file_format = "PNG"
    img.save()


# --- export ---------------------------------------------------------------------

def export(obj, path):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.fbx(filepath=path, use_selection=True, apply_scale_options="FBX_SCALE_UNITS",
                             axis_forward="-Y", axis_up="Z", mesh_smooth_type="FACE",
                             use_mesh_modifiers=True, add_leaf_bones=False, bake_anim=False)


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0
    os.makedirs(FBX_DIR, exist_ok=True)
    manifest = []
    for build, lods, collision in PROPS:
        obj = build()
        path = os.path.join(FBX_DIR, f"{obj.name}.fbx")
        export(obj, path)
        tris = sum(len(p.vertices) - 2 for p in obj.data.polygons)
        lo = [min(v.co[k] for v in obj.data.vertices) for k in range(3)]
        hi = [max(v.co[k] for v in obj.data.vertices) for k in range(3)]
        manifest.append({"name": obj.name, "file": os.path.basename(path), "lods": lods,
                         "collision": collision, "triangles": tris,
                         "bounds_m": [[round(c, 3) for c in lo], [round(c, 3) for c in hi]]})
        print(f"ASSETGEN {obj.name}: {tris} triangles, slots {[m.name for m in obj.data.materials]}")
    json.dump(manifest, open(os.path.join(FBX_DIR, "manifest.json"), "w"), indent=2)
    chainlink_mask(os.path.join(GEN_DIR, "chainlink_opacity.png"))
    print("ASSETGEN done")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
