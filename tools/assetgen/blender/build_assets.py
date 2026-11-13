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
FOLIAGE = set(MATERIALS["foliage"])          # leaf cards: their own 0..1 UVs
SLOTS = set(MATERIALS["materials"]) | set(MATERIALS["flat"]) | FOLIAGE | {"ChainLink"}


# --- mesh building ----------------------------------------------------------------

class Part:
    """One bmesh per prop; every primitive carries its material slot."""

    def __init__(self, name):
        self.name = name
        self.bm = bmesh.new()
        self.slots = []
        self.uv = self.bm.loops.layers.uv.new("UVMap")
        # Leaf cards shade as if the crown were one soft volume: each card
        # remembers the centre of the clump it belongs to (see soft_normals).
        self.clump = self.bm.faces.layers.float_vector.new("clump")

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

    def cone(self, a, b, r0, r1, mat, segments=8):
        """Tapered round section from a (radius r0) to b (radius r1), open ends."""
        a, b = Vector(a), Vector(b)
        axis = b - a
        rot = Vector((0, 0, 1)).rotation_difference(axis.normalized()).to_matrix().to_4x4()
        m = Matrix.Translation((a + b) / 2) @ rot
        before = set(self.bm.faces)
        bmesh.ops.create_cone(self.bm, cap_ends=False, cap_tris=False, segments=segments,
                              radius1=r0, radius2=r1, depth=axis.length, matrix=m)
        self._tag(before, mat, smooth=True)

    def card(self, base, out, up, w, h, mat, clump):
        """A leaf card: a w x h quad standing on `base`, its face along `out`,
        leaning towards `up`; UVs 0..1 (the texture's twig at the bottom)."""
        out, up = Vector(out).normalized(), Vector(up).normalized()
        side = up.cross(out)
        if side.length < 1e-4:
            side = Vector((1, 0, 0))
        side.normalize()
        along = out.cross(side).normalized()        # in the card's plane, away from the twig
        if along.dot(up) < 0:
            along = -along
        base = Vector(base)
        corners = [base - side * w / 2, base + side * w / 2,
                   base + side * w / 2 + along * h, base - side * w / 2 + along * h]
        before = set(self.bm.faces)
        f = self.bm.faces.new([self.bm.verts.new(c) for c in corners])
        for loop, uv in zip(f.loops, ((0, 0), (1, 0), (1, 1), (0, 1))):
            loop[self.uv].uv = uv
        f[self.clump] = Vector(clump)
        self._tag(before, mat, smooth=True)

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
        soft_normals(obj)
        return obj


def metre_uvs(obj):
    """Box projection: each face gets the two coordinates across its normal, in
    metres. Leaf cards keep the 0..1 UVs they were made with."""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    uv = bm.loops.layers.uv.active or bm.loops.layers.uv.new("UVMap")
    keep = {i for i, m in enumerate(obj.data.materials) if m and m.name in FOLIAGE}
    for f in bm.faces:
        if f.material_index in keep:
            continue
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


def soft_normals(obj):
    """Leaf cards get normals pointing out of their clump, so a crown lights
    like a soft round mass and not like a pile of flat cards. Everything else
    keeps its own normals."""
    mesh = obj.data
    attr = mesh.attributes.get("clump")
    if attr is None:
        return
    cards = {i for i, m in enumerate(mesh.materials) if m and m.name in FOLIAGE}
    if not cards:
        mesh.attributes.remove(attr)
        return
    normals = [Vector(n.vector) for n in mesh.corner_normals]
    for poly in mesh.polygons:
        if poly.material_index not in cards:
            continue
        c = Vector(attr.data[poly.index].vector)
        for li in poly.loop_indices:
            v = mesh.vertices[mesh.loops[li].vertex_index].co
            n = (v - c)
            n.z *= 0.6                      # crowns are lit from above: flatten a little
            normals[li] = (n.normalized() * 0.85 + Vector(poly.normal) * 0.15).normalized()
    mesh.normals_split_custom_set(normals)
    mesh.attributes.remove(attr)


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


def garage_bay(open_door=False):
    """One 6 m pit-garage front: roller door, frame, glazed upper floor.

    Origin: the bay's left end at the facade, ground level; it runs to
    x = 6 and its front faces +y (the pit lane). The building box behind it
    stays the world's; this is the face you see from the track.

    open_door: the door rolled up into its box, showing a dark garage with a
    ceiling light, a tool cabinet and a stack of tyres (GarageBayOpen). The
    garage is only as deep as the frame: the building box sits right behind.
    """
    p = Part("GarageBayOpen" if open_door else "GarageBay")
    w, door_w, door_h = 6.0, 5.0, 4.2
    # Pillars, lintel and floor slab edge.
    p.box((0.0, 0.0, 0.0), (0.5, 0.45, 8.25), "ConcreteCream")
    p.box((5.5, 0.0, 0.0), (6.0, 0.45, 8.25), "ConcreteCream")
    p.box((0.5, 0.0, door_h), (5.5, 0.45, 5.0), "ConcreteCream")
    p.box((0.0, 0.0, 7.4), (w, 0.5, 8.25), "ConcreteCream")
    if open_door:
        # The garage behind the opening: dark walls, a light along the ceiling,
        # a cabinet on the left and tyres on the right.
        p.box((0.5, 0.0, 0.0), (5.5, 0.02, door_h), "GarageInterior")
        p.box((0.8, 0.02, door_h - 0.55), (5.2, 0.10, door_h - 0.47), "Lamp")
        p.box((0.8, 0.02, 0.0), (2.1, 0.42, 1.05), "SteelGrey")
        for k in range(4):
            p.box((0.82, 0.42, 0.18 + 0.22 * k), (2.08, 0.43, 0.2 + 0.22 * k), "DoorPanel")   # drawers
        for k in range(3):
            p.tube((4.4, 0.33, 0.25 * k), (4.4, 0.33, 0.25 * k + 0.24), 0.31, "Rubber", segments=16)
    else:
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
    barrier). Sizes are measured on a ground-level photo of the real one,
    scaled by the walls beside it: the truss's underside is at 5.0 m and it
    is 1.3 m deep, with a bay about as wide as it is deep. The start lights
    (five columns of four lamps, 2.0 x 1.15 m) hang straight under it, 2 m
    towards the grandstand side, with a camera on a short mast above them;
    a dark board hangs on the pit side.
    """
    p = Part("StartGantry")
    y0, y1, z0, d, h = -12.0, 11.5, 5.0, 0.8, 1.3
    zt = z0 + h
    for y in (y0, y1):
        p.box((-0.15, y - 0.15, 0.0), (0.15, y + 0.15, zt + 0.15), "SteelWhite")
        p.box((-0.4, y - 0.4, 0.0), (0.4, y + 0.4, 0.3), "ConcreteWall")
    for (x, z) in [(-d / 2, z0), (d / 2, z0), (-d / 2, zt), (d / 2, zt)]:
        p.tube((x, y0, z), (x, y1, z), 0.06, "SteelWhite", segments=8)
    n = 18
    ys = [y0 + (y1 - y0) * k / n for k in range(n + 1)]
    for k, y in enumerate(ys):
        # Verticals on both faces, cross members top and bottom.
        for x in (-d / 2, d / 2):
            p.tube((x, y, z0), (x, y, zt), 0.035, "SteelWhite", segments=6, caps=False)
        for z in (z0, zt):
            p.tube((-d / 2, y, z), (d / 2, y, z), 0.03, "SteelWhite", segments=6, caps=False)
        if k == n:
            break
        ya, yb = y, ys[k + 1]
        # Face diagonals alternate, so the faces read as a row of Vs.
        za, zb = (z0, zt) if k % 2 == 0 else (zt, z0)
        for x in (-d / 2, d / 2):
            p.tube((x, ya, za), (x, yb, zb), 0.03, "SteelWhite", segments=6, caps=False)
        for z in (z0, zt):
            p.tube((-d / 2, ya, z), (d / 2, yb, z), 0.025, "SteelWhite", segments=6, caps=False)
    # Start lights, facing the grid (-x): five columns, each a housing with
    # four lenses under visors; the lower two light red, the upper two stay dark.
    yl, bx = 2.0, -d / 2
    for k in range(5):
        y = yl + 0.4 * (k - 2)
        p.box((bx - 0.25, y - 0.16, z0 - 1.15), (bx, y + 0.16, z0 - 0.02), "Rubber")
        for i in range(4):
            z = z0 - 0.17 - 0.27 * i
            p.tube((bx - 0.25, y, z), (bx - 0.28, y, z), 0.11, "LampRed" if i >= 2 else "PanelDark",
                   segments=16)
            p.box((bx - 0.42, y - 0.13, z + 0.11), (bx - 0.25, y + 0.13, z + 0.13), "Rubber")
    # Camera mast on the top chord, over the lights: a dome on an arm.
    p.tube((-d / 2, yl, zt), (-d / 2, yl, zt + 0.65), 0.03, "SteelGrey", segments=8)
    p.tube((-d / 2, yl, zt + 0.65), (-d / 2 - 0.25, yl, zt + 0.65), 0.025, "SteelGrey", segments=6)
    p.tube((-d / 2 - 0.3, yl, zt + 0.66), (-d / 2 - 0.3, yl, zt + 0.52), 0.09, "SteelGrey", segments=12)
    p.cone((-d / 2 - 0.3, yl, zt + 0.52), (-d / 2 - 0.3, yl, zt + 0.44), 0.09, 0.05, "PanelDark", segments=12)
    # The dark board on the pit side, hung under the truss.
    yb0, yb1 = -4.7, -2.1
    p.box((bx - 0.15, yb0, z0 - 1.0), (bx, yb1, z0 - 0.1), "Rubber")
    for y in (yb0 + 0.2, yb1 - 0.2):
        p.box((bx - 0.1, y - 0.02, z0 - 0.1), (bx - 0.05, y + 0.02, z0), "SteelGrey")
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


# --- trees ----------------------------------------------------------------------
#
# Every tree is modelled 10 m tall with a 4 m crown radius (the plugin's
# TreeModelHeightM / TreeModelRadiusM) and scaled per instance to the height
# and crown the LiDAR measured. Origin: the trunk's foot.

TREE_H, TREE_R = 10.0, 4.0


def _rng(seed):
    import random
    return random.Random(seed)


def _shell_point(r, c, radii, inner=0.7):
    """A point in the outer part of an ellipsoid (centre c, radii), and the
    outward direction there."""
    while True:
        d = Vector((r.gauss(0, 1), r.gauss(0, 1), r.gauss(0, 1)))
        if d.length > 1e-3:
            break
    d.normalize()
    k = inner + (1 - inner) * r.random() ** 0.5
    p = Vector(c) + Vector((d.x * radii[0], d.y * radii[1], d.z * radii[2])) * k
    out = Vector((d.x / radii[0], d.y / radii[1], d.z / radii[2])).normalized()
    return p, out


def _leaves(p, r, clump, radii, n, size, mat):
    """n cards through the outer half of one clump, facing out, tilted at
    random: deep enough that the crown reads as a mass, not a shell."""
    for _ in range(n):
        base, out = _shell_point(r, clump, radii, inner=0.45)
        jitter = Vector((r.uniform(-0.6, 0.6), r.uniform(-0.6, 0.6), r.uniform(-0.3, 0.6)))
        face = (out + jitter).normalized()
        up = (out * 0.6 + Vector((0, 0, 0.8)) + Vector((r.uniform(-0.5, 0.5), r.uniform(-0.5, 0.5), 0))).normalized()
        sz = size * r.uniform(0.75, 1.25)
        # The twig starts a little inside the crown.
        p.card(base - out * sz * 0.35, face, up, sz, sz, mat, clump)


def _limb(p, r, a, b, r0, r1, mat, bends=2, segments=7):
    """A slightly crooked limb from a to b, as tapered pieces."""
    a, b = Vector(a), Vector(b)
    pts = [a]
    for k in range(1, bends + 1):
        t = k / (bends + 1)
        q = a.lerp(b, t)
        wob = (b - a).length * 0.06
        pts.append(q + Vector((r.uniform(-wob, wob), r.uniform(-wob, wob), r.uniform(-wob, wob) * 0.5)))
    pts.append(b)
    for k in range(len(pts) - 1):
        t0, t1 = k / (len(pts) - 1), (k + 1) / (len(pts) - 1)
        p.cone(pts[k], pts[k + 1], r0 + (r1 - r0) * t0, r0 + (r1 - r0) * t1, mat, segments)


def broadleaf(variant, seed):
    """Holm oak / plane: a short trunk forking into a rounded, slightly lumpy
    crown from about a quarter of the height to the top."""
    r = _rng(seed)
    p = Part(f"TreeBroadleaf{variant}")
    fork = Vector((r.uniform(-0.2, 0.2), r.uniform(-0.2, 0.2), 3.2))
    _limb(p, r, (0, 0, 0), fork, 0.24, 0.17, "Bark", bends=1, segments=9)
    # Lobes: the crown as three to five overlapping clumps.
    lobes = [((0, 0, 6.4), (3.0, 3.0, 2.6))]
    for k in range(r.randint(3, 5)):
        a = r.uniform(0, 2 * math.pi) + k * 1.3
        d = r.uniform(1.4, 2.0)
        lobes.append(((math.cos(a) * d, math.sin(a) * d, r.uniform(5.2, 7.6)),
                      (r.uniform(1.6, 2.1), r.uniform(1.6, 2.1), r.uniform(1.4, 1.9))))
    for c, radii in lobes:
        _limb(p, r, fork, Vector(c) * Vector((0.75, 0.75, 1.0)) + Vector((0, 0, -0.6)), 0.12, 0.05, "Bark")
        n = int(110 * (radii[0] * radii[1] * radii[2]) ** (1 / 3) / 2.0)
        _leaves(p, r, c, radii, n, 1.4, "LeavesBroadleaf")
    return p.finish()


def pine(variant, seed):
    """Aleppo / stone pine: a tall, leaning bare trunk and a flat umbrella of
    needle clumps in the top third."""
    r = _rng(seed)
    p = Part(f"TreePine{variant}")
    lean = Vector((r.uniform(-0.7, 0.7), r.uniform(-0.7, 0.7), 0))
    top = Vector((0, 0, 6.6)) + lean
    _limb(p, r, (0, 0, 0), top, 0.22, 0.12, "Bark", bends=3, segments=9)
    clumps = [(top + Vector((0, 0, 1.9)), (1.9, 1.9, 0.9))]
    for k in range(r.randint(6, 8)):
        a = k * 2 * math.pi / 7 + r.uniform(-0.3, 0.3)
        d = r.uniform(2.0, 2.8)
        c = Vector((math.cos(a) * d, math.sin(a) * d, r.uniform(7.6, 9.0))) + lean * 0.5
        clumps.append((c, (r.uniform(1.2, 1.6), r.uniform(1.2, 1.6), r.uniform(0.6, 0.85))))
    for c, radii in clumps:
        _limb(p, r, top, Vector(c) - Vector((0, 0, 0.5)), 0.09, 0.04, "Bark", bends=1, segments=6)
        n = int(46 * radii[0] * radii[1] / 1.8)
        _leaves(p, r, c, radii, n, 1.15, "LeavesPine")
    return p.finish()


def foliage_textures(size=512, seed=7):
    """Leaf and needle sprigs drawn into colour + opacity maps for the cards:
    Saved/AssetGen/foliage_<Slot>_color.png and _opacity.png. The colour is
    near-grey detail (lighter and darker leaves); the material tints it."""
    import numpy as np
    r = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:size, 0:size] / size          # v up from the twig at the bottom

    def save(name, rgb, alpha):
        # Colour bleeds past the edges so mip maps don't fringe dark.
        fill = rgb[alpha > 0.5].mean(0) if (alpha > 0.5).any() else np.array([0.5, 0.5, 0.5])
        rgb = np.where(alpha[..., None] > 0.02, rgb, fill)
        for tag, img in (("color", rgb), ("opacity", np.repeat(alpha[..., None], 3, -1))):
            im = bpy.data.images.new(f"{name}_{tag}", size, size, alpha=True)
            # Row 0 is the bottom in both: v = 0 is the twig, as on the cards.
            px = np.concatenate([img, np.ones((size, size, 1))], -1).astype(np.float32)
            im.pixels.foreach_set(px.ravel())
            im.filepath_raw = os.path.join(GEN_DIR, f"foliage_{name}_{tag}.png")
            im.file_format = "PNG"
            im.save()

    # Broadleaf: twigs from the bottom centre, small oval leaves along them.
    rgb = np.zeros((size, size, 3)); alpha = np.zeros((size, size))
    for _ in range(7):
        a = r.uniform(-0.65, 0.65)
        x0, y0 = 0.5, 0.02
        dx, dy = math.sin(a), math.cos(a)
        L = r.uniform(0.5, 0.82)
        for t in np.linspace(0.08, L, 22):
            px_, py_ = x0 + dx * t, y0 + dy * t
            for side in (-1, 1):
                ang = a + side * r.uniform(0.5, 1.1)
                cx, cy = px_ + math.sin(ang) * 0.035, py_ + math.cos(ang) * 0.035
                lx, ly = 0.045 * r.uniform(0.8, 1.2), 0.02 * r.uniform(0.8, 1.2)
                ca, sa = math.cos(-ang), math.sin(-ang)
                u = (xx - cx) * ca - (yy - cy) * sa
                v = (xx - cx) * sa + (yy - cy) * ca
                m = (u / ly) ** 2 + (v / lx) ** 2 < 1.0
                shade = r.uniform(0.35, 0.75)
                vein = np.clip(1.0 - 0.25 * np.exp(-(u / 0.003) ** 2), 0, 1)
                rgb[m] = (shade * vein[m])[:, None] * np.array([1.0, 1.0, 1.0])
                alpha[m] = 1.0
    save("LeavesBroadleaf", rgb, alpha)

    # Pine: tufts of long thin needles at the ends of short twigs.
    rgb = np.zeros((size, size, 3)); alpha = np.zeros((size, size))
    for _ in range(26):
        cx, cy = r.uniform(0.15, 0.85), r.uniform(0.25, 0.9)
        shade = r.uniform(0.4, 0.75)
        for _ in range(45):
            a = r.uniform(0, 2 * math.pi)
            L = r.uniform(0.06, 0.13)
            ex, ey = cx + math.cos(a) * L, cy + math.sin(a) * L
            # distance to the segment (cx,cy)-(ex,ey)
            vx, vy = ex - cx, ey - cy
            t = np.clip(((xx - cx) * vx + (yy - cy) * vy) / (vx * vx + vy * vy), 0, 1)
            d = np.hypot(xx - (cx + t * vx), yy - (cy + t * vy))
            m = d < 0.0028
            rgb[m] = shade * r.uniform(0.85, 1.15)
            alpha[m] = 1.0
    save("LeavesPine", rgb, alpha)


# LOD screen sizes: [triangle fraction, screen size] after LOD0.
PROPS = [
    (seat_block, [[0.35, 0.25], [0.08, 0.08]], None),
    (fence_post, [[0.5, 0.05]], None),
    (sign_board, [[0.3, 0.1]], None),
    (garage_bay, [[0.4, 0.3], [0.1, 0.1]], None),
    (lambda: garage_bay(open_door=True), [[0.4, 0.3], [0.1, 0.1]], None),
    (start_gantry, [[0.4, 0.3], [0.15, 0.1]], None),
    (handrail, [[0.4, 0.08]], None),
    (marshal_post, [[0.4, 0.15]], None),
    (roof_column, [], None),
    # Trees: two of each kind. Leaf cards thin out with distance.
    (lambda: broadleaf("A", 11), [[0.5, 0.25], [0.2, 0.08]], None),
    (lambda: broadleaf("B", 23), [[0.5, 0.25], [0.2, 0.08]], None),
    (lambda: pine("A", 31), [[0.5, 0.25], [0.2, 0.08]], None),
    (lambda: pine("B", 47), [[0.5, 0.25], [0.2, 0.08]], None),
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
    foliage_textures()
    print("ASSETGEN done")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
