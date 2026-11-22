"""Import the IFS-08 car into the project (runs inside the UE editor).

    UnrealEditor-Cmd IFSSIM.uproject -run=pythonscript -EnablePlugins=PythonScriptPlugin \
        -script="tools/assetgen/ue_import_vehicle.py" -unattended -nullrhi

Reads what tools/assetgen/blender/build_ifs08_vehicle.py exported (folder from
IFSSIM_VEHICLE_DIR) and writes /Game/Vehicle/IFS08:

    SK_IFS08, SK_IFS08_Skeleton    body: bones root + WheelFL/FR/RL/RR at the hub centres
    PA_IFS08                       one box on the root bone, the car's collision body
    SM_IFS08_Wheel_<c>             what spins, pivot at the hub (FL, FR, RL, RR)
    SM_IFS08_Corner_<c>            upright and caliper: steer and travel, do not spin
    Textures/T_IFS08_<atlas>_{BC,N,ORM}
    Materials/M_IFS08_Baked        BaseColor + Normal + ORM (AO, roughness, metallic)
    Materials/M_IFS08_FX           an LED or the TSAL: Color, and Glow (0 = off)
    Materials/M_IFS08_Screen       the dash display: Screen texture, Brightness
    Materials/MI_IFS08_<atlas>, MI_FX_<name>

LODs are generated here. Each run rebuilds everything; nothing is hand-edited.
"""

import json
import os

import unreal

PROJECT = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
SRC = os.environ.get("IFSSIM_VEHICLE_DIR")
if not SRC or not os.path.isdir(SRC):
    raise RuntimeError("set IFSSIM_VEHICLE_DIR to the folder build_ifs08_vehicle.py wrote")
MANIFEST = json.load(open(os.path.join(SRC, "manifest.json")))
DASH = os.environ.get("IFSSIM_VEHICLE_DASH")   # optional: the dash screen's default image

ROOT = "/Game/Vehicle/IFS08"
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
CORNERS = ("FL", "FR", "RL", "RR")


def log(msg):
    unreal.log("IFS08 " + msg)


def fresh(path):
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)


def import_task(filename, dest, name, options=None):
    task = unreal.AssetImportTask()
    task.filename = filename
    task.destination_path = dest
    task.destination_name = name
    task.automated = True
    task.replace_existing = True
    task.save = False
    if options:
        task.options = options
    TOOLS.import_asset_tasks([task])
    asset = EAL.load_asset(f"{dest}/{name}")
    if not asset:
        raise RuntimeError(f"import failed: {filename}")
    return asset


# --- textures -------------------------------------------------------------------

def import_texture(path, name, kind):
    tex = import_task(path, f"{ROOT}/Textures", name)
    if kind == "normal":
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_NORMALMAP)
        tex.set_editor_property("srgb", False)
        # Blender bakes OpenGL normals (+Y up), UE wants DirectX.
        tex.set_editor_property("flip_green_channel", True)
    elif kind == "mask":
        tex.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
        tex.set_editor_property("srgb", False)
    EAL.save_loaded_asset(tex)
    return tex


# --- materials ------------------------------------------------------------------

def expr(mat, cls, x, y, **props):
    e = MEL.create_material_expression(mat, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def link(a, a_out, b, b_in):
    if not MEL.connect_material_expressions(a, a_out, b, b_in):
        raise RuntimeError(f"cannot connect {a.get_name()}.{a_out} -> {b.get_name()}.{b_in}")


def new_material(name):
    fresh(f"{ROOT}/Materials/{name}")
    mat = TOOLS.create_asset(name, f"{ROOT}/Materials", unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("used_with_skeletal_mesh", True)
    return mat


def build_baked(defaults):
    mat = new_material("M_IFS08_Baked")
    S = unreal.MaterialSamplerType
    bc = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, -200, parameter_name="BaseColor",
              texture=defaults["bc"], sampler_type=S.SAMPLERTYPE_COLOR)
    nrm = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, 100, parameter_name="Normal",
               texture=defaults["n"], sampler_type=S.SAMPLERTYPE_NORMAL)
    orm = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, 400, parameter_name="ORM",
               texture=defaults["orm"], sampler_type=S.SAMPLERTYPE_MASKS)
    P = unreal.MaterialProperty
    MEL.connect_material_property(bc, "RGB", P.MP_BASE_COLOR)
    MEL.connect_material_property(nrm, "RGB", P.MP_NORMAL)
    MEL.connect_material_property(orm, "R", P.MP_AMBIENT_OCCLUSION)
    MEL.connect_material_property(orm, "G", P.MP_ROUGHNESS)
    MEL.connect_material_property(orm, "B", P.MP_METALLIC)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def build_fx():
    """An LED: its colour when off, and Glow x Color x 40 emissive when lit."""
    mat = new_material("M_IFS08_FX")
    col = expr(mat, unreal.MaterialExpressionVectorParameter, -700, 0, parameter_name="Color",
               default_value=unreal.LinearColor(1, 0.1, 0.05, 1))
    glow = expr(mat, unreal.MaterialExpressionScalarParameter, -700, 250, parameter_name="Glow", default_value=0.0)
    off = expr(mat, unreal.MaterialExpressionMultiply, -400, -100, const_b=0.35)
    link(col, "", off, "A")
    em = expr(mat, unreal.MaterialExpressionMultiply, -400, 150)
    link(col, "", em, "A")
    link(glow, "", em, "B")
    em2 = expr(mat, unreal.MaterialExpressionMultiply, -200, 150, const_b=40.0)
    link(em, "", em2, "A")
    P = unreal.MaterialProperty
    MEL.connect_material_property(off, "", P.MP_BASE_COLOR)
    MEL.connect_material_property(em2, "", P.MP_EMISSIVE_COLOR)
    r = expr(mat, unreal.MaterialExpressionConstant, -200, 350, r=0.25)
    MEL.connect_material_property(r, "", P.MP_ROUGHNESS)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def build_screen(default_tex):
    """The dash display: a texture (later a render target), emissive."""
    mat = new_material("M_IFS08_Screen")
    t = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -700, 0, parameter_name="Screen",
             texture=default_tex, sampler_type=unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
    b = expr(mat, unreal.MaterialExpressionScalarParameter, -700, 250, parameter_name="Brightness", default_value=2.0)
    em = expr(mat, unreal.MaterialExpressionMultiply, -400, 100)
    link(t, "RGB", em, "A")
    link(b, "", em, "B")
    P = unreal.MaterialProperty
    black = expr(mat, unreal.MaterialExpressionConstant3Vector, -400, -100,
                 constant=unreal.LinearColor(0.01, 0.01, 0.01, 1))
    MEL.connect_material_property(black, "", P.MP_BASE_COLOR)
    MEL.connect_material_property(em, "", P.MP_EMISSIVE_COLOR)
    r = expr(mat, unreal.MaterialExpressionConstant, -400, 300, r=0.1)
    MEL.connect_material_property(r, "", P.MP_ROUGHNESS)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def instance(name, parent, textures=None, vectors=None, scalars=None):
    fresh(f"{ROOT}/Materials/{name}")
    mi = TOOLS.create_asset(name, f"{ROOT}/Materials", unreal.MaterialInstanceConstant,
                            unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(mi, parent)
    for k, v in (textures or {}).items():
        MEL.set_material_instance_texture_parameter_value(mi, k, v)
    for k, v in (vectors or {}).items():
        MEL.set_material_instance_vector_parameter_value(mi, k, v)
    for k, v in (scalars or {}).items():
        MEL.set_material_instance_scalar_parameter_value(mi, k, v)
    EAL.save_loaded_asset(mi)
    return mi


def fx_colour(slot):
    s = slot.lower()
    for key, c in (("green", (0.05, 1, 0.1)), ("yellow", (1, 0.75, 0.02)), ("red", (1, 0.03, 0.02)),
                   ("tsal", (1, 0.03, 0.02)), ("brake", (1, 0.02, 0.01))):
        if key in s:
            return unreal.LinearColor(*c, 1)
    return unreal.LinearColor(1, 1, 1, 1)


def build_materials():
    atlases = sorted({s[len("MI_IFS08_"):] for m in MANIFEST["meshes"].values() for s in m["slots"]
                      if s.startswith("MI_IFS08_")})
    tex = {}
    for a in atlases:
        base = os.path.join(SRC, f"T_IFS08_{a}")
        if not os.path.exists(base + "_BC.png"):
            raise RuntimeError(f"atlas {a} has no baked textures (run the Blender step without --no-bake)")
        tex[a] = {"bc": import_texture(base + "_BC.png", f"T_IFS08_{a}_BC", "color"),
                  "n": import_texture(base + "_N.png", f"T_IFS08_{a}_N", "normal"),
                  "orm": import_texture(base + "_ORM.png", f"T_IFS08_{a}_ORM", "mask")}
    baked = build_baked(tex[atlases[0]])
    mats = {}
    for a in atlases:
        mats[f"MI_IFS08_{a}"] = instance(f"MI_IFS08_{a}", baked, textures={
            "BaseColor": tex[a]["bc"], "Normal": tex[a]["n"], "ORM": tex[a]["orm"]})
    fx = build_fx()
    screen_tex = import_texture(DASH, "T_IFS08_Dash", "color") if DASH and os.path.exists(DASH) else \
        tex[atlases[0]]["bc"]
    screen = build_screen(screen_tex)
    for m in MANIFEST["meshes"].values():
        for s in m["slots"]:
            if s in mats or not s.startswith("FX_"):
                continue
            if "DISPLAY_SCREEN" in s.upper():
                mats[s] = instance("MI_" + s, screen)
            else:
                mats[s] = instance("MI_" + s, fx, vectors={"Color": fx_colour(s)})
    log(f"materials: {sorted(mats)}")
    return mats


# --- meshes -----------------------------------------------------------------------

def assign(mesh, mats, static):
    slots = mesh.get_editor_property("static_materials" if static else "materials")
    for i, sm in enumerate(slots):
        name = str(sm.get_editor_property("material_slot_name"))
        mi = mats.get(name)
        if mi is None:
            log(f"WARNING {mesh.get_name()}: slot {name} has no material")
            continue
        if static:
            mesh.set_material(i, mi)
        else:
            sm.set_editor_property("material_interface", mi)
    if not static:
        mesh.set_editor_property("materials", slots)


def import_body(mats):
    for n in ("SK_IFS08", "SK_IFS08_Skeleton", "SK_IFS08_PhysicsAsset", "PA_IFS08"):
        fresh(f"{ROOT}/{n}")
    opts = unreal.FbxImportUI()
    opts.import_mesh = True
    opts.import_as_skeletal = True
    opts.mesh_type_to_import = unreal.FBXImportType.FBXIT_SKELETAL_MESH
    opts.import_materials = False
    opts.import_textures = False
    opts.import_animations = False
    opts.create_physics_asset = True
    sk = opts.skeletal_mesh_import_data
    sk.import_morph_targets = False
    sk.convert_scene = True
    sk.force_front_x_axis = False
    sk.use_t0_as_ref_pose = True
    sk.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
    mesh = import_task(os.path.join(SRC, "SK_IFS08.fbx"), ROOT, "SK_IFS08", opts)
    assign(mesh, mats, static=False)

    # Physics asset: one box on the root, the size of the chassis between the
    # wheels (floor to the top of the cockpit rim), centred on the body.
    pa = mesh.get_editor_property("physics_asset")
    if pa is None:
        raise RuntimeError("no physics asset was created for SK_IFS08")
    (lo, hi) = MANIFEST["meshes"]["SK_IFS08"]["bounds_cm"]
    wheels = MANIFEST["wheels"]
    inner_y = min(abs(w["hub_cm"][1]) - w["width_cm"] / 2 for w in wheels.values())
    box = unreal.KBoxElem()
    box.set_editor_property("center", unreal.Vector((lo[0] + hi[0]) / 2, 0.0, 30.0))
    box.set_editor_property("x", hi[0] - lo[0])
    box.set_editor_property("y", 2 * inner_y)
    box.set_editor_property("z", 50.0)
    bodies = pa.get_editor_property("skeletal_body_setups")
    keep = [b for b in bodies if str(b.get_editor_property("bone_name")) == "root"]
    if not keep:
        raise RuntimeError(f"physics asset has no root body: {[str(b.get_editor_property('bone_name')) for b in bodies]}")
    agg = keep[0].get_editor_property("agg_geom")
    agg.set_editor_property("box_elems", [box])
    agg.set_editor_property("sphyl_elems", [])
    agg.set_editor_property("sphere_elems", [])
    agg.set_editor_property("convex_elems", [])
    keep[0].set_editor_property("agg_geom", agg)
    if len(bodies) != 1:
        pa.set_editor_property("skeletal_body_setups", keep)
    EAL.save_loaded_asset(pa)
    log(f"physics asset {pa.get_path_name()}: {len(bodies)} bodies -> 1 box "
        f"{hi[0] - lo[0]:.0f} x {2 * inner_y:.0f} x 50 cm")

    # LODs from LOD0.
    sub = unreal.get_editor_subsystem(unreal.SkeletalMeshEditorSubsystem)
    if sub and sub.regenerate_lod(mesh, 4, False, False):
        log("SK_IFS08: 4 LODs generated")
    else:
        log("WARNING SK_IFS08: LODs not generated")
    EAL.save_loaded_asset(mesh)
    skel = mesh.get_editor_property("skeleton")
    pose = unreal.AnimPoseExtensions.get_reference_pose(skel)
    for k in CORNERS:
        t = unreal.AnimPoseExtensions.get_bone_pose(pose, "Wheel" + k, unreal.AnimPoseSpaces.WORLD)
        want = wheels[k]["hub_cm"]
        got = t.translation
        log(f"bone Wheel{k}: ({got.x:.1f}, {got.y:.1f}, {got.z:.1f}) cm, manifest "
            f"({want[0]:.1f}, {want[1]:.1f}, {want[2]:.1f})")
        if abs(got.x - want[0]) > 0.5 or abs(got.y - want[1]) > 0.5 or abs(got.z - want[2]) > 0.5:
            raise RuntimeError(f"bone Wheel{k} is not at its hub: axes or units went wrong in the FBX")
    box_b = mesh.get_bounds().box_extent
    log(f"SK_IFS08 bounds extent ({box_b.x:.0f}, {box_b.y:.0f}, {box_b.z:.0f}) cm")
    return mesh


def import_static(name, mats):
    path = os.path.join(SRC, name + ".fbx")
    if not os.path.exists(path):
        return None
    fresh(f"{ROOT}/{name}")
    opts = unreal.FbxImportUI()
    opts.import_mesh = True
    opts.import_as_skeletal = False
    opts.import_materials = False
    opts.import_textures = False
    sm = opts.static_mesh_import_data
    sm.combine_meshes = True
    sm.auto_generate_collision = False
    sm.generate_lightmap_u_vs = False
    sm.convert_scene = True
    sm.force_front_x_axis = False
    sm.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
    mesh = import_task(path, ROOT, name, opts)
    assign(mesh, mats, static=True)
    sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    lod = unreal.EditorScriptingMeshReductionOptions()
    lod.auto_compute_lod_screen_size = False
    lod.reduction_settings = [unreal.EditorScriptingMeshReductionSettings(1.0, 1.0),
                              unreal.EditorScriptingMeshReductionSettings(0.4, 0.3),
                              unreal.EditorScriptingMeshReductionSettings(0.12, 0.1)]
    sub.set_lods(mesh, lod)
    EAL.save_loaded_asset(mesh)
    b = mesh.get_bounding_box()
    log(f"{name}: {sub.get_lod_count(mesh)} LODs, bounds cm ({b.min.x:.1f},{b.min.y:.1f},{b.min.z:.1f})"
        f"..({b.max.x:.1f},{b.max.y:.1f},{b.max.z:.1f})")
    return mesh


def main():
    mats = build_materials()
    import_body(mats)
    for kind in ("Wheel", "Corner"):
        for k in CORNERS:
            import_static(f"SM_IFS08_{kind}_{k}", mats)
    log("done")


main()
