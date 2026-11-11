"""Import the world assets into the project (runs inside the UE editor).

    UnrealEditor-Cmd IFSSIM.uproject -run=pythonscript -EnablePlugins=PythonScriptPlugin \
        -script="tools/assetgen/ue_import.py" -unattended -nullrhi

Reads tools/assetgen/materials.json and the meshes tools/assetgen/blender
exported, and writes /Game/Worlds:

    Textures/<Set>/T_<Set>_<Map>   the CC0 scans (cache: ~/Library/Caches/ifssim-geo/ambientcg)
    Materials/M_WorldPBR           one master: tiling in metres, macro variation, recolour
    Materials/M_WorldPBRMasked     the same, two-sided with an opacity mask (fences)
    Materials/M_WorldPaint         road paint over the asphalt scan, worn through in places
    Materials/MI_<Name>            one instance per material in materials.json
    Meshes/SM_<Name>               the Blender assets, with generated LODs

Environment: IFSSIM_TEXTURE_CACHE overrides the texture cache, IFSSIM_ASSET_DIR
the folder of exported meshes (default Saved/AssetGen/fbx). Textures already
imported are kept (each import rewrites megabytes in LFS); IFSSIM_REIMPORT=1
imports them again. IFSSIM_MESHES=GarageBay,SeatBlock imports only those meshes.
Materials are always rebuilt.
"""

import json
import os

import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = json.load(open(os.path.join(HERE, "materials.json")))
CACHE = os.environ.get("IFSSIM_TEXTURE_CACHE",
                       os.path.expanduser("~/Library/Caches/ifssim-geo/ambientcg"))
PROJECT = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
ASSET_DIR = os.environ.get("IFSSIM_ASSET_DIR", os.path.join(PROJECT, "Saved", "AssetGen", "fbx"))
REIMPORT = os.environ.get("IFSSIM_REIMPORT") == "1"
ONLY_MESHES = {m for m in os.environ.get("IFSSIM_MESHES", "").split(",") if m}
GEN_DIR = os.path.join(PROJECT, "Saved", "AssetGen")

ROOT = "/Game/Worlds"
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()
EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary


def log(msg):
    unreal.log("ASSETGEN " + msg)


# --- textures -----------------------------------------------------------------

def import_file(path, dest, name):
    if not REIMPORT and EAL.does_asset_exist(f"{dest}/{name}"):
        return EAL.load_asset(f"{dest}/{name}")
    task = unreal.AssetImportTask()
    task.filename = path
    task.destination_path = dest
    task.destination_name = name
    task.automated = True
    task.replace_existing = True
    task.save = False
    TOOLS.import_asset_tasks([task])
    asset = EAL.load_asset(f"{dest}/{name}")
    if not asset:
        raise RuntimeError(f"import failed: {path}")
    return asset


def configure_texture(tex, kind):
    if kind == "normal":
        want = {"compression_settings": unreal.TextureCompressionSettings.TC_NORMALMAP, "srgb": False,
                "lod_group": unreal.TextureGroup.TEXTUREGROUP_WORLD_NORMAL_MAP}
    elif kind == "linear":
        want = {"compression_settings": unreal.TextureCompressionSettings.TC_DEFAULT, "srgb": False}
    else:
        want = {"srgb": True}
    # Only touch (and save) a texture whose settings differ: a kept texture stays byte-identical.
    if all(tex.get_editor_property(k) == v for k, v in want.items()):
        return
    for k, v in want.items():
        tex.set_editor_property(k, v)
    EAL.save_loaded_asset(tex)


KIND = {"Color": "color", "NormalDX": "normal", "Roughness": "linear",
        "AmbientOcclusion": "linear", "Metalness": "linear", "Opacity": "linear"}


def flat_textures():
    """Tiny flat maps for sets that lack AO or metalness, made here, not downloaded."""
    os.makedirs(GEN_DIR, exist_ok=True)
    out = {}
    for name, rgb, kind in (("T_White", (255, 255, 255), "color"), ("T_WhiteLinear", (255, 255, 255), "linear"),
                            ("T_BlackLinear", (0, 0, 0), "linear"), ("T_FlatNormal", (128, 128, 255), "normal")):
        path = os.path.join(GEN_DIR, name + ".bmp")
        write_bmp(path, 4, 4, rgb)
        tex = import_file(path, f"{ROOT}/Textures/Flat", name)
        configure_texture(tex, kind)
        out[name] = tex
    return out


def write_bmp(path, w, h, rgb):
    import struct
    row = bytes((rgb[2], rgb[1], rgb[0])) * w
    pad = (4 - len(row) % 4) % 4
    data = (row + b"\0" * pad) * h
    header = struct.pack("<2sIHHI", b"BM", 54 + len(data), 0, 0, 54)
    info = struct.pack("<IiiHHIIiiII", 40, w, h, 1, 24, 0, len(data), 2835, 2835, 0, 0)
    with open(path, "wb") as f:
        f.write(header + info + data)


# Sources are re-encoded before import: UE keeps the source image inside the
# .uasset, and the scans ship as near-lossless JPGs (a 4K colour map is 28 MB).
JPEG_QUALITY = 85
MASK_CAP = 1024  # roughness / AO / metalness: low-frequency, 1K is plenty


def prepare(path, cap):
    """Re-encode path as a JPEG at JPEG_QUALITY, at most cap pixels wide, in Saved/AssetGen."""
    import subprocess
    os.makedirs(GEN_DIR, exist_ok=True)
    out = os.path.join(GEN_DIR, f"q{JPEG_QUALITY}_{cap or 0}_" + os.path.basename(path))
    if not os.path.exists(out):
        cmd = ["sips", "-s", "format", "jpeg", "-s", "formatOptions", str(JPEG_QUALITY)]
        if cap:
            cmd += ["-Z", str(cap)]
        subprocess.run(cmd + [path, "--out", out], check=True, capture_output=True)
    return out


def import_textures():
    out = {}
    for set_name, s in SPEC["textures"].items():
        maps = {}
        for spec in s["maps"]:
            m, _, cap = spec.partition("@")
            path = os.path.join(CACHE, set_name, "maps", f"{set_name}_{s['res']}-JPG_{m}.jpg")
            if not os.path.exists(path):
                raise RuntimeError(f"missing {path}: download the set first (docs/world_assets.md)")
            dest, name = f"{ROOT}/Textures/{set_name}", f"T_{set_name}_{m}"
            if not REIMPORT and EAL.does_asset_exist(f"{dest}/{name}"):
                maps[m] = EAL.load_asset(f"{dest}/{name}")   # kept as imported, settings included
                continue
            cap = int(cap) if cap else (MASK_CAP if KIND[m] == "linear" else 0)
            path = prepare(path, cap)
            tex = import_file(path, dest, name)
            configure_texture(tex, KIND[m])
            maps[m] = tex
        out[set_name] = maps
        log(f"textures {set_name}: {', '.join(maps)}")
    return out


# --- master material ------------------------------------------------------------

def expr(mat, cls, x, y, **props):
    e = MEL.create_material_expression(mat, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def link(a, a_out, b, b_in):
    if not MEL.connect_material_expressions(a, a_out, b, b_in):
        raise RuntimeError(f"cannot connect {a.get_name()}.{a_out} -> {b.get_name()}.{b_in}")


def tex_param(mat, name, default, sampler, x, y):
    return expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, x, y,
                parameter_name=name, texture=default, sampler_type=sampler)


def build_master(name, flat, masked):
    path = f"{ROOT}/Materials/{name}"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mat = TOOLS.create_asset(name, f"{ROOT}/Materials", unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("used_with_instanced_static_meshes", True)
    if masked:
        mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
        mat.set_editor_property("two_sided", True)

    S = unreal.MaterialSamplerType
    # UV = TexCoord(metres) / tile_m
    tc = expr(mat, unreal.MaterialExpressionTextureCoordinate, -1400, 0)
    tiling = expr(mat, unreal.MaterialExpressionScalarParameter, -1400, 120, parameter_name="Tiling", default_value=0.5)
    uv = expr(mat, unreal.MaterialExpressionMultiply, -1200, 40)
    link(tc, "", uv, "A"); link(tiling, "", uv, "B")

    col = tex_param(mat, "BaseColorTex", flat["T_White"], S.SAMPLERTYPE_COLOR, -1000, -300)
    nrm = tex_param(mat, "NormalTex", flat["T_FlatNormal"], S.SAMPLERTYPE_NORMAL, -1000, 0)
    rgh = tex_param(mat, "RoughnessTex", flat["T_WhiteLinear"], S.SAMPLERTYPE_LINEAR_COLOR, -1000, 250)
    ao = tex_param(mat, "AOTex", flat["T_WhiteLinear"], S.SAMPLERTYPE_LINEAR_COLOR, -1000, 500)
    met = tex_param(mat, "MetalnessTex", flat["T_BlackLinear"], S.SAMPLERTYPE_LINEAR_COLOR, -1000, 750)
    for t in (col, nrm, rgh, ao, met):
        link(uv, "", t, "UVs")

    # Macro variation: the colour map again at 1/14 the frequency, as a
    # brightness wobble, so 1 km of asphalt doesn't show a 3 m repeat.
    macro_uv = expr(mat, unreal.MaterialExpressionMultiply, -1000, -550, const_b=0.071)
    link(uv, "", macro_uv, "A")
    macro = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -800, -550,
                 parameter_name="BaseColorTex", texture=flat["T_White"], sampler_type=S.SAMPLERTYPE_COLOR)
    link(macro_uv, "", macro, "UVs")
    macro_l = expr(mat, unreal.MaterialExpressionDesaturation, -600, -550)
    link(macro, "RGB", macro_l, "")
    var = expr(mat, unreal.MaterialExpressionScalarParameter, -600, -430, parameter_name="Variation", default_value=0.0)
    centred = expr(mat, unreal.MaterialExpressionSubtract, -450, -550, const_b=0.5)
    link(macro_l, "", centred, "A")
    wob = expr(mat, unreal.MaterialExpressionMultiply, -300, -550)
    link(centred, "", wob, "A"); link(var, "", wob, "B")
    gain = expr(mat, unreal.MaterialExpressionAdd, -150, -550, const_a=1.0)
    link(wob, "", gain, "B")

    # Recolour: lerp(scan * tint, luminance(scan) * tint * 2, recolor)
    tint = expr(mat, unreal.MaterialExpressionVectorParameter, -800, -200, parameter_name="Tint",
                default_value=unreal.LinearColor(1, 1, 1, 1))
    recolor = expr(mat, unreal.MaterialExpressionScalarParameter, -800, -80, parameter_name="Recolor", default_value=0.0)
    tinted = expr(mat, unreal.MaterialExpressionMultiply, -600, -300)
    link(col, "RGB", tinted, "A"); link(tint, "", tinted, "B")
    lum = expr(mat, unreal.MaterialExpressionDesaturation, -600, -180)
    link(col, "RGB", lum, "")
    lum2 = expr(mat, unreal.MaterialExpressionMultiply, -450, -180, const_b=2.0)
    link(lum, "", lum2, "A")
    lumt = expr(mat, unreal.MaterialExpressionMultiply, -300, -180)
    link(lum2, "", lumt, "A"); link(tint, "", lumt, "B")
    mix = expr(mat, unreal.MaterialExpressionLinearInterpolate, -150, -250)
    link(tinted, "", mix, "A"); link(lumt, "", mix, "B"); link(recolor, "", mix, "Alpha")
    base = expr(mat, unreal.MaterialExpressionMultiply, 50, -300)
    link(mix, "", base, "A"); link(gain, "", base, "B")

    # Wear (the track's rubber): the colour times twice the vertex red, which
    # the world actor fills from the wear map; WearAmount 0 ignores it.
    vc = expr(mat, unreal.MaterialExpressionVertexColor, -150, -800)
    vc2 = expr(mat, unreal.MaterialExpressionMultiply, 0, -800, const_b=2.0)
    link(vc, "R", vc2, "A")
    wear_amt = expr(mat, unreal.MaterialExpressionScalarParameter, 0, -700, parameter_name="WearAmount", default_value=0.0)
    wear = expr(mat, unreal.MaterialExpressionLinearInterpolate, 150, -800, const_a=1.0)
    link(vc2, "", wear, "B"); link(wear_amt, "", wear, "Alpha")
    worn = expr(mat, unreal.MaterialExpressionMultiply, 250, -300)
    link(base, "", worn, "A"); link(wear, "", worn, "B")
    MEL.connect_material_property(worn, "", unreal.MaterialProperty.MP_BASE_COLOR)

    MEL.connect_material_property(nrm, "RGB", unreal.MaterialProperty.MP_NORMAL)

    rs = expr(mat, unreal.MaterialExpressionScalarParameter, -800, 330, parameter_name="RoughnessScale", default_value=1.0)
    r = expr(mat, unreal.MaterialExpressionMultiply, -600, 250)
    link(rgh, "R", r, "A"); link(rs, "", r, "B")
    # Rubbered asphalt is a little smoother, the unused edges a little rougher.
    gloss = expr(mat, unreal.MaterialExpressionScalarParameter, -450, 400, parameter_name="WearGloss", default_value=0.5)
    rwear = expr(mat, unreal.MaterialExpressionLinearInterpolate, -300, 330, const_a=1.0)
    link(wear, "", rwear, "B"); link(gloss, "", rwear, "Alpha")
    r2 = expr(mat, unreal.MaterialExpressionMultiply, -150, 250)
    link(r, "", r2, "A"); link(rwear, "", r2, "B")
    MEL.connect_material_property(r2, "", unreal.MaterialProperty.MP_ROUGHNESS)
    MEL.connect_material_property(ao, "R", unreal.MaterialProperty.MP_AMBIENT_OCCLUSION)
    ms = expr(mat, unreal.MaterialExpressionScalarParameter, -800, 830, parameter_name="MetallicScale", default_value=1.0)
    m = expr(mat, unreal.MaterialExpressionMultiply, -600, 750)
    link(met, "R", m, "A"); link(ms, "", m, "B")
    MEL.connect_material_property(m, "", unreal.MaterialProperty.MP_METALLIC)

    em = expr(mat, unreal.MaterialExpressionScalarParameter, -150, -420, parameter_name="Emissive", default_value=0.0)
    emc = expr(mat, unreal.MaterialExpressionMultiply, 50, -450)
    link(base, "", emc, "A"); link(em, "", emc, "B")
    MEL.connect_material_property(emc, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

    if masked:
        op = tex_param(mat, "OpacityTex", flat["T_WhiteLinear"], S.SAMPLERTYPE_LINEAR_COLOR, -1000, 1000)
        link(uv, "", op, "UVs")
        MEL.connect_material_property(op, "R", unreal.MaterialProperty.MP_OPACITY_MASK)

    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log(f"master {name}")
    return mat


def make_instance(name, parent, tex=None, tile_m=1.0, tint=(1, 1, 1), recolor=0.0, variation=0.0,
                  roughness=1.0, metallic=1.0, emissive=0.0, extra_tex=None, wear=0.0):
    path = f"{ROOT}/Materials/MI_{name}"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mi = TOOLS.create_asset(f"MI_{name}", f"{ROOT}/Materials", unreal.MaterialInstanceConstant,
                            unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(mi, parent)
    slots = {"Color": "BaseColorTex", "NormalDX": "NormalTex", "Roughness": "RoughnessTex",
             "AmbientOcclusion": "AOTex", "Metalness": "MetalnessTex", "Opacity": "OpacityTex"}
    for m, t in (tex or {}).items():
        MEL.set_material_instance_texture_parameter_value(mi, slots[m], t)
    for m, t in (extra_tex or {}).items():
        MEL.set_material_instance_texture_parameter_value(mi, m, t)
    MEL.set_material_instance_scalar_parameter_value(mi, "Tiling", 1.0 / tile_m)
    MEL.set_material_instance_scalar_parameter_value(mi, "Recolor", recolor)
    MEL.set_material_instance_scalar_parameter_value(mi, "Variation", variation)
    MEL.set_material_instance_scalar_parameter_value(mi, "RoughnessScale", roughness)
    MEL.set_material_instance_scalar_parameter_value(mi, "MetallicScale", metallic)
    MEL.set_material_instance_scalar_parameter_value(mi, "Emissive", emissive)
    MEL.set_material_instance_scalar_parameter_value(mi, "WearAmount", wear)
    MEL.set_material_instance_vector_parameter_value(mi, "Tint", unreal.LinearColor(*tint, 1.0))
    MEL.update_material_instance(mi)
    EAL.save_loaded_asset(mi)
    return mi


# The asphalt scan's luminance (UE desaturation weights, linear): mean and
# spread, measured from Asphalt031's colour map. The paint mask is built in
# units of that spread, so `coverage` means the same whatever the scan.
GRAIN_MEAN, GRAIN_STD = 0.197, 0.077


def build_paint_master(flat):
    """Road paint: the asphalt scan under a layer of paint covering `Coverage`
    of it. The stones' tops (bright grain) and larger worn patches (the same
    scan at 1/16 the frequency) break through; the paint picks up the grain's
    shading and half its relief."""
    name = "M_WorldPaint"
    path = f"{ROOT}/Materials/{name}"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mat = TOOLS.create_asset(name, f"{ROOT}/Materials", unreal.Material, unreal.MaterialFactoryNew())
    S = unreal.MaterialSamplerType
    tc = expr(mat, unreal.MaterialExpressionTextureCoordinate, -1600, 0)
    tiling = expr(mat, unreal.MaterialExpressionScalarParameter, -1600, 120, parameter_name="Tiling", default_value=0.333)
    uv = expr(mat, unreal.MaterialExpressionMultiply, -1400, 40)
    link(tc, "", uv, "A"); link(tiling, "", uv, "B")
    col = tex_param(mat, "BaseColorTex", flat["T_White"], S.SAMPLERTYPE_COLOR, -1200, -400)
    nrm = tex_param(mat, "NormalTex", flat["T_FlatNormal"], S.SAMPLERTYPE_NORMAL, -1200, 0)
    rgh = tex_param(mat, "RoughnessTex", flat["T_WhiteLinear"], S.SAMPLERTYPE_LINEAR_COLOR, -1200, 300)
    for t in (col, nrm, rgh):
        link(uv, "", t, "UVs")
    puv = expr(mat, unreal.MaterialExpressionMultiply, -1200, -700, const_b=0.061)
    link(uv, "", puv, "A")
    patch = expr(mat, unreal.MaterialExpressionTextureSampleParameter2D, -1000, -700,
                 parameter_name="BaseColorTex", texture=flat["T_White"], sampler_type=S.SAMPLERTYPE_COLOR)
    link(puv, "", patch, "UVs")

    def z_score(src, x, y, weight):
        lum = expr(mat, unreal.MaterialExpressionDesaturation, x, y)
        link(src, "RGB", lum, "")
        c = expr(mat, unreal.MaterialExpressionSubtract, x + 150, y, const_b=GRAIN_MEAN)
        link(lum, "", c, "A")
        z = expr(mat, unreal.MaterialExpressionMultiply, x + 300, y, const_b=weight / GRAIN_STD)
        link(c, "", z, "A")
        return lum, z

    grain, zg = z_score(col, -900, -400, 0.45)
    _, zp = z_score(patch, -900, -700, 0.6)
    cov = expr(mat, unreal.MaterialExpressionScalarParameter, -900, -900, parameter_name="Coverage", default_value=0.85)
    bias = expr(mat, unreal.MaterialExpressionSubtract, -750, -900, const_b=0.5)
    link(cov, "", bias, "A")
    bias4 = expr(mat, unreal.MaterialExpressionMultiply, -600, -900, const_b=4.0)
    link(bias, "", bias4, "A")
    s1 = expr(mat, unreal.MaterialExpressionSubtract, -450, -800)
    link(bias4, "", s1, "A"); link(zg, "", s1, "B")
    s2 = expr(mat, unreal.MaterialExpressionSubtract, -300, -800)
    link(s1, "", s2, "A"); link(zp, "", s2, "B")
    s3 = expr(mat, unreal.MaterialExpressionMultiply, -150, -800, const_b=1.2)
    link(s2, "", s3, "A")
    s4 = expr(mat, unreal.MaterialExpressionAdd, 0, -800, const_b=0.5)
    link(s3, "", s4, "A")
    m = expr(mat, unreal.MaterialExpressionSaturate, 150, -800)
    link(s4, "", m, "")

    tint = expr(mat, unreal.MaterialExpressionVectorParameter, -600, -300, parameter_name="Tint",
                default_value=unreal.LinearColor(1, 1, 1, 1))
    road = expr(mat, unreal.MaterialExpressionMultiply, -400, -400)
    link(col, "RGB", road, "A"); link(tint, "", road, "B")
    paint = expr(mat, unreal.MaterialExpressionVectorParameter, -600, -150, parameter_name="PaintColor",
                 default_value=unreal.LinearColor(0.78, 0.78, 0.76, 1))
    shade = expr(mat, unreal.MaterialExpressionMultiply, -600, -50, const_b=0.75)
    link(grain, "", shade, "A")
    shade2 = expr(mat, unreal.MaterialExpressionAdd, -450, -50, const_b=0.85)
    link(shade, "", shade2, "A")
    painted = expr(mat, unreal.MaterialExpressionMultiply, -300, -150)
    link(paint, "", painted, "A"); link(shade2, "", painted, "B")
    base = expr(mat, unreal.MaterialExpressionLinearInterpolate, 300, -350)
    link(road, "", base, "A"); link(painted, "", base, "B"); link(m, "", base, "Alpha")
    MEL.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)

    pr = expr(mat, unreal.MaterialExpressionScalarParameter, -600, 450, parameter_name="PaintRoughness", default_value=0.55)
    rough = expr(mat, unreal.MaterialExpressionLinearInterpolate, 300, 300)
    link(rgh, "R", rough, "A"); link(pr, "", rough, "B"); link(m, "", rough, "Alpha")
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

    up = expr(mat, unreal.MaterialExpressionConstant3Vector, -600, 100, constant=unreal.LinearColor(0, 0, 1, 1))
    half = expr(mat, unreal.MaterialExpressionMultiply, 0, 150, const_b=0.6)
    link(m, "", half, "A")
    n = expr(mat, unreal.MaterialExpressionLinearInterpolate, 300, 50)
    link(nrm, "RGB", n, "A"); link(up, "", n, "B"); link(half, "", n, "Alpha")
    MEL.connect_material_property(n, "", unreal.MaterialProperty.MP_NORMAL)

    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    log(f"master {name}")
    return mat


def make_paint(name, parent, tex, spec):
    path = f"{ROOT}/Materials/MI_{name}"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mi = TOOLS.create_asset(f"MI_{name}", f"{ROOT}/Materials", unreal.MaterialInstanceConstant,
                            unreal.MaterialInstanceConstantFactoryNew())
    MEL.set_material_instance_parent(mi, parent)
    for m, slot in (("Color", "BaseColorTex"), ("NormalDX", "NormalTex"), ("Roughness", "RoughnessTex")):
        MEL.set_material_instance_texture_parameter_value(mi, slot, tex[m])
    MEL.set_material_instance_scalar_parameter_value(mi, "Tiling", 1.0 / spec["tile_m"])
    MEL.set_material_instance_scalar_parameter_value(mi, "Coverage", spec["coverage"])
    MEL.set_material_instance_scalar_parameter_value(mi, "PaintRoughness", spec["roughness"])
    MEL.set_material_instance_vector_parameter_value(mi, "Tint", unreal.LinearColor(*spec["tint"], 1.0))
    MEL.set_material_instance_vector_parameter_value(mi, "PaintColor", unreal.LinearColor(*spec["color"], 1.0))
    MEL.update_material_instance(mi)
    EAL.save_loaded_asset(mi)
    return mi


def build_materials(textures, flat):
    master = build_master("M_WorldPBR", flat, masked=False)
    masked = build_master("M_WorldPBRMasked", flat, masked=True)
    for name, m in SPEC["materials"].items():
        make_instance(name, master, textures[m["set"]], m["tile_m"], m["tint"], m["recolor"], m["variation"],
                      roughness=m.get("roughness", 1.0), wear=m.get("wear", 0.0))
    paint = build_paint_master(flat)
    for name, m in SPEC.get("paint", {}).items():
        if not name.startswith("_"):
            make_paint(name, paint, textures[m["set"]], m)
    for name, m in SPEC["flat"].items():
        make_instance(name, master, None, 1.0, m["color"], 0.0, 0.0,
                      roughness=m["roughness"], metallic=m["metallic"], emissive=m.get("emissive", 0.0),
                      extra_tex={"MetalnessTex": flat["T_WhiteLinear"]})
    # Chain-link: the galvanised scan, cut by the opacity mask Blender drew.
    mask = os.path.join(GEN_DIR, "chainlink_opacity.png")
    if os.path.exists(mask):
        op = import_file(mask, f"{ROOT}/Textures/Generated", "T_ChainLink_Opacity")
        configure_texture(op, "linear")
        make_instance("ChainLink", masked, textures["CorrugatedSteel005"], 0.1, (1.1, 1.1, 1.1), 0.0, 0.0,
                      extra_tex={"OpacityTex": op})
    build_foliage(flat)
    n_paint = sum(not k.startswith("_") for k in SPEC.get("paint", {}))
    log(f"materials: {len(SPEC['materials']) + len(SPEC['flat']) + n_paint} instances")


def build_foliage(flat):
    """Leaf cards: masked, two-sided foliage shading (light comes through the
    leaves), card UVs 0..1, and a colour per tree between Tint and TintVary."""
    if not SPEC.get("foliage"):
        return
    name = "M_Foliage"
    path = f"{ROOT}/Materials/{name}"
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    mat = TOOLS.create_asset(name, f"{ROOT}/Materials", unreal.Material, unreal.MaterialFactoryNew())
    mat.set_editor_property("used_with_instanced_static_meshes", True)
    mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
    mat.set_editor_property("two_sided", True)
    mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE)
    mat.set_editor_property("opacity_mask_clip_value", 0.4)
    S = unreal.MaterialSamplerType
    white, white_lin = flat["T_White"], flat["T_WhiteLinear"]
    col = tex_param(mat, "BaseColorTex", white, S.SAMPLERTYPE_COLOR, -900, -200)
    op = tex_param(mat, "OpacityTex", white_lin, S.SAMPLERTYPE_LINEAR_COLOR, -900, 300)
    tint = expr(mat, unreal.MaterialExpressionVectorParameter, -900, -450, parameter_name="Tint",
                default_value=unreal.LinearColor(0.07, 0.1, 0.04, 1))
    vary = expr(mat, unreal.MaterialExpressionVectorParameter, -900, -600, parameter_name="TintVary",
                default_value=unreal.LinearColor(0.1, 0.12, 0.045, 1))
    rnd = expr(mat, unreal.MaterialExpressionPerInstanceRandom, -700, -650)
    pick = expr(mat, unreal.MaterialExpressionLinearInterpolate, -500, -500)
    link(tint, "", pick, "A"); link(vary, "", pick, "B"); link(rnd, "", pick, "Alpha")
    # The drawn leaves average about 0.55 grey: x1.8 makes Tint the mean albedo.
    detail = expr(mat, unreal.MaterialExpressionMultiply, -500, -250, const_b=1.8)
    link(col, "RGB", detail, "A")
    base = expr(mat, unreal.MaterialExpressionMultiply, -300, -350)
    link(detail, "", base, "A"); link(pick, "", base, "B")
    MEL.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)
    tr = expr(mat, unreal.MaterialExpressionScalarParameter, -500, -50, parameter_name="Translucency", default_value=0.6)
    sss = expr(mat, unreal.MaterialExpressionMultiply, -300, -100)
    link(base, "", sss, "A"); link(tr, "", sss, "B")
    MEL.connect_material_property(sss, "", unreal.MaterialProperty.MP_SUBSURFACE_COLOR)
    rough = expr(mat, unreal.MaterialExpressionConstant, -300, 100, r=0.7)
    MEL.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    spec = expr(mat, unreal.MaterialExpressionConstant, -300, 180, r=0.3)
    MEL.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)
    MEL.connect_material_property(op, "R", unreal.MaterialProperty.MP_OPACITY_MASK)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)

    for slot, f in SPEC["foliage"].items():
        maps = {}
        for tag, kind in (("color", "color"), ("opacity", "linear")):
            src = os.path.join(GEN_DIR, f"foliage_{slot}_{tag}.png")
            if not os.path.exists(src):
                raise RuntimeError(f"{src} missing: run tools/assetgen/blender first")
            t = import_file(src, f"{ROOT}/Textures/Generated", f"T_{slot}_{tag.capitalize()}")
            configure_texture(t, kind)
            maps[tag] = t
        path = f"{ROOT}/Materials/MI_{slot}"
        if EAL.does_asset_exist(path):
            EAL.delete_asset(path)
        mi = TOOLS.create_asset(f"MI_{slot}", f"{ROOT}/Materials", unreal.MaterialInstanceConstant,
                                unreal.MaterialInstanceConstantFactoryNew())
        MEL.set_material_instance_parent(mi, mat)
        MEL.set_material_instance_texture_parameter_value(mi, "BaseColorTex", maps["color"])
        MEL.set_material_instance_texture_parameter_value(mi, "OpacityTex", maps["opacity"])
        MEL.set_material_instance_vector_parameter_value(mi, "Tint", unreal.LinearColor(*f["tint"], 1.0))
        MEL.set_material_instance_vector_parameter_value(mi, "TintVary", unreal.LinearColor(*f["tint_vary"], 1.0))
        MEL.set_material_instance_scalar_parameter_value(mi, "Translucency", f["translucency"])
        MEL.update_material_instance(mi)
        EAL.save_loaded_asset(mi)
    log(f"foliage: {len(SPEC['foliage'])} instances")


# --- meshes ---------------------------------------------------------------------

def mesh_subsystem():
    """StaticMeshEditorSubsystem; a commandlet only creates it once its module is loaded."""
    sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    if sub is None:
        unreal.load_module("StaticMeshEditor")
        sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    if sub is None:
        raise RuntimeError("StaticMeshEditorSubsystem unavailable: LODs can't be built")
    return sub


def import_meshes():
    manifest = os.path.join(ASSET_DIR, "manifest.json")
    if not os.path.exists(manifest):
        log("no meshes exported yet (run tools/assetgen/blender first)")
        return
    for a in json.load(open(manifest)):
        if ONLY_MESHES and a["name"] not in ONLY_MESHES:
            continue
        task = unreal.AssetImportTask()
        task.filename = os.path.join(ASSET_DIR, a["file"])
        task.destination_path = f"{ROOT}/Meshes"
        task.destination_name = f"SM_{a['name']}"
        task.automated = True
        task.replace_existing = True
        task.save = False
        opts = unreal.FbxImportUI()
        opts.import_mesh = True
        opts.import_materials = False
        opts.import_textures = False
        opts.import_as_skeletal = False
        sm = opts.static_mesh_import_data
        sm.combine_meshes = True
        sm.auto_generate_collision = False
        sm.generate_lightmap_u_vs = False
        sm.convert_scene = True
        sm.force_front_x_axis = False
        task.options = opts
        TOOLS.import_asset_tasks([task])
        mesh = EAL.load_asset(f"{ROOT}/Meshes/SM_{a['name']}")
        if not mesh:
            raise RuntimeError(f"mesh import failed: {a['file']}")
        # Materials by slot name, as Blender named them.
        mats = mesh.get_editor_property("static_materials")
        for i, sm_mat in enumerate(mats):
            slot = str(sm_mat.get_editor_property("material_slot_name"))
            mi = EAL.load_asset(f"{ROOT}/Materials/MI_{slot}")
            if mi:
                mesh.set_material(i, mi)
            else:
                log(f"WARNING {a['name']}: no material MI_{slot}")
        # LODs: [triangle fraction, screen size] pairs after LOD0, from the manifest.
        lods = a.get("lods") or []
        subsystem = mesh_subsystem()
        if lods:
            opts_lod = unreal.EditorScriptingMeshReductionOptions()
            opts_lod.auto_compute_lod_screen_size = False
            opts_lod.reduction_settings = [unreal.EditorScriptingMeshReductionSettings(1.0, 1.0)] + [
                unreal.EditorScriptingMeshReductionSettings(frac, screen) for frac, screen in lods]
            subsystem.set_lods(mesh, opts_lod)
        # Collision: none for detail (the LiDAR renders, it doesn't trace);
        # simple boxes where the car could reach.
        if a.get("collision") == "box":
            subsystem.add_simple_collisions(mesh, unreal.ScriptCollisionShapeType.BOX)
        EAL.save_loaded_asset(mesh)
        box = mesh.get_bounding_box()
        lo, hi = box.min / 100, box.max / 100
        log(f"mesh {a['name']}: {len(mats)} material(s), {subsystem.get_lod_count(mesh)} LODs, "
            f"bounds m ({lo.x:.2f},{lo.y:.2f},{lo.z:.2f})..({hi.x:.2f},{hi.y:.2f},{hi.z:.2f})")


def main():
    flat = flat_textures()
    textures = import_textures()
    build_materials(textures, flat)
    import_meshes()
    log("done")


main()
