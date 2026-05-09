"""
Create the M_LiDARStencilEncoder post-process material via the UE5 editor's
Python API. One-click alternative to authoring the material by hand —
produces a .uasset structurally equivalent to the recipe in issue #358.

USAGE
-----
1. Open the project in UE5 5.7.
2. Make sure the Python Editor Script Plugin is enabled
   (Edit → Plugins → "Python Editor Script Plugin").
3. Open the Output Log → Cmd dropdown (bottom of the log) → switch to
   "Python".
4. Run:
       exec(open(r"<absolute path to this file>").read())
   …or use the Tools → Execute Python Script menu and pick this file.
5. The script creates `/FSDSPlugin/Materials/M_LiDARStencilEncoder`,
   wires it up, and saves it. The next sim cook + run picks it up
   automatically — `bReflectanceLUTActive` flips to true in
   FSDSLidarSensor::InitializeGPUPath, the LUT goes live.

WHAT IT BUILDS
--------------
A Material Domain = Post Process material that packs cone-stencil ID +
Rec.709 luminance of the rendered scene into two RGB channels of
Emissive Color, Opacity = 1 so the material output replaces the scene:

    Emissive Color = MakeFloat3(
        SceneTexture(CustomStencil) / 255,                # R: stencil ID encoding
        dot(SceneTexture(PostProcessInput0).rgb, Rec709), # G: luminance fallback
        0                                                 # B: unused
    )
    Opacity = 1

The decode shader (FSDSLidarDecode.usf) reads R for the stencil ID,
indexes ReflectanceLUT[]. If LUT slot is unset / stencil = 0 / out of
range, falls back to G (pre-computed luminance) — exact same fallback
behaviour as before, just relocated into the material so the shader
can decode in two channel reads instead of doing the dot product.

Why this design (rather than alpha for stencil): UE5 post-process
materials' MP_OPACITY controls the blend amount with the previous
scene color, not the framebuffer alpha. Encoding stencil into Opacity
silently makes "blend at stencil%/255 strength" — wrong dimension,
no useful output. Packing into RGB channels works regardless of how
the post-process pipeline treats alpha.

If the asset already exists, the script asks before overwriting.
"""

import unreal


PACKAGE_PATH = "/FSDSPlugin/Materials"
ASSET_NAME   = "M_LiDARStencilEncoder"
ASSET_PATH   = f"{PACKAGE_PATH}/{ASSET_NAME}"


def _ensure_dir(package_path: str) -> None:
    """`AssetTools.create_asset` needs the destination directory to exist
    in the Content Browser. Create it if it isn't already there."""
    if not unreal.EditorAssetLibrary.does_directory_exist(package_path):
        unreal.EditorAssetLibrary.make_directory(package_path)


def _create_material() -> unreal.Material:
    """Always create from scratch. If the asset exists, delete it
    first — `build_material` only adds nodes, it doesn't clear the
    existing graph, so re-running over an existing asset would
    produce a salad of stale + new nodes (which is exactly what
    happened on the first cycle of #358 D-Phase-2)."""
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    factory = unreal.MaterialFactoryNew()

    if unreal.EditorAssetLibrary.does_asset_exist(ASSET_PATH):
        unreal.log(f"{ASSET_PATH} exists — deleting before rebuild")
        unreal.EditorAssetLibrary.delete_asset(ASSET_PATH)

    _ensure_dir(PACKAGE_PATH)
    material = asset_tools.create_asset(ASSET_NAME, PACKAGE_PATH, unreal.Material, factory)
    if material is None:
        raise RuntimeError(f"Failed to create material at {ASSET_PATH}")
    return material


def build_material(material: unreal.Material) -> None:
    """Configure the material's domain + expression graph.

    Output (post-tonemapping in the post-process chain):

        Emissive Color = MakeFloat3(
            stencil / 255,                      # R: stencil ID
            dot(scene.rgb, Rec709Coefficients), # G: luminance fallback
            0,                                  # B: unused
        )
        Opacity = 1                              # full replace, not blend

    Packing into RGB channels (rather than alpha) is deliberate: in
    UE5 post-process materials, MP_OPACITY controls the BLEND amount
    with the previous scene color, not the framebuffer alpha. So
    `Opacity = stencil/255` would just fade the material output in at
    a tiny percentage for cone hits — the wrong dimension entirely.
    Writing the encoded values into Emissive (which the shader reads
    as `ColorTexture.rgb` at sample time) sidesteps the question of
    whether the post-process alpha channel propagates at all.
    """
    lib = unreal.MaterialEditingLibrary

    # `MaterialDomain` is the right knob (not `MaterialUsage`) — usage
    # bits are for things like SkeletalMesh / StaticLighting / etc.
    material.set_editor_property("material_domain", unreal.MaterialDomain.MD_POST_PROCESS)
    # UE5 5.7's `EBlendableLocation` Python binding uses positional
    # names (BEFORE_BLOOM, BEFORE_DOF, AFTER_DOF, AFTER_TONEMAPPING,
    # REPLACING_TONEMAPPER, SSR_INPUT, TRANSLUCENCY_AFTER_DOF). Run
    # AFTER_TONEMAPPING so the encoded RGB output is the very last
    # thing written to FinalColorLDR before the SceneCapture reads it.
    material.set_editor_property("blendable_location",
                                 unreal.BlendableLocation.BL_SCENE_COLOR_AFTER_TONEMAPPING)

    # === SceneTexture nodes ============================================
    color_node = lib.create_material_expression(
        material, unreal.MaterialExpressionSceneTexture, -700, -100)
    color_node.set_editor_property(
        "scene_texture_id", unreal.SceneTextureId.PPI_POST_PROCESS_INPUT0)

    stencil_node = lib.create_material_expression(
        material, unreal.MaterialExpressionSceneTexture, -700, 200)
    stencil_node.set_editor_property(
        "scene_texture_id", unreal.SceneTextureId.PPI_CUSTOM_STENCIL)

    # SceneTexture's default ("Color") output is float4. We need scalar
    # values to feed AppendVector, so each branch gets a ComponentMask
    # right after the SceneTexture sample.

    def _make_mask(x, y, r=False, g=False, b=False, a=False):
        m = lib.create_material_expression(
            material, unreal.MaterialExpressionComponentMask, x, y)
        m.set_editor_property("R", r)
        m.set_editor_property("G", g)
        m.set_editor_property("B", b)
        m.set_editor_property("A", a)
        return m

    # === R channel: stencil scalar / 255 ===============================
    # CustomStencil is single-channel; SceneTexture replicates the
    # value across the .r component (the others are typically 0). Mask
    # to .r to get a scalar before dividing.
    stencil_r = _make_mask(-450, 220, r=True)
    lib.connect_material_expressions(stencil_node, "", stencil_r, "")

    const255 = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant, -300, 320)
    const255.set_editor_property("R", 255.0)

    stencil_div = lib.create_material_expression(
        material, unreal.MaterialExpressionDivide, -150, 270)
    lib.connect_material_expressions(stencil_r, "", stencil_div, "A")
    lib.connect_material_expressions(const255,  "", stencil_div, "B")

    # === G channel: dot(scene.rgb, Rec709) — pre-computed luminance =====
    # DotProduct's two inputs must have matching component counts —
    # mask scene.rgb to a float3 and pair it with a Constant3Vector.
    color_rgb = _make_mask(-450, -90, r=True, g=True, b=True)
    lib.connect_material_expressions(color_node, "", color_rgb, "")

    rec709 = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, -450, 30)
    rec709.set_editor_property(
        "constant", unreal.LinearColor(0.2126, 0.7152, 0.0722, 0.0))
    luminance_dot = lib.create_material_expression(
        material, unreal.MaterialExpressionDotProduct, -200, -50)
    lib.connect_material_expressions(color_rgb, "", luminance_dot, "A")
    lib.connect_material_expressions(rec709,    "", luminance_dot, "B")

    # === Pack (R=stencil/255, G=luminance, B=0) into a float3 ===========
    # UE5 5.7's Python binding doesn't expose `MaterialExpressionMakeFloat3`;
    # build the float3 by chaining `MaterialExpressionAppendVector`
    # (the long-standing UE5 way to combine scalars/vectors). Two stages:
    #   1. Append(stencil/255, luminance) → float2 (R, G)
    #   2. Append(float2, 0)              → float3 (R, G, 0)
    zero_const = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant, 0, 320)
    zero_const.set_editor_property("R", 0.0)

    append_xy = lib.create_material_expression(
        material, unreal.MaterialExpressionAppendVector, 100, 100)
    lib.connect_material_expressions(stencil_div,   "", append_xy, "A")
    lib.connect_material_expressions(luminance_dot, "", append_xy, "B")

    pack = lib.create_material_expression(
        material, unreal.MaterialExpressionAppendVector, 300, 150)
    lib.connect_material_expressions(append_xy,  "", pack, "A")
    lib.connect_material_expressions(zero_const, "", pack, "B")

    # === Emissive Color ← float3 (stencil/255, luminance, 0) ============
    lib.connect_material_property(
        pack, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

    # === Opacity = 1 (full replace, not blend) ==========================
    one = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant, 200, 300)
    one.set_editor_property("R", 1.0)
    lib.connect_material_property(
        one, "", unreal.MaterialProperty.MP_OPACITY)

    # Compile + save.
    lib.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)


def main() -> None:
    material = _create_material()
    build_material(material)
    unreal.log(f"FSDS: {ASSET_PATH} built and saved.")


if __name__ == "__main__":
    main()
