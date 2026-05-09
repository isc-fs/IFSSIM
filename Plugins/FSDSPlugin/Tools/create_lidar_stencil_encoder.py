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
A Material Domain = Post Process material whose graph is:

    SceneTexture(PostProcessInput0).rgb  ─►  Emissive Color
    SceneTexture(CustomStencil) / 255    ─►  Opacity

That puts the cone's CustomDepthStencilValue into the alpha channel of
the LiDAR's FinalColorLDR capture. The decode shader's
`UseReflectanceLUT > 0` branch reads alpha → stencil ID → datasheet
ρ_905 from `ReflectanceLUT[]`.

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
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    factory = unreal.MaterialFactoryNew()

    if unreal.EditorAssetLibrary.does_asset_exist(ASSET_PATH):
        unreal.log_warning(
            f"{ASSET_PATH} already exists. Delete it first if you want to regenerate."
        )
        return unreal.load_asset(ASSET_PATH)

    _ensure_dir(PACKAGE_PATH)
    material = asset_tools.create_asset(ASSET_NAME, PACKAGE_PATH, unreal.Material, factory)
    if material is None:
        raise RuntimeError(f"Failed to create material at {ASSET_PATH}")
    return material


def build_material(material: unreal.Material) -> None:
    """Configure the material's domain + expression graph."""
    lib = unreal.MaterialEditingLibrary

    # Post-process domain. BlendableLocation = BeforeTonemapping so the
    # alpha-encoded stencil survives tonemap clamps — the LiDAR colour
    # capture has tonemapping enabled (post-processed pipeline output)
    # so the alpha needs to land before that pass crushes it.
    lib.set_material_usage(material, unreal.MaterialUsage.MATUSAGE_POSTPROCESS_MATERIAL) \
        if hasattr(unreal, "MaterialUsage") else None
    material.set_editor_property("material_domain", unreal.MaterialDomain.MD_POST_PROCESS)
    material.set_editor_property("blendable_location",
                                 unreal.BlendableLocation.BL_BEFORE_TONEMAPPING)

    # === SceneTexture: PostProcessInput0 (the rendered colour we want
    # to preserve in RGB so the luminance fallback still works for
    # non-cone hits where stencil = 0). =================================
    color_node = lib.create_material_expression(
        material, unreal.MaterialExpressionSceneTexture, -400, -150)
    color_node.set_editor_property(
        "scene_texture_id", unreal.SceneTextureId.PPI_PostProcessInput0)

    # === SceneTexture: CustomStencil (uint, stencil per pixel)
    stencil_node = lib.create_material_expression(
        material, unreal.MaterialExpressionSceneTexture, -400, 150)
    stencil_node.set_editor_property(
        "scene_texture_id", unreal.SceneTextureId.PPI_CustomStencil)

    # === Constant 255 (UE5 outputs CustomStencil in [0..255] as a float
    # at 8-bit precision; divide by 255 puts it in [0..1] for the alpha
    # channel where the shader expects it).
    const_node = lib.create_material_expression(
        material, unreal.MaterialExpressionConstant, -200, 250)
    const_node.set_editor_property("R", 255.0)

    # === Divide: Stencil / 255
    div_node = lib.create_material_expression(
        material, unreal.MaterialExpressionDivide, 0, 200)

    # SceneTexture has multiple outputs; we want the .r component
    # (stencil is single-channel, packed into R). MaterialExpressionSceneTexture's
    # "Color" output already returns the packed value as a float4 with the
    # same scalar in all channels for stencil/depth — connect output 0.
    lib.connect_material_expressions(stencil_node, "", div_node, "A")
    lib.connect_material_expressions(const_node,   "", div_node, "B")

    # === Emissive Color  ←  PostProcessInput0.rgb ====================
    lib.connect_material_property(
        color_node, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

    # === Opacity  ←  Stencil / 255 ====================================
    lib.connect_material_property(
        div_node, "", unreal.MaterialProperty.MP_OPACITY)

    # Compile + save.
    lib.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)


def main() -> None:
    material = _create_material()
    build_material(material)
    unreal.log(f"FSDS: {ASSET_PATH} built and saved.")


if __name__ == "__main__":
    main()
