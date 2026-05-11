# Closes #401 — disable sky-sphere stars on all maps.
#
# UE5 Python commandlet. Runs headless (no editor UI). For each
# .umap under /Game/ that contains an actor based on
# /Engine/EngineSky/BP_Sky_Sphere (the engine's stock sky-sphere
# Blueprint, instance-named `SkyShereBlueprint` in the affected maps),
# this:
#
#   1. Loads the level.
#   2. Finds the sky-sphere actor instance.
#   3. Sets its `StarsBrightness` property to 0.0.
#   4. Calls `Refresh Material` (the BP's `UpdateSunDirection`/refresh
#      function) so the property change takes effect at runtime + at
#      next save.
#   5. Marks the package dirty + saves.
#
# Invoke from the repo root:
#
#   /Users/Shared/Epic\ Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor \
#     "$(pwd)/IFSSIM.uproject" \
#     -run=pythonscript \
#     -script="$(pwd)/tools/ue5_fixes/fix_sky_stars.py"
#
# The script is idempotent — running it twice does nothing the second
# time (StarsBrightness is already 0, save is skipped).

import unreal


# All maps known to use the stock BP_Sky_Sphere. Hard-coded rather
# than auto-discovered so a one-off scratch map can't inadvertently
# get its sky stripped. Add/remove paths here when the level layout
# changes.
TARGET_MAPS = [
    "/Game/TrainingMap",
    "/Game/customMap",
    "/Game/Acceleration",
    "/Game/Skidpad",
    "/Game/RaceCourse/Maps/IM_MAP1",
    "/Game/RaceCourse/Maps/Demo",
    "/Game/RaceCourse/Maps/Assets",
]

SKY_BP_PATH = "/Engine/EngineSky/BP_Sky_Sphere"


def fix_one_map(map_path):
    """Returns (status_str, n_actors_patched)."""
    asset_lib = unreal.EditorAssetLibrary
    if not asset_lib.does_asset_exist(map_path):
        return ("MISSING", 0)

    # Load the level. EditorLevelLibrary.load_level expects the long
    # path WITHOUT a trailing .<asset_name>; the path is the package
    # name (`/Game/TrainingMap`), unreal resolves the .umap under it.
    if not unreal.EditorLevelLibrary.load_level(map_path):
        return ("LOAD_FAIL", 0)

    # All actors in the loaded level. Filter to ones whose class
    # inherits from BP_Sky_Sphere. The Blueprint class object
    # itself is `BP_Sky_Sphere_C` once cooked; checking by class
    # name is the robust path.
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    n_patched = 0
    for a in actors:
        cls = a.get_class()
        cls_name = cls.get_name()
        if "Sky_Sphere" not in cls_name:
            continue
        # The Stars Brightness float property. Property NAME is the
        # display-name with the space, NOT the no-space PascalCase
        # version (confirmed 2026-05-11 by probing the BP_Sky_Sphere_C
        # CDO in UE5.7 — see tools/ue5_fixes/probe_sky_actor.py).
        try:
            current = a.get_editor_property("Stars Brightness")
        except Exception:
            unreal.log_warning(
                f"{map_path}: actor {a.get_name()} has no "
                f"'Stars Brightness' property; skipping")
            continue
        if current == 0.0:
            unreal.log_warning(
                f"{map_path}: {a.get_name()} already at 0; no-op")
            continue
        a.set_editor_property("Stars Brightness", 0.0)
        # Trigger the BP's RefreshMaterial-style update so the change
        # is reflected at game-runtime too (the BP's construction
        # script bakes StarsBrightness into a dynamic-material-
        # instance parameter; without a re-run, the runtime material
        # still has the old value until the BP construction script
        # fires again — which happens on load anyway, but better to
        # be explicit).
        try:
            a.call_method("RefreshMaterial")
        except Exception:
            # Older UE5 versions named it differently or never exposed
            # it — the save+reload-on-launch path still works without.
            pass
        n_patched += 1
        unreal.log(f"{map_path}: patched {a.get_name()} StarsBrightness -> 0.0")

    if n_patched == 0:
        return ("NO_SKY_ACTOR", 0)

    # Save the level. EditorAssetLibrary.save_asset takes the same
    # /Game/... path; flush is the safer mode.
    if not asset_lib.save_asset(map_path, only_if_is_dirty=False):
        return ("SAVE_FAIL", n_patched)

    return ("OK", n_patched)


def main():
    total_patched = 0
    failures = []
    for m in TARGET_MAPS:
        status, n = fix_one_map(m)
        unreal.log(f"==> {m}: {status} ({n} actor(s))")
        total_patched += n
        if status not in ("OK", "NO_SKY_ACTOR"):
            failures.append((m, status))

    unreal.log(f"DONE — {total_patched} sky-sphere instances patched "
               f"across {len(TARGET_MAPS)} maps")
    if failures:
        unreal.log_error(f"FAILURES: {failures}")


main()
