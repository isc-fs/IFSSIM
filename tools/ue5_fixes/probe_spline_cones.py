# Probe the spline_cones BP — find what z it actually spawns cones at.
import unreal

SPLINE_BPS = [
    "/Game/RaceCourse/Model/Splines/spline_cones",
    "/Game/RaceCourse/Model/Splines/spline_cones_best",
    "/Game/RaceCourse/Model/Splines/spline_cones_mini_orange",
]

def main():
    for path in SPLINE_BPS:
        if not unreal.EditorAssetLibrary.does_asset_exist(path):
            unreal.log_warning(f"MISSING: {path}")
            continue
        bp = unreal.EditorAssetLibrary.load_asset(path)
        unreal.log_warning(f"--- {path.split('/')[-1]} ---")
        unreal.log_warning(f"  type: {type(bp).__name__}")
        # The Blueprint's CDO has the editable variable defaults.
        gen_cls = bp.generated_class()
        cdo = unreal.get_default_object(gen_cls)
        unreal.log_warning(f"  generated_class: {gen_cls.get_name()}")
        # Probe known editable variable names from the strings dump.
        candidates = [
            "NodeHeight", "Node Height",
            "HitLocation", "Hit Location",
            "blue_cone_locations", "yellow_cone_locations",
            "SavedViewOffset",
        ]
        for name in candidates:
            try:
                v = cdo.get_editor_property(name)
                # Be polite about big collections
                if isinstance(v, list) and len(v) > 3:
                    unreal.log_warning(f"  HAS '{name}' = list of {len(v)}")
                else:
                    unreal.log_warning(f"  HAS '{name}' = {v}")
            except Exception:
                pass
        # Try the bigger reflection trick — list all editor properties
        # by iterating the BP's property names.
        try:
            for prop_name in unreal.SystemLibrary.get_class_default_object(gen_cls).__dir__():
                if prop_name.startswith('_'):
                    continue
                # Heuristic skip
                if prop_name in dir(unreal.Actor):
                    continue
                unreal.log_warning(f"  attr: {prop_name}")
        except Exception as e:
            unreal.log_warning(f"  (dir failed: {e})")

main()
