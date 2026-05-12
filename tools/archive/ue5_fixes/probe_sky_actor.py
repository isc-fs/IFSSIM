# Probe — uses log_warning so output shows up under default log filter.
import unreal

def main():
    if not unreal.EditorLevelLibrary.load_level("/Game/TrainingMap"):
        unreal.log_error("Failed to load TrainingMap")
        return
    n = 0
    for a in unreal.EditorLevelLibrary.get_all_level_actors():
        cls = a.get_class()
        cls_name = cls.get_name()
        if "Sky" not in cls_name:
            continue
        n += 1
        unreal.log_warning(f"--- {a.get_name()} class={cls_name} ---")
        # Iterate the UClass's editor properties using FProperty traversal.
        # In UE5 Python: there's no direct iterator exposed, but
        # `unreal.SystemLibrary.get_class_display_name` etc are reflection-only.
        # We can dump the BP variable list via cls.get_class_default_object():
        cdo = unreal.get_default_object(cls)
        # And use built-in dir() on the actor to discover Python-callable names:
        for attr in sorted(set(dir(a)) - set(dir(unreal.Actor))):
            if attr.startswith('_'):
                continue
            unreal.log_warning(f"  attr: {attr}")
        # Also try the named candidates
        for name in ["StarsBrightness", "Stars Brightness", "stars_brightness",
                     "CloudSpeed", "CloudOpacity", "DirectionalLightActor",
                     "RefreshMaterial", "UpdateSunDirection", "SunBrightness"]:
            try:
                v = a.get_editor_property(name)
                unreal.log_warning(f"  HAS '{name}' = {v}")
            except Exception as e:
                pass
    unreal.log_warning(f"DONE — {n} sky actors found")

main()
