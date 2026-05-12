# Probe — inspect cone static-mesh pivot positions relative to their
# bounding box, so we know whether the pivot is centred (causing the
# #402 clipping bug) or already at the base.
import unreal

CONE_ASSETS = [
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/Traffic_Cone",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/yellow_trafficone",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/blue_trafficone",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_trafficone",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_blue",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_orange",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_mini_trafficone",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_big_orange",
    "/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_bigmac_orange",
]

def main():
    for path in CONE_ASSETS:
        if not unreal.EditorAssetLibrary.does_asset_exist(path):
            unreal.log_warning(f"MISSING: {path}")
            continue
        mesh = unreal.EditorAssetLibrary.load_asset(path)
        if not isinstance(mesh, unreal.StaticMesh):
            unreal.log_warning(f"NOT_MESH: {path} (got {type(mesh).__name__})")
            continue
        # local-space bounds — origin is the pivot, extent goes ±
        # half-size around it.
        bounds = mesh.get_bounds()
        origin = bounds.origin       # Vector — pivot-to-bounds-center
        extent = bounds.box_extent   # Vector — half-size
        z_min = origin.z - extent.z
        z_max = origin.z + extent.z
        unreal.log_warning(
            f"{path.split('/')[-1]}: "
            f"origin_z={origin.z:+.2f}  extent_z={extent.z:+.2f}  "
            f"=>  z_range [{z_min:+.2f}, {z_max:+.2f}]  "
            f"pivot_offset_from_base={-z_min:+.2f}")

main()
