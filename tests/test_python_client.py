"""
Test the IFSSIM Python client — verifies API compatibility with FSDS.
Run while the simulator is in Play mode.

Usage: python test_python_client.py
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from ifssim import IFSSIMClient, CarControls, ImageType, ImageRequest

def main():
    passed = 0
    failed = 0

    def check(name, condition, detail=""):
        nonlocal passed, failed
        if condition:
            passed += 1
            print(f"  [PASS] {name}: {detail}")
        else:
            failed += 1
            print(f"  [FAIL] {name}: {detail}")

    print("=" * 60)
    print("  IFSSIM Python Client Test (FSDS API compatible)")
    print("=" * 60)

    # Connect — same API as FSDS
    client = IFSSIMClient()

    print("\n--- Connection ---")
    check("ping", client.ping(), "pong")
    try:
        client.confirmConnection()
        check("confirmConnection", True, "connected")
    except:
        check("confirmConnection", False, "failed")
        return 1

    client.enableApiControl(True)
    check("enableApiControl", True)
    check("isApiControlEnabled", client.isApiControlEnabled())

    # Car state — same API as FSDS
    print("\n--- Car State ---")
    state = client.getCarState()
    check("getCarState returns CarState", hasattr(state, 'speed'), type(state).__name__)
    check("speed is float", isinstance(state.speed, float), f"{state.speed}")
    check("rpm > 0", state.rpm > 0, f"rpm={state.rpm}")
    check("kinematics_estimated", hasattr(state.kinematics_estimated, 'position'))
    pos = state.kinematics_estimated.position
    check("position is Vector3r", hasattr(pos, 'x_val'), f"({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f})")

    # Car controls — same API as FSDS
    print("\n--- Car Controls ---")
    controls = CarControls(throttle=0.5, steering=0.1, brake=0.0)
    client.setCarControls(controls)
    check("setCarControls", True, "throttle=0.5, steering=0.1")
    import time; time.sleep(0.5)
    state2 = client.getCarState()
    check("car responded", True, f"speed={state2.speed:.4f}")

    # Stop the car
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))

    # GPS — same API as FSDS
    print("\n--- GPS ---")
    gps = client.getGpsData()
    check("getGpsData returns GpsData", hasattr(gps, 'gnss'))
    check("latitude", abs(gps.gnss.geo_point.latitude) > 0, f"{gps.gnss.geo_point.latitude:.6f}")
    check("longitude", abs(gps.gnss.geo_point.longitude) > 0, f"{gps.gnss.geo_point.longitude:.6f}")

    # IMU — same API as FSDS
    print("\n--- IMU ---")
    imu = client.getImuData()
    check("getImuData returns ImuData", hasattr(imu, 'linear_acceleration'))
    check("gravity ~9.8", 9.0 < abs(imu.linear_acceleration.z_val) < 11.0, f"az={imu.linear_acceleration.z_val:.2f}")
    check("orientation", abs(imu.orientation.w_val) > 0.9, f"qw={imu.orientation.w_val:.4f}")

    # GSS — same API as FSDS
    print("\n--- GSS ---")
    gss = client.getGroundSpeedSensorData()
    check("getGroundSpeedSensorData", hasattr(gss, 'linear_velocity'))

    # LiDAR — same API as FSDS
    print("\n--- LiDAR ---")
    lidar = client.getLidarData()
    check("getLidarData returns LidarData", hasattr(lidar, 'point_cloud'))
    num_points = len(lidar.point_cloud) // 3
    check("point_cloud has data", num_points > 0, f"{num_points} points")
    if num_points > 0:
        x, y, z = lidar.point_cloud[0], lidar.point_cloud[1], lidar.point_cloud[2]
        dist = (x*x + y*y + z*z) ** 0.5
        check("first point valid", dist > 0.1, f"({x:.2f},{y:.2f},{z:.2f}) dist={dist:.1f}m")

    # Camera — same API as FSDS
    print("\n--- Camera ---")
    img = client.simGetImage("cam1", ImageType.Scene)
    check("simGetImage returns bytes", img is not None and len(img) > 0, f"{len(img)//1024} KB")
    check("valid PNG", img[:4] == b"\x89PNG" if img else False)

    # Batch images — same API as FSDS
    imgs = client.simGetImages([
        ImageRequest("cam1", ImageType.Scene),
        ImageRequest("cam2", ImageType.Scene),
    ])
    check("simGetImages batch", len(imgs) == 2, f"{len(imgs)} images")
    check("both valid", all(i and len(i) > 0 for i in imgs))

    # Ground truth — same API as FSDS
    print("\n--- Ground Truth ---")
    kin = client.simGetGroundTruthKinematics()
    check("simGetGroundTruthKinematics", hasattr(kin, 'position'))
    check("has orientation", abs(kin.orientation.w_val) > 0.9, f"qw={kin.orientation.w_val:.4f}")

    # Referee — same API as FSDS
    print("\n--- Referee ---")
    ref = client.getRefereeState()
    check("getRefereeState", hasattr(ref, 'doo_counter'), f"doo={ref.doo_counter}")
    check("laps count", hasattr(ref, 'laps'), f"laps={ref.laps}")
    check("lap_times is list", isinstance(ref.lap_times, list), f"lap_times={ref.lap_times}")
    check("cone_count > 0", ref.cone_count > 0, f"cone_count={ref.cone_count}")
    check("cones is list", isinstance(ref.cones, list), f"{len(ref.cones)} cones")
    if ref.cones:
        c = ref.cones[0]
        check("cone has x,y,color", hasattr(c, 'x') and hasattr(c, 'color'), f"({c.x:.2f},{c.y:.2f}) color={c.color}")

    # Settings
    print("\n--- Settings ---")
    settings = client.getSettingsString()
    check("getSettingsString", len(settings) > 10, f"{len(settings)} chars")

    # Simulation control
    print("\n--- Simulation Control ---")
    check("simIsPaused", client.simIsPaused() in (True, False))

    # Summary
    print("\n" + "=" * 60)
    print(f"  RESULTS: {passed}/{passed+failed} passed, {failed} failed")
    print("=" * 60)

    if failed == 0:
        print("\n  Python client is FSDS API compatible!")
    return failed


if __name__ == "__main__":
    sys.exit(main())
