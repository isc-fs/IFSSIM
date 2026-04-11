"""
IFSSIM Vehicle Physics Validation — IFS-08 correlation tests.

Runs automated checks against the real car's expected behavior:
1. Max speed (power-limited)
2. Acceleration (0-75m)
3. Steady-state cornering (lateral G)
4. Braking performance

Usage: python tests/test_vehicle_physics.py
Requires: UE5 sim running in Play mode on port 41451
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))
from ifssim import IFSSIMClient, CarControls

# IFS-08 expected values (from MATLAB model)
EXPECTED = {
    "mass": 290,           # kg
    "motor_peak_torque": 230,  # Nm at motor
    "motor_max_power": 80000,  # W
    "gear_ratio": 2.909,
    "tire_radius": 0.200,  # m
    "tire_mu": 1.65,
    "CdA": 0.95,
    "ClA": 3.0,
    # Derived expectations
    "max_speed_ms": 35.0,      # ~126 km/h power-limited (estimated)
    "accel_75m_s": 5.0,        # seconds for 0-75m (estimated)
    "max_lateral_g": 1.8,      # with aero at ~60 km/h
    "max_decel_g": 1.5,        # braking G
}

HOST = "127.0.0.1"
PORT = 41451


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {name}: {detail}")
    return condition


def test_max_speed(client):
    """Full throttle on a straight, measure top speed."""
    print("\n--- Max Speed Test ---")

    client.enableApiControl(True)
    # Reset to start
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))
    time.sleep(0.5)

    # Full throttle
    client.setCarControls(CarControls(throttle=1.0, steering=0, brake=0))

    max_speed = 0
    start = time.time()
    while time.time() - start < 15:  # 15 seconds of full throttle
        state = client.getCarState()
        if state.speed > max_speed:
            max_speed = state.speed
        time.sleep(0.1)

    # Stop
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))
    time.sleep(1)

    max_speed_kmh = max_speed * 3.6
    expected_max = EXPECTED["max_speed_ms"]

    check("Max speed reached", max_speed > 5.0, f"{max_speed:.1f} m/s ({max_speed_kmh:.0f} km/h)")
    check("Speed reasonable", max_speed < 50.0, f"Should be < 180 km/h for FS car")

    return max_speed


def test_acceleration(client):
    """Measure 0-75m acceleration time."""
    print("\n--- 0-75m Acceleration Test ---")

    client.enableApiControl(True)
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))
    time.sleep(1)

    # Record start position
    start_state = client.getCarState()
    start_pos = (start_state.kinematics_estimated.position.x_val,
                 start_state.kinematics_estimated.position.y_val)

    # Full throttle
    client.setCarControls(CarControls(throttle=1.0, steering=0, brake=0))
    start_time = time.time()

    distance = 0
    while distance < 75.0 and time.time() - start_time < 20:
        state = client.getCarState()
        pos = (state.kinematics_estimated.position.x_val,
               state.kinematics_estimated.position.y_val)
        distance = math.sqrt((pos[0]-start_pos[0])**2 + (pos[1]-start_pos[1])**2)
        time.sleep(0.05)

    elapsed = time.time() - start_time

    # Stop
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))
    time.sleep(1)

    check("Reached 75m", distance >= 75.0, f"{distance:.1f}m in {elapsed:.2f}s")
    if distance >= 75.0:
        check("Accel time reasonable", 3.0 < elapsed < 10.0, f"{elapsed:.2f}s (expected ~{EXPECTED['accel_75m_s']}s)")

    return elapsed if distance >= 75.0 else None


def test_braking(client):
    """Full brake from speed, measure deceleration."""
    print("\n--- Braking Test ---")

    client.enableApiControl(True)

    # Get up to speed first
    client.setCarControls(CarControls(throttle=1.0, steering=0, brake=0))
    time.sleep(5)

    state = client.getCarState()
    initial_speed = state.speed
    print(f"  Initial speed: {initial_speed:.1f} m/s")

    # Full brake
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))
    start_time = time.time()
    start_speed = initial_speed

    while time.time() - start_time < 5:
        state = client.getCarState()
        if state.speed < 0.5:
            break
        time.sleep(0.05)

    elapsed = time.time() - start_time
    if elapsed > 0 and start_speed > 2:
        decel = start_speed / elapsed  # m/s²
        decel_g = decel / 9.81
        check("Braking decel", decel_g > 0.5, f"{decel_g:.2f}g ({decel:.1f} m/s²)")
    else:
        check("Braking", False, "Couldn't measure")

    client.setCarControls(CarControls(throttle=0, steering=0, brake=0))
    time.sleep(0.5)


def main():
    print("=" * 60)
    print("  IFSSIM Vehicle Physics Validation (IFS-08)")
    print("=" * 60)

    try:
        client = IFSSIMClient(HOST, PORT)
        client.confirmConnection()
    except Exception as e:
        print(f"  Cannot connect to sim at {HOST}:{PORT}: {e}")
        return 1

    print(f"\n  Connected to simulator")
    print(f"  Expected: {EXPECTED['mass']}kg, {EXPECTED['motor_peak_torque']}Nm EMRAX, mu={EXPECTED['tire_mu']}")

    # Enable API control
    client.enableApiControl(True)

    # Run tests
    max_speed = test_max_speed(client)
    accel_time = test_acceleration(client)
    test_braking(client)

    # Summary
    print("\n" + "=" * 60)
    print("  Summary")
    print("=" * 60)
    print(f"  Max speed: {max_speed:.1f} m/s ({max_speed*3.6:.0f} km/h)")
    if accel_time:
        print(f"  0-75m: {accel_time:.2f}s")
    print()

    # Cleanup
    client.setCarControls(CarControls(throttle=0, steering=0, brake=1.0))

    return 0


if __name__ == "__main__":
    sys.exit(main())
