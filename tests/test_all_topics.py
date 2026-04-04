"""
IFSSIM — Comprehensive ROS2 Topic Data Test
Tests every sensor endpoint that the ROS2 bridge would use.
Run while the simulator is in Play mode.

Usage: python test_all_topics.py
"""
import socket
import struct
import time
import json
import sys

HOST = "127.0.0.1"
PORT = 41451

class SimClient:
    """Mimics what the ROS2 bridge does."""

    def text_cmd(self, cmd):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((HOST, PORT))
            s.sendall((cmd + "\n").encode())
            time.sleep(0.3)
            data = s.recv(65536).decode().strip()
            s.close()
            return data
        except Exception as e:
            return f"ERROR: {e}"

    def binary_cmd(self, cmd):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(10)
            s.connect((HOST, PORT))
            s.sendall((cmd + "\n").encode())

            # Read header line
            header = b""
            while True:
                byte = s.recv(1)
                if not byte or byte == b"\n":
                    break
                header += byte
            header = header.decode()

            # Parse size
            colon = header.find(":")
            if colon < 0:
                s.close()
                return header, b""

            prefix = header[:colon]
            count = int(header[colon+1:])

            if prefix == "PTS":
                byte_size = count * 3 * 4
            elif prefix == "IMG":
                byte_size = count
            else:
                s.close()
                return header, b""

            # Read binary data
            data = b""
            while len(data) < byte_size:
                chunk = s.recv(byte_size - len(data))
                if not chunk:
                    break
                data += chunk
            s.close()
            return header, data
        except Exception as e:
            return f"ERROR: {e}", b""

    def parse(self, json_str, key):
        try:
            search = f'"{key}":'
            pos = json_str.find(search)
            if pos < 0: return None
            pos += len(search)
            end = json_str.find(",", pos)
            if end < 0: end = json_str.find("}", pos)
            val = json_str[pos:end].strip().strip('"')
            try: return float(val)
            except: return val
        except:
            return None


def main():
    c = SimClient()
    passed = 0
    failed = 0
    total = 0

    def check(name, condition, detail=""):
        nonlocal passed, failed, total
        total += 1
        if condition:
            passed += 1
            print(f"  [PASS] {name}: {detail}")
        else:
            failed += 1
            print(f"  [FAIL] {name}: {detail}")

    print("=" * 60)
    print("  IFSSIM — Comprehensive Topic Data Test")
    print("=" * 60)

    # 1. Connection
    print("\n--- Connection ---")
    resp = c.text_cmd("ping")
    check("ping", resp == "true", resp)

    resp = c.text_cmd("enableApiControl")
    check("enableApiControl", resp == "true", resp)

    # 2. Camera discovery
    print("\n--- Camera Discovery ---")
    resp = c.text_cmd("listCameras")
    check("listCameras", "[" in resp, resp)
    cameras = []
    if resp.startswith("["):
        cameras = [x.strip().strip('"') for x in resp[1:-1].split(",") if x.strip()]
    check("cameras found", len(cameras) >= 1, f"{len(cameras)} cameras: {cameras}")

    # 3. GPS (10 Hz topic: /gps)
    print("\n--- GPS (NavSatFix @ 10Hz) ---")
    resp = c.text_cmd("getGpsData")
    check("getGpsData responds", "lat" in resp, resp[:80])
    lat = c.parse(resp, "lat")
    lon = c.parse(resp, "lon")
    alt = c.parse(resp, "alt")
    check("latitude valid", lat is not None and abs(lat) > 0, f"lat={lat}")
    check("longitude valid", lon is not None and abs(lon) > 0, f"lon={lon}")
    check("altitude valid", alt is not None and alt > 0, f"alt={alt}")

    # 4. IMU (400 Hz topic: /imu)
    print("\n--- IMU (Imu @ 400Hz) ---")
    resp = c.text_cmd("getImuData")
    check("getImuData responds", "ax" in resp, resp[:80])
    az = c.parse(resp, "az")
    check("gravity ~9.8 m/s²", az is not None and 9.0 < abs(az) < 11.0, f"az={az}")
    qw = c.parse(resp, "qw")
    check("orientation quaternion", qw is not None and 0.9 < abs(qw) < 1.1, f"qw={qw}")

    # 5. GSS (100 Hz topic: /gss)
    print("\n--- GSS (TwistWithCovarianceStamped @ 100Hz) ---")
    resp = c.text_cmd("getGroundSpeedSensorData")
    check("getGroundSpeedSensorData responds", "vx" in resp, resp[:80])

    # 6. LiDAR text endpoint
    print("\n--- LiDAR Text (PointCloud2 metadata) ---")
    resp = c.text_cmd("getLidarData")
    check("getLidarData responds", "points" in resp, resp[:80])
    pts = c.parse(resp, "points")
    channels = c.parse(resp, "channels")
    check("point count > 0", pts is not None and pts > 0, f"points={pts}")
    check("channels = 128", channels == 128, f"channels={channels}")

    # 7. LiDAR binary (what ROS2 bridge uses)
    print("\n--- LiDAR Binary (PointCloud2 @ 10Hz) ---")
    header, data = c.binary_cmd("getLidarDataBinary")
    check("binary header", header.startswith("PTS:"), header)

    if header.startswith("PTS:"):
        num_pts = int(header.split(":")[1])
        expected = num_pts * 3 * 4
        check(f"points received", num_pts > 0, f"{num_pts} points")
        check(f"data size", len(data) == expected, f"{len(data)}/{expected} bytes")

        if num_pts > 0 and len(data) >= 12:
            floats = struct.unpack(f"<{min(num_pts*3, len(data)//4)}f", data[:min(num_pts*3*4, len(data))])
            x, y, z = floats[0], floats[1], floats[2]
            dist = (x*x + y*y + z*z) ** 0.5
            check("first point valid", dist > 0.1 and dist < 300, f"({x:.2f},{y:.2f},{z:.2f}) dist={dist:.1f}m")

            distances = [(floats[i*3]**2+floats[i*3+1]**2+floats[i*3+2]**2)**0.5 for i in range(min(num_pts, len(floats)//3))]
            check("range realistic", min(distances) > 0.3 and max(distances) < 250, f"min={min(distances):.1f}m max={max(distances):.1f}m")

    # 8. Camera images (what ROS2 bridge uses)
    print("\n--- Camera Images (CompressedImage @ 10Hz) ---")
    for cam in cameras:
        header, data = c.binary_cmd(f"simGetImageBinary {cam} 0")
        check(f"{cam} header", header.startswith("IMG:"), header)
        if header.startswith("IMG:"):
            size = int(header.split(":")[1])
            check(f"{cam} data received", len(data) > 0, f"{len(data)} bytes")
            is_png = data[:4] == b"\x89PNG" if len(data) >= 4 else False
            check(f"{cam} valid PNG", is_png, f"header={data[:4].hex() if len(data)>=4 else 'empty'}")
            check(f"{cam} reasonable size", len(data) > 10000, f"{len(data)/1024:.0f} KB")

    # 9. Odometry (250 Hz topic: /testing_only/odom)
    print("\n--- Odometry (Odometry @ 250Hz) ---")
    resp = c.text_cmd("getCarState")
    check("getCarState responds", "speed" in resp, resp[:80])
    x = c.parse(resp, "x")
    rpm = c.parse(resp, "rpm")
    check("position in meters (ENU)", x is not None, f"x={x}")
    check("RPM valid", rpm is not None and rpm >= 0, f"rpm={rpm}")

    # 10. Ground truth kinematics
    print("\n--- Ground Truth Kinematics ---")
    resp = c.text_cmd("simGetGroundTruthKinematics")
    check("kinematics responds", "px" in resp, resp[:80])
    qw = c.parse(resp, "qw")
    check("quaternion present", qw is not None, f"qw={qw}")

    # 11. Distance sensor
    print("\n--- Distance Sensor ---")
    resp = c.text_cmd("getDistanceSensorData")
    check("distance responds", "distance" in resp, resp[:80])

    # 12. Barometer
    print("\n--- Barometer ---")
    resp = c.text_cmd("getBarometerData")
    check("barometer responds", "pressure" in resp, resp[:80])
    pressure = c.parse(resp, "pressure")
    check("pressure ~100kPa", pressure is not None and 90000 < pressure < 110000, f"{pressure:.0f} Pa")

    # 13. Magnetometer
    print("\n--- Magnetometer ---")
    resp = c.text_cmd("getMagnetometerData")
    check("magnetometer responds", "mx" in resp, resp[:80])

    # 14. Referee
    print("\n--- Referee ---")
    resp = c.text_cmd("getRefereeState")
    check("referee responds", "doo_counter" in resp, resp[:80])

    # 15. Simulation control
    print("\n--- Simulation Control ---")
    check("simIsPaused", c.text_cmd("simIsPaused") in ("true","false"), c.text_cmd("simIsPaused"))

    # Summary
    print("\n" + "=" * 60)
    print(f"  RESULTS: {passed}/{total} passed, {failed} failed")
    print("=" * 60)

    if failed > 0:
        print("\n  FAILED TESTS NEED ATTENTION!")
    else:
        print("\n  ALL TESTS PASSED — ROS2 bridge data is ready!")

    return failed


if __name__ == "__main__":
    sys.exit(main())
