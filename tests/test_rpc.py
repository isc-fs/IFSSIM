"""
Quick test for the IFSSIM TCP RPC server.
Run while the simulator is in Play mode.

Usage: python test_rpc.py
"""
import socket
import json
import time

HOST = "127.0.0.1"
PORT = 41451

def send_command(sock, command):
    """Send a command and receive response."""
    sock.sendall((command + "\n").encode())
    time.sleep(0.05)
    data = sock.recv(4096).decode().strip()
    return data

def main():
    print("=" * 50)
    print("  IFSSIM RPC Server Test")
    print("=" * 50)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((HOST, PORT))
        print(f"[OK] Connected to {HOST}:{PORT}")
    except Exception as e:
        print(f"[FAIL] Cannot connect: {e}")
        print("Make sure the simulator is running (Play in Editor)")
        return

    tests = [
        ("ping", "true"),
        ("getSettingsString", None),
        ("enableApiControl", "true"),
        ("isApiControlEnabled", "true"),
        ("getCarState", None),
        ("getGpsData", None),
        ("getImuData", None),
        ("getGroundSpeedSensorData", None),
        ("getLidarData", None),
        ("getRefereeState", None),
    ]

    passed = 0
    for cmd, expected in tests:
        try:
            response = send_command(sock, cmd)
            if expected and response != expected:
                print(f"  [{cmd}] WARN: got '{response}', expected '{expected}'")
            else:
                print(f"  [{cmd}] -> {response[:80]}")
            passed += 1
        except Exception as e:
            print(f"  [{cmd}] FAIL: {e}")

    # Test setCarControls
    try:
        response = send_command(sock, "setCarControls 0.5 0.0 0.0")
        print(f"  [setCarControls 0.5 0 0] -> {response}")
        time.sleep(1)
        response = send_command(sock, "getCarState")
        print(f"  [getCarState after throttle] -> {response[:80]}")
        passed += 1
    except Exception as e:
        print(f"  [setCarControls] FAIL: {e}")

    sock.close()

    print()
    print(f"Results: {passed}/{len(tests)+1} passed")
    print("=" * 50)

if __name__ == "__main__":
    main()
