"""
Test binary data transfer — images and LiDAR point clouds.
Run while the simulator is in Play mode.

Usage: python test_binary.py
"""
import socket
import struct
import time
import os

HOST = "127.0.0.1"
PORT = 41451

def recv_all(sock, size):
    """Receive exactly `size` bytes."""
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            break
        data += chunk
    return data

def recv_line(sock):
    """Receive until newline."""
    data = b""
    while True:
        byte = sock.recv(1)
        if not byte or byte == b"\n":
            break
        data += byte
    return data.decode()

def get_image(camera="cam1", image_type=0):
    """Capture an image and return PNG bytes."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((HOST, PORT))

    cmd = f"simGetImageBinary {camera} {image_type}\n"
    sock.sendall(cmd.encode())

    # Read header: "IMG:size\n"
    header = recv_line(sock)
    if not header.startswith("IMG:"):
        sock.close()
        return None, header

    size = int(header.split(":")[1])

    # Read raw PNG bytes
    png_data = recv_all(sock, size)
    sock.close()
    return png_data, f"IMG:{size}"

def get_lidar():
    """LiDAR point-cloud retrieval over TCP was removed in #322 — the
    `getLidarDataBinary` RPC and `streamLidar` push are both gone.
    LiDAR is UDP-only now (port 51453), consumed by `udp_receiver.h`
    in the bridge. This test stub stays so the rest of `test_binary.py`
    keeps running; the LiDAR coverage moved to the bridge integration
    tests."""
    return [], "skipped:lidar-tcp-rpc-removed-in-322"


def main():
    print("=" * 50)
    print("  IFSSIM Binary Data Transfer Test")
    print("=" * 50)
    print()

    os.makedirs("tests/output", exist_ok=True)

    # Test 1: Scene image from cam1
    print("[1] Capturing scene image from cam1...")
    png_data, header = get_image("cam1", 0)
    if png_data and len(png_data) > 0:
        # Verify PNG header
        is_png = png_data[:4] == b"\x89PNG"
        with open("tests/output/cam1_scene.png", "wb") as f:
            f.write(png_data)
        print(f"    [OK] {header} — {len(png_data)} bytes, PNG={is_png}")
        print(f"    Saved to tests/output/cam1_scene.png")
    else:
        print(f"    [FAIL] {header}")

    # Test 2: Scene image from cam2
    print("[2] Capturing scene image from cam2...")
    png_data, header = get_image("cam2", 0)
    if png_data and len(png_data) > 0:
        with open("tests/output/cam2_scene.png", "wb") as f:
            f.write(png_data)
        print(f"    [OK] {header} — {len(png_data)} bytes")
        print(f"    Saved to tests/output/cam2_scene.png")
    else:
        print(f"    [FAIL] {header}")

    # Test 3: Depth image
    print("[3] Capturing depth image from cam1...")
    png_data, header = get_image("cam1", 1)
    if png_data and len(png_data) > 0:
        with open("tests/output/cam1_depth.png", "wb") as f:
            f.write(png_data)
        print(f"    [OK] {header} — {len(png_data)} bytes")
        print(f"    Saved to tests/output/cam1_depth.png")
    else:
        print(f"    [FAIL] {header}")

    # Test 4: LiDAR point cloud — skipped since #322. The TCP-LiDAR
    # RPCs that this test exercised are gone; LiDAR consumers must
    # subscribe to /lidar/Lidar1 from the ROS bridge instead.
    print("[4] Getting LiDAR point cloud...")
    points, header = get_lidar()
    print(f"    [SKIPPED] {header}")

    print()
    print("=" * 50)
    print("  Test complete!")
    print("=" * 50)

if __name__ == "__main__":
    main()
