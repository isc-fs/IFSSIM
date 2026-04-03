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
    """Get LiDAR point cloud as list of (x,y,z) tuples."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((HOST, PORT))

    sock.sendall(b"getLidarDataBinary\n")

    # Read header: "PTS:num_points\n"
    header = recv_line(sock)
    if not header.startswith("PTS:"):
        sock.close()
        return [], header

    num_points = int(header.split(":")[1])

    # Read raw float data (3 floats per point)
    byte_size = num_points * 3 * 4  # 3 floats * 4 bytes each
    raw_data = recv_all(sock, byte_size)
    sock.close()

    # Parse floats
    num_floats = len(raw_data) // 4
    floats = struct.unpack(f"<{num_floats}f", raw_data)

    points = []
    for i in range(0, len(floats), 3):
        if i + 2 < len(floats):
            points.append((floats[i], floats[i+1], floats[i+2]))

    return points, f"PTS:{num_points}"

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

    # Test 4: LiDAR point cloud
    print("[4] Getting LiDAR point cloud...")
    points, header = get_lidar()
    if len(points) > 0:
        print(f"    [OK] {header} — {len(points)} points received")
        print(f"    Sample points:")
        for i, p in enumerate(points[:5]):
            print(f"      [{i}] x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}")
    else:
        print(f"    [FAIL] {header} — 0 points")

    print()
    print("=" * 50)
    print("  Test complete!")
    print("=" * 50)

if __name__ == "__main__":
    main()
