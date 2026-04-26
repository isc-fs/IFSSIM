#!/usr/bin/env python3
"""
Record the vehicle's trajectory in odom/ENU during a session, then plot it
on top of the ground-truth track + detected cones + the latest planned path.

Designed to run inside the ros_stack container alongside the live pipeline.
Run BEFORE starting the session so the trajectory captures from t=0:

    docker exec -it ifssim-ros_stack-1 bash -c \
      "source /opt/ros/humble/setup.bash && \
       source /ros_stack_ws/install/setup.bash && \
       python3 /ros_stack_ws/src/tools/trajectory_record.py --duration 30"

Or attach for a fixed duration with --duration N (seconds), or stop early
with Ctrl-C — either way a PNG drops at /tmp/trajectory.png.
"""
import argparse
import json
import math
import signal
import socket
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener


def get_groundtruth_cones():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("host.docker.internal", 41451))
    s.sendall(b"getRefereeState\n")
    data = b""
    while b"\n" not in data:
        data += s.recv(65536)
    s.close()
    ref = json.loads(data.decode().strip())
    return ref.get("cone_positions", [])


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        self.path = None
        self.detected_cones = None
        self.trajectory = []  # list of (t, x, y, yaw)

        self.create_subscription(Path, "/Path", self._on_path, 10)
        be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(MarkerArray, "/Conos", self._on_cones, be)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # Sample TF at 20 Hz (50 ms). Higher than DIAG (5 Hz) so the
        # plotted trajectory captures the full curve dynamics, not just
        # one point per Stanley tick.
        self.create_timer(0.05, self._sample_tf)

    def _on_path(self, msg):
        self.path = msg

    def _on_cones(self, msg):
        out = []
        for m in msg.markers:
            if m.action == 3:
                out = []
                continue
            out.append(
                (
                    m.pose.position.x,
                    m.pose.position.y,
                    m.color.r,
                    m.color.g,
                    m.color.b,
                )
            )
        self.detected_cones = out

    def _sample_tf(self):
        try:
            tf = self.tf_buf.lookup_transform(
                "odom", "fsds/FSCar", rclpy.time.Time()
            )
        except Exception:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.trajectory.append((time.time(), x, y, yaw))


def render_plot(node: TrajectoryRecorder, out_path: str):
    cones = []
    try:
        cones = get_groundtruth_cones()
    except Exception as ex:
        print(f"WARN: ground-truth fetch failed: {ex}")

    print(f"GT cones: {len(cones)}")
    print(f"Trajectory samples: {len(node.trajectory)}")
    print(
        f"Detected /Conos: "
        f"{0 if node.detected_cones is None else len(node.detected_cones)}"
    )
    print(f"Path captured: {node.path is not None and len(node.path.poses) > 0}")

    fig, ax = plt.subplots(1, 1, figsize=(14, 14))
    fig.patch.set_facecolor("#1a1a1a")
    ax.set_facecolor("#0e0e0e")

    # Ground truth cones (background)
    gt_colors = {0: "#ffd00060", 1: "#3b82f660", 2: "#ff6a0080"}
    gt_labels = {0: "GT yellow", 1: "GT blue", 2: "GT big_orange"}
    seen = set()
    for c in cones:
        col = gt_colors.get(c["color"], "#88888860")
        lab = gt_labels.get(c["color"]) if c["color"] not in seen else None
        seen.add(c["color"])
        size = 60 if c["color"] == 2 else 22
        ax.scatter(
            c["x"], c["y"], c=col, s=size, edgecolors="none", label=lab, zorder=2
        )

    # Detected /Conos at the end of the run (foreground markers)
    if node.detected_cones:
        det_x = [c[0] for c in node.detected_cones]
        det_y = [c[1] for c in node.detected_cones]
        det_c = [(c[2], c[3], c[4]) for c in node.detected_cones]
        ax.scatter(
            det_x,
            det_y,
            c=det_c,
            s=70,
            edgecolors="#fff",
            linewidths=0.8,
            label=f"detected /Conos ({len(node.detected_cones)})",
            zorder=4,
            marker="X",
        )

    # Latest planned path
    if node.path is not None and node.path.poses:
        px = [p.pose.position.x for p in node.path.poses]
        py = [p.pose.position.y for p in node.path.poses]
        ax.plot(
            px,
            py,
            color="#00e5ff",
            linewidth=2.0,
            label=f"latest /Path ({len(px)} poses)",
            zorder=5,
            marker="o",
            markersize=3,
        )

    # Vehicle trajectory — the headline plot. Drawn as a line + scatter
    # so the full path is visible even when many samples cluster at the
    # same world location (car stuck) and the dots would otherwise over-
    # plot into a single visible point. Markers are colour-by-time so
    # the direction of travel and stuck regions are obvious.
    if node.trajectory:
        ts = [s[0] for s in node.trajectory]
        tx = [s[1] for s in node.trajectory]
        ty = [s[2] for s in node.trajectory]
        t0 = ts[0]
        cs = [(t - t0) for t in ts]
        ax.plot(
            tx, ty, color="#ff3366", linewidth=1.5, alpha=0.7, zorder=5
        )
        sc = ax.scatter(
            tx,
            ty,
            c=cs,
            cmap="plasma",
            s=18,
            zorder=6,
            edgecolors="#fff",
            linewidths=0.3,
            label=f"trajectory ({len(node.trajectory)} samples)",
        )
        cbar = plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
        cbar.set_label("t since start (s)", color="#ccc")
        cbar.ax.yaxis.set_tick_params(color="#ccc")
        plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color="#ccc")

        # Start triangle
        x0, y0, yaw0 = tx[0], ty[0], node.trajectory[0][3]
        L = 1.5
        nose = (x0 + L * math.cos(yaw0), y0 + L * math.sin(yaw0))
        left = (x0 + 0.6 * math.cos(yaw0 + 2.5), y0 + 0.6 * math.sin(yaw0 + 2.5))
        right = (x0 + 0.6 * math.cos(yaw0 - 2.5), y0 + 0.6 * math.sin(yaw0 - 2.5))
        ax.add_patch(
            plt.Polygon([nose, left, right], color="#00ff7f", zorder=7, label="start")
        )

        # End triangle
        x1, y1, yaw1 = tx[-1], ty[-1], node.trajectory[-1][3]
        nose = (x1 + L * math.cos(yaw1), y1 + L * math.sin(yaw1))
        left = (x1 + 0.6 * math.cos(yaw1 + 2.5), y1 + 0.6 * math.sin(yaw1 + 2.5))
        right = (x1 + 0.6 * math.cos(yaw1 - 2.5), y1 + 0.6 * math.sin(yaw1 - 2.5))
        ax.add_patch(
            plt.Polygon([nose, left, right], color="#ff3366", zorder=7, label="end")
        )

    ax.axhline(0, color="#333", lw=0.5, zorder=1)
    ax.axvline(0, color="#333", lw=0.5, zorder=1)

    # Force the axis bounds to the union of (track cones, trajectory) with
    # generous padding so off-track excursions can never be cropped out.
    xs, ys = [], []
    for c in cones:
        xs.append(c["x"]); ys.append(c["y"])
    if node.trajectory:
        for s in node.trajectory:
            xs.append(s[1]); ys.append(s[2])
    if xs:
        pad = 5.0
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)

    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", color="#ccc")
    ax.set_ylabel("North (m)", color="#ccc")
    ax.tick_params(colors="#ccc")
    for spine in ax.spines.values():
        spine.set_color("#444")
    ax.grid(alpha=0.15, color="#666")
    ax.legend(
        loc="upper right",
        facecolor="#222",
        edgecolor="#444",
        labelcolor="#ccc",
        fontsize=9,
    )
    ax.set_title(
        "Vehicle trajectory + GT track + detected cones + latest path (ENU)",
        color="#ffb81c",
        fontsize=13,
        pad=15,
    )

    fig.savefig(out_path, facecolor=fig.get_facecolor(), dpi=120, bbox_inches="tight")
    print(f"Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="seconds to record (default 60). Ctrl-C stops early.",
    )
    parser.add_argument(
        "--out",
        default="/tmp/trajectory.png",
        help="output PNG path (default /tmp/trajectory.png)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryRecorder()

    deadline = time.time() + args.duration
    interrupted = {"flag": False}

    def _handle_sigint(signum, frame):
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)

    print(f"Recording for {args.duration:.0f} s (Ctrl-C to stop early)...")
    while time.time() < deadline and not interrupted["flag"]:
        rclpy.spin_once(node, timeout_sec=0.1)

    render_plot(node, args.out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
