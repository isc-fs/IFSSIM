#!/usr/bin/env python3
"""
Plot ground-truth track + detected cones (/Conos) + planned path (/Path)
+ vehicle frame, all in odom/ENU.

Designed to run inside the dv_pipeline_stack container.
Outputs /tmp/track_path_vehicle.png.
"""
import json
import math
import socket
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


class GrabState(Node):
    def __init__(self):
        super().__init__("debug_plot_grabber")
        self.path = None
        self.detected_cones = None  # list of (x, y, r, g, b)
        self.create_subscription(Path, "/Path", self._on_path, 10)
        # /Conos is published by Cone_Detection — tolerate either QoS
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(MarkerArray, "/Conos", self._on_cones, be)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

    def _on_path(self, msg):
        self.path = msg

    def _on_cones(self, msg):
        out = []
        for m in msg.markers:
            if m.action == 3:  # DELETEALL — empty frame
                out = []
                continue
            out.append((m.pose.position.x, m.pose.position.y,
                        m.color.r, m.color.g, m.color.b))
        self.detected_cones = out

    def get_vehicle_tf(self, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                return self.tf_buf.lookup_transform("odom", "fsds/FSCar", rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        return None


def main():
    rclpy.init()
    node = GrabState()

    cones = get_groundtruth_cones()
    print(f"GT cones: {len(cones)}")

    # Spin to collect everything
    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.path is not None and node.detected_cones is not None:
            break
    tf = node.get_vehicle_tf()

    print(f"Path captured: {node.path is not None and len(node.path.poses) > 0}")
    print(f"Detected cones: {0 if node.detected_cones is None else len(node.detected_cones)}")
    print(f"Vehicle TF: {tf is not None}")

    fig, ax = plt.subplots(1, 1, figsize=(13, 13))
    fig.patch.set_facecolor("#1a1a1a")
    ax.set_facecolor("#0e0e0e")

    # Ground truth cones (faint background)
    gt_colors = {0: "#ffd00040", 1: "#3b82f640", 2: "#ff6a0080"}
    gt_labels = {0: "GT yellow", 1: "GT blue", 2: "GT big_orange"}
    seen = set()
    for c in cones:
        col = gt_colors.get(c["color"], "#88888840")
        lab = gt_labels.get(c["color"]) if c["color"] not in seen else None
        seen.add(c["color"])
        size = 60 if c["color"] == 2 else 18
        ax.scatter(c["x"], c["y"], c=col, s=size, edgecolors="none",
                   label=lab, zorder=2)

    # Detected cones from /Conos (bright foreground)
    if node.detected_cones:
        det_x = [c[0] for c in node.detected_cones]
        det_y = [c[1] for c in node.detected_cones]
        det_c = [(c[2], c[3], c[4]) for c in node.detected_cones]
        ax.scatter(det_x, det_y, c=det_c, s=80, edgecolors="#fff",
                   linewidths=1.0, label=f"detected /Conos ({len(node.detected_cones)})",
                   zorder=4, marker="X")

    # Path
    if node.path is not None and node.path.poses:
        px = [p.pose.position.x for p in node.path.poses]
        py = [p.pose.position.y for p in node.path.poses]
        ax.plot(px, py, color="#00e5ff", linewidth=2.5,
                label=f"/Path ({len(px)} poses)", zorder=5,
                marker="o", markersize=4)
        if len(px) >= 2:
            ax.annotate("", xy=(px[1], py[1]), xytext=(px[0], py[0]),
                        arrowprops=dict(arrowstyle="->", color="#00e5ff", lw=3),
                        zorder=6)
        first = node.path.poses[0].pose.orientation
        path_yaw = 2 * math.atan2(first.z, first.w)
        ax.annotate("", xy=(px[0] + 3 * math.cos(path_yaw), py[0] + 3 * math.sin(path_yaw)),
                    xytext=(px[0], py[0]),
                    arrowprops=dict(arrowstyle="->", color="#00ffaa", lw=2.5),
                    zorder=6)

    # Vehicle frame
    if tf is not None:
        vx = tf.transform.translation.x
        vy = tf.transform.translation.y
        vqz = tf.transform.rotation.z
        vqw = tf.transform.rotation.w
        v_yaw = 2 * math.atan2(vqz, vqw)

        L = 1.5
        nose = (vx + L * math.cos(v_yaw), vy + L * math.sin(v_yaw))
        left = (vx + 0.6 * math.cos(v_yaw + 2.5), vy + 0.6 * math.sin(v_yaw + 2.5))
        right = (vx + 0.6 * math.cos(v_yaw - 2.5), vy + 0.6 * math.sin(v_yaw - 2.5))
        triangle = plt.Polygon([nose, left, right], color="#ff3366", zorder=7,
                               label="vehicle")
        ax.add_patch(triangle)

        ax.annotate("", xy=(vx + 5 * math.cos(v_yaw), vy + 5 * math.sin(v_yaw)),
                    xytext=(vx, vy),
                    arrowprops=dict(arrowstyle="->", color="#ff3366", lw=3),
                    zorder=7)
        ax.text(vx + 0.8, vy - 0.8, f"vehicle\n({vx:.1f},{vy:.1f}) yaw={math.degrees(v_yaw):+.0f}°",
                color="#ff3366", fontsize=10, zorder=8)

    ax.axhline(0, color="#333", lw=0.5, zorder=1)
    ax.axvline(0, color="#333", lw=0.5, zorder=1)

    ax.set_aspect("equal")
    ax.set_xlabel("East (m)", color="#ccc")
    ax.set_ylabel("North (m)", color="#ccc")
    ax.tick_params(colors="#ccc")
    for spine in ax.spines.values():
        spine.set_color("#444")
    ax.grid(alpha=0.15, color="#666")
    ax.legend(loc="upper right", facecolor="#222", edgecolor="#444",
              labelcolor="#ccc", fontsize=10)
    ax.set_title("GT track + detected cones + planned path + vehicle frame (ENU)",
                 color="#ffb81c", fontsize=13, pad=15)

    out = "/tmp/track_path_vehicle.png"
    fig.savefig(out, facecolor=fig.get_facecolor(), dpi=120, bbox_inches="tight")
    print(f"Saved: {out}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
