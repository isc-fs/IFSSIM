"""Cone-frame diagnostic for fix/21.

Listens to /lidar/Lidar1, /Conos_raw, /Conos_Orange and the odom→fsds/FSCar
transform. Prints one summary line per second:

    [t=3.2s] car=(11.14, 0.04) | lidar_n=... lidar_xrange=[-1.80, 50.20]
           | cones_n=12 cones_x=[2.10, 48.50]
           | orange_n=2 orange_x=[3.05, 7.10]

Run inside the ros_stack container with the pipeline up:

    docker exec -it ros_stack bash -lc \\
      "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \\
       python3 /ws/tools/diagnostics/cone_frame_probe.py"

Interpretation:
- If `cones_x` tracks car motion (decreases as car=x grows) → cones ARE
  car-relative; the bug is elsewhere.
- If `cones_x` stays pinned to the same range regardless of car motion
  → the points feeding Cone_Detection are world-fixed (bridge or sim bug).
- `lidar_xrange` gives the same answer for the raw PointCloud2 input.
"""
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
)
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

QOS_LATEST = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Probe(Node):
    def __init__(self):
        super().__init__("cone_frame_probe")
        self.lidar_n = 0
        self.lidar_xrange = (math.nan, math.nan)
        self.close_stats = None
        self.cones_raw = []
        self.cones_orange = []

        self.create_subscription(PointCloud2, "/lidar/Lidar1", self._lidar_cb, QOS_LATEST)
        self.create_subscription(MarkerArray, "/Conos_raw", self._raw_cb, 10)
        self.create_subscription(MarkerArray, "/Conos_Orange", self._orange_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t0 = time.monotonic()
        self.create_timer(1.0, self._report)

    def _lidar_cb(self, msg: PointCloud2) -> None:
        fpp = msg.point_step // 4
        n = msg.width * msg.height
        if n == 0:
            self.lidar_n = 0
            self.lidar_xrange = (math.nan, math.nan)
            self.close_stats = None
            return
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(n, fpp)
        xs = raw[:, 0]
        ys = raw[:, 1]
        zs = raw[:, 2]
        self.lidar_n = n
        self.lidar_xrange = (float(xs.min()), float(xs.max()))
        # Analyze the closest forward returns (x in 1.8-2.5m band).
        # These are the candidates for self-detection; knowing their z/y
        # spread tells us if they're ground (z ≈ 0), nose (z ≈ 0.1-0.3),
        # or something else.
        close_mask = (xs > 1.8) & (xs < 2.5)
        if close_mask.any():
            cxs, cys, czs = xs[close_mask], ys[close_mask], zs[close_mask]
            self.close_stats = (
                int(close_mask.sum()),
                float(cxs.min()), float(cxs.max()),
                float(cys.min()), float(cys.max()),
                float(czs.min()), float(czs.max()),
            )
        else:
            self.close_stats = None

    def _raw_cb(self, msg: MarkerArray) -> None:
        self.cones_raw = [
            (m.pose.position.x, m.pose.position.y)
            for m in msg.markers if m.action != 3
        ]

    def _orange_cb(self, msg: MarkerArray) -> None:
        self.cones_orange = [
            (m.pose.position.x, m.pose.position.y)
            for m in msg.markers if m.action != 3
        ]

    def _car_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("odom", "fsds/FSCar", rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return (math.nan, math.nan)

    def _report(self) -> None:
        t = time.monotonic() - self.t0
        cx, cy = self._car_pose()
        lxa, lxb = self.lidar_xrange
        if self.cones_raw:
            xs = [x for x, _ in self.cones_raw]
            raw_range = f"[{min(xs):.2f}, {max(xs):.2f}]"
        else:
            raw_range = "[]"
        if self.cones_orange:
            oxs = [x for x, _ in self.cones_orange]
            orange_range = f"[{min(oxs):.2f}, {max(oxs):.2f}]"
        else:
            orange_range = "[]"
        if self.close_stats:
            n_c, _xa, _xb, ya, yb, za, zb = self.close_stats
            close = f"close_n={n_c} y=[{ya:.2f},{yb:.2f}] z=[{za:.2f},{zb:.2f}]"
        else:
            close = "close_n=0"
        print(
            f"[t={t:5.1f}s] car=({cx:.2f}, {cy:.2f}) | "
            f"lidar_n={self.lidar_n} lidar_xrange=[{lxa:.2f}, {lxb:.2f}] | "
            f"{close} | "
            f"raw_n={len(self.cones_raw)} raw_x={raw_range} | "
            f"orange_n={len(self.cones_orange)} orange_x={orange_range}",
            flush=True,
        )


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
