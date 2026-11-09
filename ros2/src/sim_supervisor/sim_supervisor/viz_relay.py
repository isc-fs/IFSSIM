"""viz_relay: what the pipeline sees, sent to the sim to draw in the world.

The sim's menu has a pipeline view (Esc during an autonomous run) that
draws, over the real track, what the pipeline believes: the cones it
detects this scan, its SLAM cone map, where SLAM puts the car, the planned
path and the cone chains the planner built it from. This node subscribes
to those topics and sends a compact summary to the sim ten times a second.

Transport: the sim's RPC port (41451, as the bridge). The node sends
``streamViz`` and then one JSON object per line:

    {"pose": [x, y, yaw] | null,       SLAM pose, map frame
     "map":  [[x, y], ...],            SLAM cone map (/Conos_full), map frame
     "det":  [[x, y], ...],            detected cones (/Conos_raw), base_link
     "path": [[x, y], ...],            planned centreline (/Path), map frame
     "left": [[x, y], ...],            planner's blue chain, map frame
     "right": [[x, y], ...]}           planner's yellow chain, map frame

While a bag replays (the sim's Bags page plays one in the stack with /testing_only/odom
remapped to /replay/odom), the recorded ground truth also goes out, 30 times
a second, on lines of its own: {"truth": [x, y, z, yaw]} (ENU, the sim's
frame). The sim moves the car along it.

The sim anchors the map frame on the car: SLAM's origin is the car's pose
when it started, while the car still stands at the start (see
FSDSPipelineView in the plugin). Sim-only, like the rest of this package.
"""

import json
import math
import socket
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

RATE_HZ = 10.0
TRUTH_HZ = 30.0
# A scan's detections older than this aren't shown (the detector stopped).
STALE_S = 1.0


def _r(v: float) -> float:
    return round(float(v), 2)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def apply_full_map(cones: dict, markers) -> None:
    """Apply one /Conos_full message (deltas, DELETEALL on keyframes) to `cones`."""
    for m in markers:
        if m.action == Marker.DELETEALL:
            cones.clear()
        elif m.action == Marker.DELETE:
            cones.pop(m.id, None)
        else:
            cones[m.id] = (_r(m.pose.position.x), _r(m.pose.position.y))


def chain_points(markers, ns: str) -> list:
    """The points of the planner debug chain `ns` (left_chain / right_chain)."""
    for m in markers:
        if m.ns == ns and m.action == Marker.ADD and m.points:
            return [[_r(p.x), _r(p.y)] for p in m.points]
    return []


class VizRelay(Node):
    def __init__(self):
        super().__init__("viz_relay")
        self.host = self.declare_parameter("host", "host.docker.internal").value
        self.port = int(self.declare_parameter("port", 41451).value)

        self.pose = None
        self.map = {}
        self.det = []
        self.det_at = 0.0
        self.path = []
        self.left = []
        self.right = []

        full_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(MarkerArray, "/Conos_full", self.on_full_map, full_qos)
        # The rest best-effort: it matches reliable and best-effort publishers
        # alike (a bag replays /testing_only/odom best-effort), and a viewer can
        # drop a message. /Conos_full stays reliable: it sends deltas.
        viz_qos = qos_profile_sensor_data
        self.create_subscription(MarkerArray, "/Conos_raw", self.on_det, viz_qos)
        self.create_subscription(Path, "/Path", self.on_path, viz_qos)
        self.create_subscription(MarkerArray, "/path_planning/debug", self.on_debug, viz_qos)
        self.create_subscription(Odometry, "/slam/pose", self.on_pose, viz_qos)
        self.truth = None
        self.truth_at = 0.0
        self.create_subscription(Odometry, "/replay/odom", self.on_truth, viz_qos)

        self.sock = None
        self.next_try = 0.0
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.create_timer(1.0 / TRUTH_HZ, self.tick_truth)
        self.get_logger().info(f"viz_relay: sending to {self.host}:{self.port} (streamViz)")

    def on_full_map(self, msg):
        apply_full_map(self.map, msg.markers)

    def on_det(self, msg):
        self.det = [[_r(m.pose.position.x), _r(m.pose.position.y)] for m in msg.markers if m.action == Marker.ADD]
        self.det_at = time.monotonic()

    def on_path(self, msg):
        self.path = [[_r(p.pose.position.x), _r(p.pose.position.y)] for p in msg.poses]

    def on_debug(self, msg):
        self.left = chain_points(msg.markers, "left_chain")
        self.right = chain_points(msg.markers, "right_chain")

    def on_pose(self, msg):
        p = msg.pose.pose
        near_origin = abs(p.position.x) + abs(p.position.y) < 0.3
        # SLAM started over (a new session, or a bag replaying): jumping back
        # to its origin, the last session's map, path and chains go.
        if near_origin and self.pose is not None and abs(self.pose[0]) + abs(self.pose[1]) > 2.0:
            self.forget()
        self.pose = [_r(p.position.x), _r(p.position.y), round(_yaw(p.orientation), 4)]

    def forget(self):
        self.map.clear()
        self.det, self.path, self.left, self.right = [], [], [], []

    def on_truth(self, msg):
        p = msg.pose.pose
        if time.monotonic() - self.truth_at > 2.0:
            self.forget()   # a replay starting: not the live session's map
            self.pose = None
        self.truth = [round(p.position.x, 3), round(p.position.y, 3), round(p.position.z, 3),
                      round(_yaw(p.orientation), 4)]
        self.truth_at = time.monotonic()

    def payload(self) -> str:
        fresh = time.monotonic() - self.det_at < STALE_S
        return json.dumps({
            "pose": self.pose,
            "map": list(self.map.values()),
            "det": self.det if fresh else [],
            "path": self.path,
            "left": self.left,
            "right": self.right,
        }, separators=(",", ":")) + "\n"

    def connect(self) -> bool:
        now = time.monotonic()
        if now < self.next_try:
            return False
        self.next_try = now + 2.0
        try:
            s = socket.create_connection((self.host, self.port), timeout=2.0)
            s.sendall(b"streamViz\n")
            if not s.recv(16).startswith(b"OK"):
                s.close()
                return False
            s.settimeout(1.0)
            self.sock = s
            self.get_logger().info("viz_relay: connected to the sim")
            return True
        except OSError:
            return False

    def tick(self):
        self.send(self.payload())

    def tick_truth(self):
        # Only while a bag replays: nothing else publishes /replay/odom.
        if self.truth is not None and time.monotonic() - self.truth_at < 0.5:
            self.send(json.dumps({"truth": self.truth}, separators=(",", ":")) + "\n")

    def send(self, line: str):
        if self.sock is None and not self.connect():
            return
        try:
            self.sock.sendall(line.encode())
        except OSError:
            # The sim went away (quit, or a new level): try again soon.
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def main():
    rclpy.init()
    node = VizRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
