from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median

import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from common import load_csv_centerline, make_run_dir, write_csv, write_json
from report_html import write_run_report


@dataclass
class Sample:
    t_s: float
    cross_track_err_m: float
    heading_err_rad: float
    speed_mps: float
    throttle: float
    steering: float
    brake: float


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ControlBenchmarkNode(Node):
    def __init__(
        self,
        centerline_source: str,
        centerline_csv: str,
        centerline_topic: str,
        output_dir: str,
        strategy: str,
    ) -> None:
        super().__init__("sim_control_benchmark_node")
        self._source = centerline_source
        self._output_dir = output_dir
        self._strategy = strategy
        self._samples: list[Sample] = []
        self._latest_cmd = ControlCommand()
        self._centerline: list[tuple[float, float]] = []

        if self._source == "csv":
            self._centerline = load_csv_centerline(centerline_csv)
        elif self._source == "topic":
            self.create_subscription(NavPath, centerline_topic, self._on_gt_path, 10)

        self.create_subscription(ControlCommand, "/ctrl/cmd_internal", self._on_cmd, 10)
        # Match ifssim_bridge sensor_qos (BEST_EFFORT) — same as track_driver.
        gt_odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, "/testing_only/odom", self._on_gt_odom, gt_odom_qos,
        )

    def _on_gt_path(self, msg: NavPath) -> None:
        self._centerline = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_cmd(self, msg: ControlCommand) -> None:
        self._latest_cmd = msg

    def _on_gt_odom(self, msg: Odometry) -> None:
        if len(self._centerline) < 2:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        speed = float(msg.twist.twist.linear.x)

        nearest_idx = min(
            range(len(self._centerline)),
            key=lambda i: (self._centerline[i][0] - x) ** 2 + (self._centerline[i][1] - y) ** 2,
        )
        nx, ny = self._centerline[nearest_idx]
        tx, ty = self._centerline[(nearest_idx + 1) % len(self._centerline)]
        path_yaw = math.atan2(ty - ny, tx - nx)
        heading_err = math.atan2(math.sin(yaw - path_yaw), math.cos(yaw - path_yaw))
        cte = math.hypot(x - nx, y - ny)

        self._samples.append(
            Sample(
                t_s=self.get_clock().now().nanoseconds * 1e-9,
                cross_track_err_m=cte,
                heading_err_rad=heading_err,
                speed_mps=speed,
                throttle=float(self._latest_cmd.throttle),
                steering=float(self._latest_cmd.steering),
                brake=float(self._latest_cmd.brake),
            )
        )

    def write_results(self) -> Path:
        run_dir = make_run_dir(self._output_dir, "control", self._strategy)
        rows = [
            {
                "t_s": s.t_s,
                "cross_track_err_m": s.cross_track_err_m,
                "heading_err_rad": s.heading_err_rad,
                "speed_mps": s.speed_mps,
                "throttle": s.throttle,
                "steering": s.steering,
                "brake": s.brake,
            }
            for s in self._samples
        ]
        write_csv(run_dir / "results.csv", rows)
        n = max(1, len(self._samples))
        cte = [s.cross_track_err_m for s in self._samples]
        heading = [abs(s.heading_err_rad) for s in self._samples]
        summary = {
            "module": "control",
            "strategy": self._strategy,
            "samples": len(self._samples),
            "mean_cross_track_err_m": sum(cte) / n,
            "median_cross_track_err_m": median(cte) if cte else 0.0,
            "max_cross_track_err_m": max(cte, default=0.0),
            "mean_heading_err_rad": sum(heading) / n,
            "median_heading_err_rad": median(heading) if heading else 0.0,
            "csv": str(run_dir / "results.csv"),
        }
        write_run_report(summary, run_dir, Path(self._output_dir))
        write_json(run_dir / "results.json", summary)
        return run_dir / "report.html"


def main() -> None:
    ap = argparse.ArgumentParser(description="Online control benchmark harness.")
    ap.add_argument("--centerline-source", choices=["csv", "topic"], default="csv")
    ap.add_argument("--centerline-csv", default="")
    ap.add_argument("--centerline-topic", default="/benchmark/gt_path")
    ap.add_argument("--results-root", default="tools/sim_benchmark/results")
    ap.add_argument("--strategy", default="sim_control_benchmark")
    ap.add_argument("--duration-s", type=float, default=60.0)
    args = ap.parse_args()

    if args.centerline_source == "csv" and not Path(args.centerline_csv).is_file():
        raise FileNotFoundError("--centerline-csv is required for csv source")

    rclpy.init()
    node = ControlBenchmarkNode(
        centerline_source=args.centerline_source,
        centerline_csv=args.centerline_csv,
        centerline_topic=args.centerline_topic,
        output_dir=args.results_root,
        strategy=args.strategy,
    )
    end_ns = node.get_clock().now().nanoseconds + int(args.duration_s * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end_ns:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        out = node.write_results()
        node.destroy_node()
        rclpy.shutdown()
        print(f"Wrote {out}")


if __name__ == "__main__":
    main()
