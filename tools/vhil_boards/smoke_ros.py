#!/usr/bin/env python3
"""Smoke test of the ROS side against running emulated boards (no sim needed).

Runs in the pipeline stack's image, on its own ROS domain, with this checkout's
sim_supervisor on PYTHONPATH and IFSSIM_VHIL_BOARDS set:

    docker run --rm --network host -e ROS_DOMAIN_ID=77 -e IFSSIM_VHIL_BOARDS=127.0.0.1:47800 \\
      -v $PWD/ros2/src/sim_supervisor:/mine -v $PWD/tools/vhil_boards:/smoke \\
      --entrypoint bash ifssim-dv_pipeline_stack:latest -c \\
      'source /opt/ros/humble/setup.bash && source /dv_pipeline_stack_ws/install/setup.bash && \\
       PYTHONPATH=/mine python3 /smoke/smoke_ros.py'

It plays Mission Control and the operator panel (/sim/mission, /sim/intent, /dv/status,
/ctrl/cmd), as the benchmark does, and prints what the node publishes: the emulated
uDV's AS state on /assi/state and what it relays to the sim bridge on /fsds/control_command.
"""
import subprocess
import sys
import time

import rclpy
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, UInt8

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def lifecycle(node: str, transition: str) -> None:
    subprocess.run(["ros2", "lifecycle", "set", node, transition], check=True, capture_output=True, timeout=60)


def main() -> int:
    proc = subprocess.Popen(
        [sys.executable, "-c", "from sim_supervisor.sim_supervisor_node import main; main()"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(4)
        lifecycle("/sim_supervisor_node", "configure")
        lifecycle("/sim_supervisor_node", "activate")
        rclpy.init()
        n = Node("smoke_mission_control")
        state = {"assi": None, "mission": None, "thr": None}
        n.create_subscription(UInt8, "/assi/state", lambda m: state.update(assi=m.data), BEST)
        n.create_subscription(Int32, "/ami/mission", lambda m: state.update(mission=m.data), BEST)
        n.create_subscription(ControlCommand, "/fsds/control_command", lambda m: state.update(thr=round(m.throttle, 3)), 10)
        mission = n.create_publisher(Int32, "/sim/mission", LATCHED)
        intent = n.create_publisher(UInt8, "/sim/intent", LATCHED)
        estop = n.create_publisher(Bool, "/sim/estop", LATCHED)
        dv = n.create_publisher(UInt8, "/dv/status", LATCHED)
        cmd = n.create_publisher(Twist, "/ctrl/cmd", BEST)
        rpm = n.create_publisher(Float32, "/motor_rpm", BEST)
        status, throttle, t0, last, driving_at = 2, 0.0, time.time(), 0.0, None
        estop.publish(Bool(data=False))
        mission.publish(Int32(data=1))
        intent.publish(UInt8(data=1))
        go = False
        while time.time() - t0 < 120:
            rclpy.spin_once(n, timeout_sec=0.025)
            dv.publish(UInt8(data=status))                       # the pipeline's heartbeat
            t = Twist(); t.linear.x = throttle; cmd.publish(t)    # its command stream
            rpm.publish(Float32(data=300.0 if state["assi"] == 3 else 0.0))
            if state["assi"] == 2 and not go:
                intent.publish(UInt8(data=2)); go = True          # the operator's GO
            if state["assi"] == 3 and driving_at is None:
                driving_at, status, throttle = time.time(), 3, 0.4
            if time.time() - last > 4:
                last = time.time(); print(f"{time.time() - t0:5.1f}s", state, flush=True)
            if driving_at and time.time() - driving_at > 12:
                break
        ok = state["assi"] == 3 and (state["thr"] or 0) > 0
        print("RESULT", "OK: the emulated uDV drives and the car follows the emulated ECU's torque" if ok else "FAILED", state)
        return 0 if ok else 1
    finally:
        proc.terminate()


if __name__ == "__main__":
    sys.exit(main())
