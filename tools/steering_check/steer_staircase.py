#!/usr/bin/env python3
"""On-stands steering STAIRCASE driver — commands a sequence of KNOWN road-wheel
angles and holds each so the operator can read a physical angle gauge on the wheel.

Publishes ONLY /ctrl/cmd (geometry_msgs/Twist): angular.z = normalised steering,
linear.x = 0 (NO throttle — steering only, safe on stands). δ_road -> norm =
δ_road / max_steer_deg, clamped to [-1, 1].

Records nothing itself — run a `ros2 bag record` alongside (at least /ctrl/cmd
and /steering/feedback) and analyse with steer_verify.py (auto-detects the holds).

ACTUATION REQUIREMENT (read this): the uDV only forwards /ctrl/cmd steering to
DV-STEERING when the drive path is active (mission in DRIVING). So either:
  (a) run this while the car is in an AS state that actuates the pipeline steer
      channel, with the pipeline's own controller NOT publishing /ctrl/cmd
      (else two publishers fight), OR
  (b) if DV-STEERING / uDV exposes a direct pit/inspection angle command, drive
      that instead — the same verify.py analysis applies to whatever bag results.
Confirm the path on your rig; this script is the generic /ctrl/cmd option.

Usage (on the car):
  ros2 bag record -o steer_cal /ctrl/cmd /steering/feedback /steering_angle &
  python3 steer_staircase.py --angles 0,5,10,15,18,0,-5,-10,-15,-18,0 --hold 5 --max-steer 18.2
  # read the wheel gauge at each printed HOLD; record gauge.csv (hold_index,physical_deg)
"""
import argparse, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", default="0,5,10,15,18,0,-5,-10,-15,-18,0",
                    help="road-wheel angles (deg), comma-separated")
    ap.add_argument("--hold", type=float, default=5.0, help="seconds to hold each angle")
    ap.add_argument("--max-steer", type=float, default=18.2, help="deg at norm=1 (== control_node max_steer_deg)")
    ap.add_argument("--rate", type=float, default=40.0, help="publish Hz (keep >= 20 so uDV sees it fresh)")
    args = ap.parse_args()

    angles = [float(x) for x in args.angles.split(",")]
    for a in angles:
        if abs(a) > args.max_steer + 1e-6:
            sys.exit(f"angle {a} exceeds max_steer {args.max_steer} (norm would clip) — reduce it")

    rclpy.init()
    node = Node("steer_staircase")
    pub = node.create_publisher(Twist, "/ctrl/cmd", 10)
    dt = 1.0 / args.rate

    def hold(norm, secs):
        t_end = time.monotonic() + secs
        m = Twist(); m.linear.x = 0.0; m.angular.z = float(norm)
        while rclpy.ok() and time.monotonic() < t_end:
            pub.publish(m); time.sleep(dt)

    print(f"# max_steer={args.max_steer}deg  hold={args.hold}s  rate={args.rate}Hz")
    print("# record a bag (/ctrl/cmd /steering/feedback /steering_angle) alongside this.")
    print("# at each HOLD: read the physical wheel gauge, note it against hold_index.\n")
    try:
        for i, deg in enumerate(angles):
            norm = max(-1.0, min(1.0, deg / args.max_steer))
            print(f"HOLD {i:2d}: commanded {deg:+6.1f} deg road-wheel (norm={norm:+.3f})  "
                  f"-> hold {args.hold}s, READ GAUGE, record gauge.csv line: {i},<physical_deg>")
            hold(norm, args.hold)
        # settle to centre
        print("\ndone -> centring (norm=0)")
        hold(0.0, 1.0)
    except KeyboardInterrupt:
        print("\naborted -> centring")
        hold(0.0, 0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
