#!/usr/bin/env python3
"""
Keyboard teleop for IFSSIM — publishes fs_msgs/ControlCommand to /control_command.

  W / S  : throttle / brake
  A / D  : steer left / right
  SPACE  : full brake (emergency stop)
  Q      : quit

Run inside the container:
  docker exec -it ifssim-dv_pipeline_stack-1 python3 /dv_pipeline_stack_ws/teleop_keyboard.py
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand

THROTTLE_STEP = 0.1
STEER_STEP    = 0.1
DECAY         = 0.85   # steering snaps back when key released

HELP = """
╔══════════════════════════════╗
║   IFSSIM Keyboard Teleop     ║
╠══════════════════════════════╣
║  W        throttle +         ║
║  S        brake +            ║
║  A / D    steer left / right ║
║  SPACE    emergency stop     ║
║  Q        quit               ║
╚══════════════════════════════╝
throttle: {:.2f}  steer: {:+.2f}  brake: {:.2f}
"""


def get_key(fd, old):
    tty.setraw(fd)
    ch = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub  = node.create_publisher(ControlCommand, '/control_command', 10)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    throttle = 0.0
    steer    = 0.0
    brake    = 0.0

    print(HELP.format(throttle, steer, brake))

    try:
        while rclpy.ok():
            key = get_key(fd, old).lower()

            if key == 'q':
                break
            elif key == 'w':
                throttle = min(1.0, throttle + THROTTLE_STEP)
                brake    = 0.0
            elif key == 's':
                brake    = min(1.0, brake + THROTTLE_STEP)
                throttle = 0.0
            elif key == 'a':
                steer = max(-1.0, steer - STEER_STEP)
            elif key == 'd':
                steer = min(1.0, steer + STEER_STEP)
            elif key == ' ':
                throttle = 0.0
                steer    = 0.0
                brake    = 1.0
            else:
                # No key — decay steering back to centre
                steer    = steer * DECAY
                throttle = max(0.0, throttle - 0.02)
                brake    = 0.0

            msg          = ControlCommand()
            msg.throttle = float(throttle)
            msg.steering = float(steer)
            msg.brake    = float(brake)
            pub.publish(msg)

            sys.stdout.write('\r' + f'throttle: {throttle:.2f}  steer: {steer:+.2f}  brake: {brake:.2f}   ')
            sys.stdout.flush()

    finally:
        # Stop the car on exit
        stop = ControlCommand()
        stop.throttle = 0.0
        stop.steering = 0.0
        stop.brake    = 1.0
        pub.publish(stop)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\nStopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
