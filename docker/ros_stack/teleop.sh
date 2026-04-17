#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros_stack_ws/install/setup.bash
export AMENT_PREFIX_PATH="/ros_stack_ws/install/fs_msgs:/ros_stack_ws/install/ifssim_bridge:$AMENT_PREFIX_PATH"
exec python3 /ros_stack_ws/teleop_keyboard.py
