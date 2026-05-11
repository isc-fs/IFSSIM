"""Standalone launch for the DV-PC-resident EKF state estimator.

Brings up the two new nodes that replace `OdometryFilter`'s role for
the autonomy stack (issue #447 Phase 1):

  rpm_to_odom            — std_msgs/Float32 → nav_msgs/Odometry adapter,
                           wraps /motor_rpm into /wheel_odom for
                           ekf_localization_node to consume.

  ekf_filter_node        — robot_localization's ekf_localization_node,
                           configured by ekf_dvpc.yaml. Fuses
                           /imu (predict) + /wheel_odom (vx update).
                           Publishes /odometry/filtered and the
                           `odom → base_link` TF.

Topic-rename convention: the EKF publishes /odometry/filtered by
default; we remap to /odom_ekf so it lives side-by-side with the
existing sim_supervisor's /odom for offline diff during validation.
Once we're satisfied with the EKF behaviour we'll flip the remap so
the EKF *is* /odom and disable sim_supervisor's filter.

Run alone for testing:

    docker compose exec -T dv_pipeline_stack bash -lc \\
      'source /opt/ros/humble/setup.bash && \\
       source /dv_pipeline_stack_ws/install/setup.bash && \\
       ros2 launch /dv_pipeline_stack_ws/dvpc_ekf.launch.py'

Then play a bag in another terminal and watch:

    ros2 topic echo /odom_ekf --field twist.twist
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # The launch file lives in /dv_pipeline_stack_ws/ at container
    # runtime (copied by the entrypoint or refresh-bridge.sh). The
    # yaml + adapter live alongside it.
    here = os.path.dirname(__file__) or "/dv_pipeline_stack_ws"
    ekf_yaml = os.path.join(here, "ekf_dvpc.yaml")
    rpm_to_odom_script = os.path.join(here, "rpm_to_odom.py")

    return LaunchDescription([

        # RPM → Odometry adapter — runs as a plain python3 process
        # since it's a single self-contained script (no setup.py
        # entry-point machinery needed for Phase 1 validation).
        ExecuteProcess(
            cmd=["python3", rpm_to_odom_script],
            name="rpm_to_odom",
            output="screen",
        ),

        # robot_localization EKF.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_yaml],
            # Remap the default output topic onto /odom_ekf so it
            # doesn't collide with sim_supervisor's /odom during the
            # parallel-run validation period.
            remappings=[
                ("/odometry/filtered", "/odom_ekf"),
            ],
        ),
    ])
