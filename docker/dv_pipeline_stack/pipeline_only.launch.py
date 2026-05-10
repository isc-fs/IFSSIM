"""
Pipeline-only launch — DV pipeline alignment series.

Brings up everything except the bridge + foxglove (those come from
bridge.launch.py running separately under entrypoint.sh):

  Always-active:
    robot_state_publisher  — coche_urdf TF tree
    joint_state_publisher  — default-zero joint states feeding RSP

  Mission management (LifecycleNodes, auto-configured+activated):
    mode_manager_node, mission_control_node, sim_supervisor_node

  Autonomy (LifecycleNodes, parked in `unconfigured` until
  mode_manager drives them through configure→activate via change_state
  fan-out triggered by mission_control_node.StartMission):
    cone_detection_node, slam_node, path_planning_node, control_node

The DV_DISABLE_CONTROL env switch from the pre-lifecycle layout has
been retired — leaving control_node parked in `unconfigured` is the
new way to keep it dormant. Once it's launched, mode_manager owns
whether it goes active or stays put.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.msg import Transition

# ---------------------------------------------------------------------
# Topic remappings — same as pipeline.launch.py. See that file's
# header for the rationale.
# ---------------------------------------------------------------------
REMAP_LIDAR = ("/fsds/lidar/Lidar1", "/lidar/Lidar1")
REMAP_GSS   = ("/fsds/gss",          "/gss")
REMAP_IMU   = ("/fsds/imu",          "/imu")
REMAP_GT    = ("/fsds/testing_only/odom", "/testing_only/odom")
REMAP_RPM   = ("/fsds/motor_rpm",    "/motor_rpm")
REMAP_CMD   = ("/fsds/control_command", "/control_command")


def _auto_active(package: str, executable: str, name: str,
                 remappings=None) -> list:
    """LifecycleNode + configure event + activate handler — auto-drives
    the node from unconfigured to active at launch start."""
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace="",
        output="screen",
        remappings=remappings or [],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return [node, activate, configure]


def _autonomy_lifecycle(package: str, executable: str, name: str,
                        remappings=None) -> LifecycleNode:
    """LifecycleNode parked in `unconfigured` — mode_manager drives it."""
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace="",
        output="screen",
        remappings=remappings or [],
    )


def generate_launch_description() -> LaunchDescription:
    coche_urdf_share = get_package_share_directory("coche_urdf")
    rsp_launch = os.path.join(
        coche_urdf_share, "launch", "robot_state_publisher.launch.py")

    actions = [
        DeclareLaunchArgument("host",         default_value="host.docker.internal"),
        DeclareLaunchArgument("port",         default_value="41451"),
        DeclareLaunchArgument("mission_name", default_value="trackdrive"),
        DeclareLaunchArgument("track_name",   default_value="A"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
        ),
    ]

    # Mission management (auto-active) — see pipeline.launch.py for
    # the order rationale.
    actions += _auto_active("mode_manager", "mode_manager_node", "mode_manager_node")
    actions += _auto_active("mission_control", "mission_control_node", "mission_control_node")
    # sim_supervisor needs /imu + /motor_rpm remapped onto /fsds/* so
    # its OdometryFilter sees the bridge's sensor stream.
    actions += _auto_active(
        "sim_supervisor", "sim_supervisor_node", "sim_supervisor_node",
        remappings=[REMAP_IMU, REMAP_RPM],
    )

    # Autonomy lifecycle nodes (unconfigured)
    actions.append(_autonomy_lifecycle(
        "cone_detection", "cone_detection_node", "cone_detection_node",
        remappings=[REMAP_LIDAR],
    ))
    actions.append(_autonomy_lifecycle(
        "cone_slam", "slam_node", "slam_node",
        remappings=[REMAP_IMU, REMAP_RPM, REMAP_GT],
    ))
    actions.append(_autonomy_lifecycle(
        "path_planning", "path_planning_node", "path_planning_node",
    ))
    actions.append(_autonomy_lifecycle(
        "control", "control_node", "control_node",
        remappings=[REMAP_CMD],
    ))

    return LaunchDescription(actions)
