"""
IFSSIM full pipeline launch file — DV pipeline alignment series.

Brings everything up in a single launch:

  Always-active (plain Nodes / launch includes):
    ifssim_bridge          — UE5 ↔ ROS bridge
    foxglove_bridge        — visualisation WebSocket
    robot_state_publisher  — coche_urdf TF tree
    joint_state_publisher  — default-zero joint states feeding RSP

  Mission management (LifecycleNodes, auto-configured+activated at
  launch start so their action/service endpoints are ready):
    mode_manager_node
    mission_control_node
    sim_supervisor_node

  Autonomy (LifecycleNodes, parked in `unconfigured` until mode_manager
  drives them through configure→activate via change_state fan-out):
    cone_detection_node
    slam_node
    path_planning_node
    control_node

The PIPELINE_ENABLED env flag from the pre-lifecycle layout has been
retired — once the autonomy nodes are LifecycleNodes, "is the pipeline
running?" is a question of lifecycle state, not whether the processes
exist. mission_control_backend (step 5 of this series) will trigger
the autonomy lifecycle through StartMission → mode_manager →
change_state, replacing the old /pipeline_ctrl/enable flag-file
mechanism in entrypoint.sh.

Topic remappings translate IFSSIM-side `/fsds/*` names to the names
the pipeline nodes consume (per the integration contract in
docs/autonomy_pipeline.md §"Topics IFSSIM publishes").
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.msg import Transition

# ---------------------------------------------------------------------
# Topic remappings (IFSSIM /fsds/* surface → pipeline-side names).
# Single source of truth — every node that consumes one of these
# pulls the tuple from here so a remap change can't drift between
# nodes.
# ---------------------------------------------------------------------
REMAP_LIDAR    = ("/fsds/lidar/Lidar1", "/lidar/Lidar1")
REMAP_GSS      = ("/fsds/gss",          "/gss")
REMAP_IMU      = ("/fsds/imu",          "/imu")
REMAP_GT       = ("/fsds/testing_only/odom", "/testing_only/odom")
REMAP_RPM      = ("/fsds/motor_rpm",    "/motor_rpm")
REMAP_CMD      = ("/fsds/control_command", "/control_command")
# Phase 3 (#383) — steering + brake_pressure remaps for the
# supervisor's OdometryFilter cross-check inputs.
REMAP_STEERING = ("/fsds/steering_angle", "/steering_angle")
REMAP_BRAKE    = ("/fsds/brake_pressure", "/brake_pressure")


# Opt-in /lidar/Lidar1/viz subsampling for browser-based visualisers.
# 0 (default) = disabled; >=2 = every-Nth-point cloud alongside the
# full /lidar/Lidar1 feed. Autonomy stack always subscribes to the
# full cloud.
try:
    LIDAR_VIZ_DECIMATION = int(os.environ.get("LIDAR_VIZ_DECIMATION", "0"))
except ValueError:
    LIDAR_VIZ_DECIMATION = 0


def _auto_active(package: str, executable: str, name: str,
                 remappings=None) -> list:
    """Return [LifecycleNode, configure_event, activate_handler] so the
    named node is auto-driven from `unconfigured` to `active` at launch
    start. Used for the management trio whose action/service endpoints
    must be live before mode_manager fans out change_state to the
    autonomy nodes."""
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
    # When configure completes (state → 'inactive'), emit activate.
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
    """Return a LifecycleNode parked in 'unconfigured'. mode_manager
    drives the configure/activate transitions via change_state fan-out
    when StartMission arrives; until then the node holds no
    subscriptions and emits no traffic."""
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
        # ------------------ Launch arguments ------------------
        DeclareLaunchArgument("host",         default_value="host.docker.internal"),
        DeclareLaunchArgument("port",         default_value="41451"),
        DeclareLaunchArgument("mission_name", default_value="trackdrive"),
        DeclareLaunchArgument("track_name",   default_value="A"),

        # ------------------ Bridge + foxglove ------------------
        Node(
            package="ifssim_bridge",
            executable="ifssim_bridge",
            name="ifssim_bridge",
            output="screen",
            parameters=[{
                "host":                 LaunchConfiguration("host"),
                "port":                 LaunchConfiguration("port"),
                "mission_name":         LaunchConfiguration("mission_name"),
                "track_name":           LaunchConfiguration("track_name"),
                "competition_mode":     False,
                "lidar_viz_decimation": LIDAR_VIZ_DECIMATION,
            }],
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port":              8765,
                "address":           "0.0.0.0",
                "send_buffer_limit": 64 * 1024 * 1024,
                "use_sim_time":      False,
            }],
        ),

        # ------------------ URDF + RSP/JSP ------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
        ),
    ]

    # ------------------ Mission management (auto-active) ------------------
    # mode_manager comes up first so its activate_mode service is
    # registered before mission_control_node's StartMission handler
    # tries to call into it. sim_supervisor last so its StartMission
    # server only opens after mission_control is ready to receive its
    # downstream call.
    actions += _auto_active("mode_manager", "mode_manager_node", "mode_manager_node")
    actions += _auto_active("mission_control", "mission_control_node", "mission_control_node")
    # sim_supervisor needs /imu + /motor_rpm remapped onto /fsds/* so
    # its OdometryFilter sees the bridge's sensor stream (Phase 1 of
    # the /odom split — feat/360).
    actions += _auto_active(
        "sim_supervisor", "sim_supervisor_node", "sim_supervisor_node",
        # REMAP_CMD: the supervisor's internal name is `/fsds/control_command`
        # (matches the topic-table contract in
        # docs/autonomy_pipeline.md §"Topics the submodule publishes back"),
        # but the bridge's subscriber is bound to the unnamespaced
        # `/control_command` since the bridge does its own /fsds-prefix
        # mapping internally. Remap keeps the supervisor's published
        # name human-readable in code while still landing on the
        # bridge's actual subscription. Post-#384 the supervisor is
        # the *only* publisher of this topic.
        remappings=[REMAP_IMU, REMAP_RPM, REMAP_STEERING, REMAP_BRAKE,
                    REMAP_CMD],
    )

    # ------------------ Autonomy lifecycle nodes (unconfigured) ------------------
    # Brought up to `active` by mode_manager when StartMission arrives.
    # Order in the LaunchDescription doesn't constrain bring-up order;
    # mode_manager.AUTONOMY_LIFECYCLE_NODES owns that.
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
        # Post-#384 control_node no longer publishes /fsds/control_command
        # directly — its output flows on /ctrl/cmd_internal to
        # mission_control_node, which surfaces it via the
        # RuntimeControl action's Feedback frames for the supervisor
        # to relay onto the bridge. No bridge-facing remap needed.
        remappings=[],
    ))

    return LaunchDescription(actions)
