"""
IFSSIM full pipeline launch file — thin wrapper over the bringup
package's full_pipeline.launch.py.

Pre-refactor (≤2026-05-13) this file held the entire node graph
inline. It moved into the `bringup` ROS package
(pipeline/bringup/launch/full_pipeline.launch.py) so the same node
graph can be reused by sim_pipeline (developer-local, no bridge)
and car_pipeline (real-car, no sim_supervisor) without copy-paste.

This file stays at /dv_pipeline_stack_ws/pipeline.launch.py so the
container entrypoint (docker/dv_pipeline_stack/entrypoint.sh)
keeps working without a path change. All it does now is forward the
four launch arguments the entrypoint sets — host / port /
mission_name / track_name — into the installed bringup launch file.

To edit the actual node graph, open
pipeline/bringup/launch/full_pipeline.launch.py.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Declare the same args at the wrapper level so callers can pass
    # them on the wrapper's command line; we forward them into the
    # included bringup launch file via launch_arguments below.
    args = [
        DeclareLaunchArgument("host",         default_value="host.docker.internal"),
        DeclareLaunchArgument("port",         default_value="41451"),
        DeclareLaunchArgument("mission_name", default_value="trackdrive"),
        DeclareLaunchArgument("track_name",   default_value="A"),
        # Sim container runs on the bridge's /clock. Forwarded so the
        # entrypoint can override (e.g. =false to debug against wall time).
        DeclareLaunchArgument("use_sim_time",  default_value="true"),
    ]

    full_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bringup"),
                "launch",
                "full_pipeline.launch.py",
            )
        ),
        launch_arguments={
            "host":         LaunchConfiguration("host"),
            "port":         LaunchConfiguration("port"),
            "mission_name": LaunchConfiguration("mission_name"),
            "track_name":   LaunchConfiguration("track_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # The sim's pipeline view (Esc during an autonomous run): what the
    # pipeline sees, streamed to the sim to draw over the track. Sim-side,
    # so it lives here rather than in the pipeline's bringup.
    # DV_VIZ_RELAY=false skips it: a headless run (the GPU box's sim check)
    # has nobody to show it to, and it costs about half a core there.
    viz_relay = Node(
        condition=IfCondition(EnvironmentVariable("DV_VIZ_RELAY", default_value="true")),
        package="sim_supervisor",
        executable="viz_relay",
        name="viz_relay",
        output="screen",
        # The stack's sources live in a named volume that START refreshes
        # (docker compose cp): on a stack that predates the relay its module
        # arrives only then, so it retries until it is there.
        respawn=True,
        respawn_delay=10.0,
        parameters=[{
            "host":         LaunchConfiguration("host"),
            "port":         LaunchConfiguration("port"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([*args, full_pipeline, viz_relay])
