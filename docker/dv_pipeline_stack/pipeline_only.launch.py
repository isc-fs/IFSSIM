"""
Pipeline-only launch — starts SLAM, path planning, and control nodes.
The bridge and foxglove are assumed to already be running (bridge.launch.py).

Set DV_DISABLE_CONTROL=true (env var read at launch time) to skip the
autonomous control node. Useful when validating SLAM in isolation or
when keyboard-driving the car for SLAM motion sanity checks.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

REMAP_LIDAR  = ('/fsds/lidar/Lidar1',       '/lidar/Lidar1')
REMAP_ODOM   = ('/fsds/testing_only/odom',  '/testing_only/odom')  # control's lateral velocity (TODO: drop in favor of /cone_slam/state)
REMAP_GSS    = ('/fsds/gss',                '/gss')                 # control's longitudinal velocity (TODO: drop in favor of /motor_rpm-derived velocity)
REMAP_CMD    = ('/fsds/control_command',    '/control_command')


def generate_launch_description():
    nodes = [
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        # Cone-graph SLAM. Subscribes /imu, /Conos_raw, /motor_rpm.
        # Publishes odom → base_link TF, /cone_slam/state (Odometry),
        # /Conos (world-frame cone map with persistent landmark IDs).
        # Replaces the legacy fast_LIMO + Publicar_Mapa pair: fast_LIMO
        # never tracked our cone-only LiDAR scenes through turns
        # (cascaded mid-drive); Publicar_Mapa was its downstream cone
        # accumulator and is unnecessary now that cone_slam owns /Conos
        # directly.
        Node(
            package='cone_slam',
            executable='cone_graph_slam',
            name='cone_graph_slam',
            output='screen',
        ),

        Node(
            package='slam',
            executable='Cone_Detection',
            name='Cone_Detection',
            output='screen',
            remappings=[REMAP_LIDAR],
        ),
        Node(
            package='path_planning',
            executable='Plan_Path',
            name='path_planning',
            output='screen',
        ),
    ]

    if os.environ.get('DV_DISABLE_CONTROL', 'false').lower() != 'true':
        nodes.append(Node(
            package='control',
            executable='Control',
            name='control',
            output='screen',
            prefix=["bash -c 'sleep 20; $0 $@' "],
            remappings=[REMAP_GSS, REMAP_ODOM, REMAP_CMD],
        ))

    return LaunchDescription(nodes)
