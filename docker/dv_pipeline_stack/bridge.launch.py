"""
Bridge-only launch — starts ifssim_bridge without the autonomous pipeline.
Use this to verify connectivity and topic flow, or for manual/keyboard driving.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # LIDAR_TRANSPORT — "udp" (default, production) or "tcp" / "uds"
    # (soft-deprecated as of #321 follow-up; kept for parity testing only,
    # bridge will WARN when either is selected). UDP bypasses macOS Docker
    # Desktop's TCP loopback throughput cap (~7 MB/s) and was the binding
    # win — TCP/UDS are no longer worth the wire-format-multiplication
    # cost.
    lidar_transport = os.environ.get("LIDAR_TRANSPORT", "udp").lower()

    return LaunchDescription([
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        Node(
            package='ifssim_bridge',
            executable='ifssim_bridge',
            name='ifssim_bridge',
            output='screen',
            parameters=[{
                'host':             LaunchConfiguration('host'),
                'port':             LaunchConfiguration('port'),
                'mission_name':     LaunchConfiguration('mission_name'),
                'track_name':       LaunchConfiguration('track_name'),
                'competition_mode': False,
                'lidar_transport':  lidar_transport,
            }],
        ),

        # Foxglove Studio WebSocket bridge — connect at ws://localhost:8765
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000,
                'use_sim_time': False,
            }],
        ),
    ])
