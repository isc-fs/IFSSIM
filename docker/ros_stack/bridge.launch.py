"""
Bridge-only launch — starts ifssim_bridge without the autonomous pipeline.
Use this to verify connectivity and topic flow, or for manual/keyboard driving.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            }],
        ),
    ])
