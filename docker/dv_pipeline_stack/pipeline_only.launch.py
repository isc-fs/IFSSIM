"""
Pipeline-only launch — starts SLAM, path planning, and control nodes.
The bridge and foxglove are assumed to already be running (bridge.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

REMAP_LIDAR  = ('/fsds/lidar/Lidar1',       '/lidar/Lidar1')
REMAP_ODOM   = ('/fsds/testing_only/odom',  '/testing_only/odom')
REMAP_TRACK  = ('/fsds/testing_only/track', '/testing_only/track')
REMAP_GSS    = ('/fsds/gss',                '/gss')
REMAP_CMD    = ('/fsds/control_command',    '/control_command')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        Node(
            package='odometria',
            executable='Odometria_perfecta',
            name='Odometria_perfecta',
            output='screen',
            remappings=[REMAP_ODOM],
        ),
        Node(
            package='slam',
            executable='Cone_Detection',
            name='Cone_Detection',
            output='screen',
            remappings=[REMAP_LIDAR],
        ),
        Node(
            package='slam',
            executable='Publicar_Mapa',
            name='Publicar_Mapa',
            output='screen',
        ),
        Node(
            package='slam',
            executable='Publicar_Track',
            name='Publicar_Track',
            output='screen',
            remappings=[REMAP_TRACK],
        ),
        Node(
            package='path_planning',
            executable='Plan_Path',
            name='path_planning',
            output='screen',
        ),
        Node(
            package='control',
            executable='Control',
            name='control',
            output='screen',
            prefix=["bash -c 'sleep 20; $0 $@' "],
            remappings=[REMAP_GSS, REMAP_ODOM, REMAP_CMD],
        ),
    ])
