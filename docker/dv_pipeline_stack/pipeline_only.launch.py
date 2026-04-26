"""
Pipeline-only launch — starts SLAM, path planning, and control nodes.
The bridge and foxglove are assumed to already be running (bridge.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

REMAP_LIDAR  = ('/fsds/lidar/Lidar1',       '/lidar/Lidar1')
REMAP_ODOM   = ('/fsds/testing_only/odom',  '/testing_only/odom')  # control's lateral velocity (TODO PR #4: replace with motor-RPM)
REMAP_GSS    = ('/fsds/gss',                '/gss')                 # control's longitudinal velocity (TODO PR #4: replace with motor-RPM)
REMAP_CMD    = ('/fsds/control_command',    '/control_command')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        # GLIM (LiDAR-IMU SLAM). CPU-only odometry estimation per the
        # docs/glim_integration.md plan. Consumes /lidar/Lidar1 + /imu,
        # publishes map → odom → base_link via TF. Single source of truth
        # for vehicle localization after step 5 deleted Odometria_perfecta.
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_ros',
            output='screen',
            parameters=[{
                'config_path': '/dv_pipeline_stack_ws/glim_config',
            }],
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
