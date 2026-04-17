"""
IFSSIM full pipeline launch file.

Starts the IFSSIM ROS2 bridge + the full IFS07-DV driverless pipeline
in a single launch. Topic remappings translate IFSSIM topic names to
the /fsds/* names expected by the IFS07-DV nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topic remappings: IFSSIM → FSDS naming expected by pipeline nodes
REMAP_LIDAR  = ('/fsds/lidar/Lidar1',        '/lidar/Lidar1')
REMAP_ODOM   = ('/fsds/testing_only/odom',   '/testing_only/odom')
REMAP_TRACK  = ('/fsds/testing_only/track',  '/testing_only/track')
REMAP_GSS    = ('/fsds/gss',                 '/gss')
REMAP_CMD    = ('/fsds/control_command',     '/control_command')


def generate_launch_description():
    return LaunchDescription([

        # --- Launch arguments (passed through from entrypoint) ---
        DeclareLaunchArgument('host',         default_value='host.docker.internal'),
        DeclareLaunchArgument('port',         default_value='41451'),
        DeclareLaunchArgument('mission_name', default_value='trackdrive'),
        DeclareLaunchArgument('track_name',   default_value='A'),

        # --- IFSSIM bridge ---
        # Connects to UE5 over TCP/UDP, publishes all sensor topics
        Node(
            package='ifssim_bridge',
            executable='ifssim_bridge',
            name='ifssim_bridge',
            output='screen',
            parameters=[{
                'host':         LaunchConfiguration('host'),
                'port':         LaunchConfiguration('port'),
                'mission_name': LaunchConfiguration('mission_name'),
                'track_name':   LaunchConfiguration('track_name'),
                'competition_mode': False,
            }],
        ),

        # --- Odometry: /testing_only/odom → TF odom→fsds/FSCar ---
        Node(
            package='odometria',
            executable='Odometria_perfecta',
            name='Odometria_perfecta',
            output='screen',
            remappings=[REMAP_ODOM],
        ),

        # --- SLAM: LiDAR → raw cone detections ---
        # Uses Numba JIT — compiles on first message, cache persists via volume
        Node(
            package='slam',
            executable='Cone_Detection',
            name='Cone_Detection',
            output='screen',
            remappings=[REMAP_LIDAR],
        ),

        # --- SLAM: cone accumulation → persistent map ---
        Node(
            package='slam',
            executable='Publicar_Mapa',
            name='Publicar_Mapa',
            output='screen',
        ),

        # --- SLAM: publish real cone positions from simulator (debug/benchmark) ---
        Node(
            package='slam',
            executable='Publicar_Track',
            name='Publicar_Track',
            output='screen',
            remappings=[REMAP_TRACK],
        ),

        # --- Path planning: cone map → target path ---
        Node(
            package='path_planning',
            executable='Plan_Path',
            name='path_planning',
            output='screen',
        ),

        # --- Control: path + GSS → control command ---
        # Delayed 20s to let Numba finish compiling in Cone_Detection first
        Node(
            package='control',
            executable='Control',
            name='control',
            output='screen',
            prefix=["bash -c 'sleep 20; $0 $@' "],
            remappings=[REMAP_GSS, REMAP_ODOM, REMAP_CMD],
        ),

    ])
