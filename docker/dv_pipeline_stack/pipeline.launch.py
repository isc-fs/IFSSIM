"""
IFSSIM full pipeline launch file.

Starts the IFSSIM ROS2 bridge + the full IFS07-DV driverless pipeline
in a single launch. Topic remappings translate IFSSIM topic names to
the /fsds/* names expected by the IFS07-DV nodes.

Set PIPELINE_ENABLED=true to include SLAM/path_planning/control nodes.
When false (default), only the bridge and foxglove_bridge are launched.
"""

import os
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

PIPELINE_ENABLED = os.environ.get("PIPELINE_ENABLED", "false").lower() == "true"
# LiDAR transport is UDP-only as of #322 — see bridge.launch.py.

# Opt-in /lidar/Lidar1/viz subsampling for browser-based visualisers.
# 0 (default) = disabled; >=2 = every-Nth-point cloud alongside the
# full /lidar/Lidar1 feed. Autonomy stack always subscribes to the
# full cloud.
try:
    LIDAR_VIZ_DECIMATION = int(os.environ.get("LIDAR_VIZ_DECIMATION", "0"))
except ValueError:
    LIDAR_VIZ_DECIMATION = 0


def generate_launch_description():
    nodes = [

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
                'host':                  LaunchConfiguration('host'),
                'port':                  LaunchConfiguration('port'),
                'mission_name':          LaunchConfiguration('mission_name'),
                'track_name':            LaunchConfiguration('track_name'),
                'competition_mode':      False,
                'lidar_viz_decimation':  LIDAR_VIZ_DECIMATION,
            }],
        ),

        # --- Foxglove WebSocket bridge — connect Lichtblick at ws://localhost:8765 ---
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
    ]

    if PIPELINE_ENABLED:
        nodes += [

            # --- LiDAR cone detection: ground removal (RANSAC) + DBSCAN
            # clustering → raw cone observations (per-cone σ_xy on
            # marker.scale.x for cone_graph_slam) ---
            # Uses Numba JIT — compiles on first message, cache persists via volume
            Node(
                package='cone_detection',
                executable='Cone_Detection',
                name='Cone_Detection',
                output='screen',
                remappings=[REMAP_LIDAR],
            ),

            # --- Cone-graph SLAM (cone_slam): IMU + cones + motor_rpm
            # → odom→base_link TF, /cone_slam/state, /Conos (persistent
            # landmark IDs, world frame). Replaces the legacy
            # Odometria_perfecta (GT-only sim hack), Publicar_Mapa
            # (downstream of fast_LIMO's TF) and Publicar_Track
            # (debug viz only) — none ran on the real car.
            Node(
                package='cone_slam',
                executable='cone_graph_slam',
                name='cone_graph_slam',
                output='screen',
            ),

            # --- Path planning: cone map → target path ---
            Node(
                package='path_planning',
                executable='Plan_Path',
                name='path_planning',
                output='screen',
            ),

            # --- Control: path + /cone_slam/state → control command ---
            # Delayed 20s to let Numba finish compiling in Cone_Detection
            # first. Control no longer subscribes to GSS or any bridge-side
            # odom topic — it consumes /cone_slam/state for vehicle-frame
            # velocity, since the real car won't have GSS mounted and the
            # SLAM node already integrates motor RPM + IMU into the same
            # twist field we'd otherwise read from the sensor.
            Node(
                package='control',
                executable='Control',
                name='control',
                output='screen',
                prefix=["bash -c 'sleep 20; $0 $@' "],
                remappings=[REMAP_CMD],
            ),
        ]

    return LaunchDescription(nodes)
