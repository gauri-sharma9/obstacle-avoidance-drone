"""sim.launch.py — Full obstacle-avoidance simulation launch

Launch order (staggered with TimerAction):
  0 s  — Gazebo Ignition with demo_world.sdf
  3 s  — ros_gz_bridge   (clock, camera, lidar/points topics)
  7 s  — lidar_sector_node
  7 s  — camera_detector_node
  9 s  — radar_node
  11 s — fusion_node
  13 s — avoidance_node        (logs commands to /avoidance/command)
  15 s — mavlink_obstacle_node (streams OBSTACLE_DISTANCE to ArduPilot)
  25 s — mission_commander_node (arm + takeoff + waypoint after SITL ready)

Environment:
  IGN_GAZEBO_RESOURCE_PATH  — points to simulation/models
  IGN_GAZEBO_SYSTEM_PLUGIN_PATH — points to ardupilot_gazebo build dir
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # ── Path resolution ────────────────────────────────────────────────────────
    # launch file is at:
    #   <repo>/ros2_ws/src/obstacle_avoidance/launch/sim.launch.py
    # walk up to repo root
    launch_dir  = os.path.dirname(os.path.abspath(__file__))   # launch/
    pkg_dir     = os.path.dirname(launch_dir)                  # obstacle_avoidance/
    src_dir     = os.path.dirname(pkg_dir)                     # src/
    ws_dir      = os.path.dirname(src_dir)                     # ros2_ws/
    repo_dir    = os.path.dirname(ws_dir)                      # repo root

    # Fallback to ~/obstacle-avoidance-drone if repo_dir doesn't look right
    if not os.path.isdir(os.path.join(repo_dir, 'simulation')):
        repo_dir = os.path.expanduser('~/obstacle-avoidance-drone')

    world_path  = os.path.join(repo_dir, 'simulation', 'worlds', 'demo_world.sdf')
    models_dir  = os.path.join(repo_dir, 'simulation', 'models')
    plugin_dir  = os.path.expanduser('~/ardupilot/ardupilot_gazebo/build')

    # ── Environment variables ──────────────────────────────────────────────────
    env = dict(os.environ)

    def _prepend(var: str, value: str) -> str:
        existing = env.get(var, '')
        return f'{value}:{existing}' if existing else value

    env['IGN_GAZEBO_RESOURCE_PATH']       = _prepend('IGN_GAZEBO_RESOURCE_PATH',   models_dir)
    env['GZ_SIM_RESOURCE_PATH']           = _prepend('GZ_SIM_RESOURCE_PATH',       models_dir)
    env['SDF_PATH']                       = _prepend('SDF_PATH',                    models_dir)
    env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH']  = _prepend('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_dir)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']      = _prepend('GZ_SIM_SYSTEM_PLUGIN_PATH',  plugin_dir)

    # ── Gazebo ─────────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--render-engine', 'ogre', '-r', world_path],
        output='screen',
        additional_env=env,
    )

    # ── ros_gz_bridge ──────────────────────────────────────────────────────────
    # Topic mapping: <gz_topic>@<ros_type>[<gz_type>
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera (color image from Gazebo → ROS)
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # LiDAR — Gazebo sensor topic is 'lidar' (set in model.sdf).
            # ros_gz_bridge uses the Gazebo topic name directly:
            '/lidar@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        output='screen',
        additional_env=env,
    )

    # ── ROS2 nodes ─────────────────────────────────────────────────────────────
    # The ros_gz_bridge publishes LiDAR on '/lidar' (Gazebo's topic name).
    # Our node code subscribes to '/lidar/points', so we remap here.
    _lidar_remap = [('/lidar/points', '/lidar')]

    lidar_sector      = Node(package='obstacle_avoidance', executable='lidar_sector_node',      output='screen', remappings=_lidar_remap)
    camera_detector   = Node(package='obstacle_avoidance', executable='camera_detector_node',   output='screen')
    radar             = Node(package='obstacle_avoidance', executable='radar_node',             output='screen')
    fusion            = Node(package='obstacle_avoidance', executable='fusion_node',            output='screen')
    avoidance         = Node(package='obstacle_avoidance', executable='avoidance_node',         output='screen')
    mavlink_obstacle  = Node(package='obstacle_avoidance', executable='mavlink_obstacle_node',  output='screen')
    mission_commander = Node(package='obstacle_avoidance', executable='mission_commander_node', output='screen')

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0,  actions=[bridge]),
        TimerAction(period=7.0,  actions=[lidar_sector, camera_detector]),
        TimerAction(period=9.0,  actions=[radar]),
        TimerAction(period=11.0, actions=[fusion]),
        TimerAction(period=13.0, actions=[avoidance]),
        TimerAction(period=15.0, actions=[mavlink_obstacle]),
        TimerAction(period=25.0, actions=[mission_commander]),
    ])