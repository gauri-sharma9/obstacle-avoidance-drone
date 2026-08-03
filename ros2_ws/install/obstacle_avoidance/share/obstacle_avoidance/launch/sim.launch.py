import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    world_path = os.path.join(
        os.path.expanduser("~"), "obstacle-avoidance-drone",
        "simulation", "worlds", "demo_world.sdf",
    )

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--render-engine", "ogre", "-r", world_path],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    camera_detector = Node(package="obstacle_avoidance", executable="camera_detector_node", output="screen")
    radar = Node(package="obstacle_avoidance", executable="radar_node", output="screen")
    fusion = Node(package="obstacle_avoidance", executable="fusion_node", output="screen")
    avoidance = Node(package="obstacle_avoidance", executable="avoidance_node", output="screen")
    mavlink_command = Node(package="obstacle_avoidance", executable="mavlink_command_node", output="screen")

    return LaunchDescription([
        gazebo,
        bridge,
        TimerAction(period=5.0, actions=[camera_detector]),
        TimerAction(period=15.0, actions=[radar]),
        TimerAction(period=17.0, actions=[fusion]),
        TimerAction(period=19.0, actions=[avoidance]),
        TimerAction(period=21.0, actions=[mavlink_command]),
    ])