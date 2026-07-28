import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    world_path = os.path.join(
        os.path.expanduser("~"),
        "obstacle-avoidance-drone",
        "simulation",
        "worlds",
        "demo_world.sdf",
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "--render-engine",
            "ogre",
            "-r",
            world_path,
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    detector = Node(
        package="obstacle_avoidance",
        executable="obstacle_detector",
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        bridge,
        TimerAction(
            period=5.0,
            actions=[detector],
        ),
    ])