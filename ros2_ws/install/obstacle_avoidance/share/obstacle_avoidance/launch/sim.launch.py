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

    spawn_drone = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-world", "demo_world",
        "-name", "iris",
        "-file",
        os.path.expanduser(
            "~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf"
        ),
        "-x", "0",
        "-y", "0",
        "-z", "0.2",
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

    TimerAction(
        period=3.0,
        actions=[spawn_drone],
    ),

    bridge,

    TimerAction(
        period=5.0,
        actions=[detector],
    ),
])