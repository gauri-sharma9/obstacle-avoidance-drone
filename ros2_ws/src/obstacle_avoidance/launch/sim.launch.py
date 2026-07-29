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
        "/world/demo_world/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
    ],
    output="screen",
)

    enable_camera_stream = ExecuteProcess(
    cmd=[
        "gz", "topic", "-t",
        "/world/demo_world/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming",
        "-m", "gz.msgs.Boolean",
        "-p", "data: true",
    ],
    output="screen",
)

    detector = Node(
        package="obstacle_avoidance",
        executable="obstacle_detector",
        output="screen",
    )

    radar = Node(
    package="obstacle_avoidance",
    executable="radar_node",
    output="screen",
)

    camera_detector = Node(
    package="obstacle_avoidance",
    executable="camera_detector_node",
    output="screen",
)

    return LaunchDescription([
    gazebo,
    bridge,
    TimerAction(period=4.0, actions=[enable_camera_stream]),
    TimerAction(period=5.0, actions=[detector]),
    TimerAction(period=5.0, actions=[radar]),
    TimerAction(period=6.0, actions=[camera_detector]),
])