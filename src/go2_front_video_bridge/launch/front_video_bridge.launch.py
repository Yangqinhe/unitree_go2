from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("go2_front_video_bridge"), "config", "front_video_bridge.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="go2_front_video_bridge",
                executable="front_video_bridge_node",
                name="go2_front_video_bridge",
                output="screen",
                parameters=[config],
            )
        ]
    )
