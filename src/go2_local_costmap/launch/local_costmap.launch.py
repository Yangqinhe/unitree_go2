from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("go2_local_costmap"), "config", "local_costmap.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="go2_local_costmap",
                executable="local_costmap_node",
                name="go2_local_costmap",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="go2_local_costmap",
                executable="target_tracker_node",
                name="go2_target_tracker",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="go2_local_costmap",
                executable="obstacle_marker_node",
                name="go2_obstacle_marker",
                output="screen",
                parameters=[config],
            ),
        ]
    )
