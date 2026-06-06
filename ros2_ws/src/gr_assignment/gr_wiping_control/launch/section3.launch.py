"""Section 3 bringup. Starts the wiping controller — assumes Isaac Sim is
running with the ROS 2 Bridge enabled and publishing the topics listed in
config/wiping.yaml.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("gr_wiping_control"), "config", "wiping.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="gr_wiping_control",
                executable="wiping_controller",
                name="wiping_controller",
                output="screen",
                parameters=[params],
            ),
        ]
    )
