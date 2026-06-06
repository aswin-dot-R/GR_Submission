"""Section 2 bringup: runs the coverage planner once. Assumes move_group is
running (Section 1 launch already does that).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("gr_coverage"), "config", "coverage.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="gr_coverage",
                executable="coverage_planner",
                name="coverage_planner",
                output="screen",
                parameters=[params],
            ),
        ]
    )
