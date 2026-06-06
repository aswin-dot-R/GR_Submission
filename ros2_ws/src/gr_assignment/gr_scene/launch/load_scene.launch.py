"""Publishes the countertop / faucet / mirror as CollisionObjects into the
MoveIt PlanningScene. Assumes a move_group node is already running
(e.g. piper_moveit/demo.launch.py).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scene_yaml = PathJoinSubstitution([
        FindPackageShare("gr_scene"), "config", "scene.yaml"
    ])
    return LaunchDescription([
        Node(
            package="gr_scene",
            executable="scene_loader",
            name="scene_loader",
            output="screen",
            parameters=[scene_yaml],
        ),
    ])
