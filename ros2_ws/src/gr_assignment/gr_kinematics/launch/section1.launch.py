"""Section 1 bringup: IK service + scene loader. Assumes move_group is
already running (launch piper_no_gripper_moveit/demo.launch.py first).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ik_params = PathJoinSubstitution(
        [FindPackageShare("gr_kinematics"), "config", "ik.yaml"]
    )
    scene_params = PathJoinSubstitution(
        [FindPackageShare("gr_scene"), "config", "scene.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="gr_kinematics",
                executable="ik_service",
                name="gr_ik_service",
                output="screen",
                parameters=[ik_params],
            ),
            Node(
                package="gr_scene",
                executable="scene_loader",
                name="scene_loader",
                output="screen",
                parameters=[scene_params],
            ),
        ]
    )
