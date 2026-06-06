"""Section 3 — Gazebo Classic bring-up for the contact-aware wiping demo.

DEPRECATED: the Section 3 deliverable is the MoveIt + software spring-damper model
(`moveit_wiping.py`); the secondary physics demo uses Gazebo Harmonic
(`gz_wiping.launch.py`). This Classic/ODE path is kept for reference only — Gazebo
Classic is EOL (Jan 2025) and the streaming controller doesn't press/sense cleanly
against the Classic position-JTC. See WIPING_NOTES.md ("Why not a physics engine").

Starts Gazebo with the wiping scene (countertop/faucet/mirror), spawns the Piper
with a wrist force/torque sensor + wiping pad, and loads:
  - joint_state_broadcaster
  - arm_controller            (JointTrajectoryController, position)
  - tcp_fts_broadcaster       (force_torque_sensor_broadcaster -> WrenchStamped)

The custom wiping controller node is launched separately (section3 script) so the
sim can settle first.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("gr_wiping_control").find("gr_wiping_control")
    xacro_file = os.path.join(pkg, "description", "piper_wiping.xacro")
    world_file = os.path.join(pkg, "worlds", "wiping.world")

    gui = LaunchConfiguration("gui")
    # Process the xacro in Python and STRIP COMMENTS before publishing it. Humble's
    # gazebo_ros2_control plugin re-passes robot_description to the controller_manager
    # as a `--param robot_description:=<xml>` rule; comment text (with ':', ':=', quotes,
    # newlines) breaks rcl's param parser -> the resource_manager loads NO hardware and
    # no controllers can start. Removing comments fixes it (matches the reference impl).
    import xacro as _xacro
    import re as _re
    _doc = _xacro.process_file(xacro_file)
    _xml = _re.sub(r"<!--.*?-->", "", _doc.toxml(), flags=_re.DOTALL)
    robot_description = _xml

    # `gazebo` = gzserver+gzclient (GUI); `gzserver` alone = headless
    gz_bin = PythonExpression(["'gazebo' if '", gui, "' == 'true' else 'gzserver'"])
    gazebo = ExecuteProcess(
        cmd=[gz_bin, "--verbose", world_file,
             "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True,
                     "robot_description": robot_description,
                     "publish_frequency": 30.0}],
    )

    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "piper", "-topic", "robot_description"],
        output="screen",
    )

    def loader(name):
        return ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "active", name],
            output="screen",
        )

    jsb = loader("joint_state_broadcaster")
    arm = loader("arm_controller")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="true=Gazebo GUI, false=headless gzserver"),
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ])
