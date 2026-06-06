"""Section 3 — Gazebo Harmonic (gz-sim) bring-up via gz_ros2_control.

Starts gz-sim with the wiping scene, spawns the Piper (with a wrist force/torque
sensor that gz_ros2_control exposes as a state interface), bridges /clock, and
loads joint_state_broadcaster, arm_controller, and tcp_fts_broadcaster (which
publishes the wrist wrench on /wrist_ft).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("gr_wiping_control").find("gr_wiping_control")
    xacro_file = os.path.join(pkg, "description", "piper_wiping_gz.xacro")
    world_file = os.path.join(pkg, "worlds", "wiping_gz.sdf")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    ros_gz_sim = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    # gui:=false -> headless server (-s); gui:=true -> server + GUI client
    from launch.substitutions import PythonExpression
    headless = PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": [world_file, " -r -v1", headless]}.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-name", "piper", "-topic", "robot_description", "-z", "0.0"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    def loader(name):
        return ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "active", name],
            output="screen",
        )

    ctrl = LaunchConfiguration("controller")   # arm_controller (graze) or admittance_controller
    jsb = loader("joint_state_broadcaster")
    fts = loader("tcp_fts_broadcaster")
    main = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", ctrl],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("controller", default_value="admittance_controller"),
        gz, rsp, clock_bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[fts])),
        RegisterEventHandler(OnProcessExit(target_action=fts, on_exit=[main])),
    ])
