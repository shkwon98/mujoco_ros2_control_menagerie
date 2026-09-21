#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathSubstitution(FindPackageShare(
                    "agibot_g2_mujoco_description"))
                / "urdf"
                / "g2_mujoco.urdf.xacro",
                " headless:=",
                headless,
            ]
        ),
        value_type=str,
    )
    controllers = (
        PathSubstitution(FindPackageShare("agibot_g2_mujoco_description"))
        / "config"
        / "ros2_control"
        / "g2_controllers.yaml"
    )
    # Keep related spawners together and reference event targets by controller name.
    spawners = {
        name: Node(
            package="controller_manager",
            executable="spawner",
            namespace="/",
            arguments=[
                name,
                "--controller-ros-args",
                "--ros-args -r __ns:=/control/body",
            ],
        )
        for name in (
            "arm_left_controller",
            "arm_right_controller",
            "torso_controller",
            "head_controller",
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without its GUI.",
            ),
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                choices=["true", "false"],
                description="Start the Nav2 control pipeline",
            ),
            DeclareLaunchArgument(
                "use_rqt",
                default_value="false",
                choices=["true", "false"],
                description="Launch RQT joint trajectory controller for the robot.",
            ),
            # Open RQT only after its selected controller is active.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawners["arm_left_controller"],
                    on_exit=lambda event, _: (
                        [
                            Node(
                                package="rqt_gui",
                                executable="rqt_gui",
                                namespace="/",
                                arguments=[
                                    "--perspective-file",
                                    PathSubstitution(
                                        FindPackageShare(
                                            "agibot_g2_mujoco_bringup")
                                    )
                                    / "config"
                                    / "agibot_g2_mujoco.perspective",
                                    "--force-discover",
                                ],
                                parameters=[{"use_sim_time": True}],
                                remappings=[
                                    ("robot_description", "/robot_description"),
                                    *[
                                        (
                                            f"/{name}/{topic}",
                                            f"/control/body/{name}/{topic}",
                                        )
                                        for name in (
                                            "arm_left_controller",
                                            "arm_right_controller",
                                            "torso_controller",
                                            "head_controller",
                                        )
                                        for topic in (
                                            "controller_state",
                                            "joint_trajectory",
                                        )
                                    ],
                                ],
                                output="screen",
                            )
                        ]
                        if event.returncode == 0
                        else []
                    ),
                ),
                condition=IfCondition(LaunchConfiguration("use_rqt")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
                remappings=[
                    ("joint_states", "/sensors/proprio/body/joint_states")],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="/",
                parameters=[controllers],
                remappings=[("robot_description", "/robot_description")],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="/",
                arguments=[
                    "body_joint_state_broadcaster",
                    "--controller-ros-args",
                    "--ros-args -r __ns:=/control/body --remap joint_states:=/sensors/proprio/body/joint_states",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="/",
                arguments=[
                    "swerve_drive_controller",
                    "--controller-ros-args",
                    "--ros-args -r __ns:=/control/body --remap /control/body/odom:=/odom",
                ],
            ),
            *spawners.values(),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("agibot_g2_mujoco_nav"))
                / "launch"
                / "navigation.launch.py",
                condition=IfCondition(LaunchConfiguration("use_navigation")),
            ),
        ]
    )
