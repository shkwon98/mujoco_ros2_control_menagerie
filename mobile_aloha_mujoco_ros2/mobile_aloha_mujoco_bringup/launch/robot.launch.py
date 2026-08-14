#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def controller_spawner(name: str, remappings: str = "") -> Node:
    arguments = [name]
    if remappings:
        arguments += ["--controller-ros-args", f"--ros-args {remappings}"]
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/control/body",
        arguments=arguments,
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration("headless")
    description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("mobile_aloha_mujoco_description"),
                        "urdf",
                        "mobile_aloha_mujoco.urdf.xacro",
                    ]
                ),
                " headless:=",
                headless,
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution(
        [
            FindPackageShare("mobile_aloha_mujoco_description"),
            "config",
            "ros2_control",
            "mobile_aloha_controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its GUI.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="/sensors/proprio/body",
                parameters=[{"robot_description": description}],
                remappings=[
                    ("robot_description", "/robot_description"),
                    ("joint_states", "/sensors/proprio/body/joint_states"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="/control/body",
                parameters=[controllers],
                remappings=[("robot_description", "/robot_description")],
                output="screen",
            ),
            controller_spawner(
                "joint_state_broadcaster",
                "--remap joint_states:=/sensors/proprio/body/joint_states",
            ),
            controller_spawner("arm_left_controller"),
            controller_spawner("arm_right_controller"),
            controller_spawner(
                "gripper_left_controller",
                "--remap ~/joint_trajectory:=/control/hand_left/gripper_controller/joint_trajectory",
            ),
            controller_spawner(
                "gripper_right_controller",
                "--remap ~/joint_trajectory:=/control/hand_right/gripper_controller/joint_trajectory",
            ),
            controller_spawner(
                "mobile_base_controller",
                "--remap ~/cmd_vel:=/cmd_vel --remap ~/odom:=/odom",
            ),
        ]
    )
