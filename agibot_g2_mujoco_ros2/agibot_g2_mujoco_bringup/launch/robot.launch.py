#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("agibot_g2_mujoco_description"), "urdf", "g2_mujoco.urdf.xacro"]
                ),
                " headless:=",
                headless,
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare("agibot_g2_mujoco_description"), "config", "ros2_control", "g2_controllers.yaml"]
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            arguments=[name],
        )
        for name in (
            "arm_left_controller",
            "arm_right_controller",
            "torso_controller",
            "head_controller",
        )
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false", description="Run MuJoCo without its GUI."),
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
                remappings=[("joint_states", "/sensors/proprio/body/joint_states")],
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
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="/control/body",
                arguments=[
                    "body_joint_state_broadcaster",
                    "--controller-ros-args",
                    "--ros-args --remap joint_states:=/sensors/proprio/body/joint_states",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="/control/body",
                arguments=[
                    "swerve_drive_controller",
                    "--controller-ros-args",
                    "--ros-args --remap /control/body/odom:=/odom",
                ],
            ),
            *spawners,
        ]
    )
