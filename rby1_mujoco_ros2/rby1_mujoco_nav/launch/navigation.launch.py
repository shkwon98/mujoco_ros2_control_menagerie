#!/usr/bin/env python3
"""Declare this robot's Nav2 controllers and lifecycle manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    nav = PathSubstitution(FindPackageShare("rby1_mujoco_nav"))
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    params_file = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration("params_file"), ""),
        nav / "config" / ["rby1", LaunchConfiguration("robot_model"), ".yaml"],
        LaunchConfiguration("params_file"),
    )
    configured_parameters = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            root_key=namespace,
            convert_types=True,
        ),
        allow_substs=True,
    )
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument(
                "robot_model",
                default_value="a",
                choices=["a", "m"],
                description="RBY1 base model used to select the Nav2 parameters",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Nav2 node namespace"
            ),
            DeclareLaunchArgument(
                "params_file", default_value="", description="Nav2 parameter YAML"
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Stamped velocity command output topic",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="ROS log level",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                namespace=namespace,
                parameters=[configured_parameters],
                remappings=[("cmd_vel", "cmd_vel_nav")],
                ros_arguments=["--log-level", log_level],
                output="screen",
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                namespace=namespace,
                parameters=[configured_parameters],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", LaunchConfiguration("cmd_vel_topic")),
                ],
                ros_arguments=["--log-level", log_level],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=namespace,
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["controller_server", "velocity_smoother"],
                    }
                ],
                ros_arguments=["--log-level", log_level],
                output="screen",
            ),
        ]
    )
