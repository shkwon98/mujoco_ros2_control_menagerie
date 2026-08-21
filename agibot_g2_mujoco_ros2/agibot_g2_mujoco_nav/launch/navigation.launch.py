#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    configured_parameters = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            param_rewrites={},
            root_key=namespace,
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("agibot_g2_mujoco_nav"),
                     "config", "nav2.yaml"]
                ),
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                namespace=namespace,
                output="screen",
                parameters=[configured_parameters],
                remappings=[("cmd_vel", "cmd_vel_nav")],
                ros_arguments=["--log-level", "warn"],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                namespace=namespace,
                output="screen",
                parameters=[configured_parameters],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", LaunchConfiguration("cmd_vel_topic")),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["controller_server", "velocity_smoother"],
                    }
                ],
            ),
        ]
    )
