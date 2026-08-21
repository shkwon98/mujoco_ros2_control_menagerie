#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def launch_setup(context):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = str(
            Path(get_package_share_directory("rby1_mujoco_nav"))
            / "config"
            / f"rby1{robot_model}.yaml"
        )

    namespace = LaunchConfiguration("namespace")
    configured_parameters = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            root_key=namespace,
            convert_types=True,
        ),
        allow_substs=True,
    )

    return [
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


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="a",
                choices=["a", "m"],
                description="RBY1 base model used to select the Nav2 parameters.",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional Nav2 parameter file override.",
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            OpaqueFunction(function=launch_setup),
        ]
    )
