#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOT_MODELS = {
    "vx300s": {
        "xacro": "mobile_aloha_mujoco.urdf.xacro",
        "controllers": "mobile_aloha_controllers.yaml",
        "leader_controllers": (),
    },
    "piper": {
        "xacro": "mobile_aloha_piper_mujoco.urdf.xacro",
        "controllers": "mobile_aloha_piper_controllers.yaml",
        "leader_controllers": (
            "leader_left_controller",
            "leader_right_controller",
            "leader_gripper_left_controller",
            "leader_gripper_right_controller",
        ),
    },
}


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


def launch_setup(context):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    headless = LaunchConfiguration("headless")
    model = ROBOT_MODELS[robot_model]
    description_share = get_package_share_directory("mobile_aloha_mujoco_description")

    description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    os.path.join(description_share, "urdf", model["xacro"]),
                    " headless:=",
                    headless,
                ]
            ),
            value_type=str,
        )
    }
    controllers = os.path.join(
        description_share, "config", "ros2_control", model["controllers"]
    )

    nodes = [
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
            parameters=[description],
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
    nodes.extend(controller_spawner(name) for name in model["leader_controllers"])
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="vx300s",
                choices=list(ROBOT_MODELS),
                description="Mobile ALOHA arm model",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its GUI.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
