#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROBOT_MODELS = {
    "a": {
        "base_model": "a",
        "initial_positions": "rby1a.yaml",
        "controllers": "rby1a_controllers.yaml",
        "mobile_base_controller_package": "diff_drive_controller",
        "has_mobile_base": True,
        "has_wuji_hands": False,
    },
    "m": {
        "base_model": "m",
        "initial_positions": "rby1m.yaml",
        "controllers": "rby1m_controllers.yaml",
        "mobile_base_controller_package": "mecanum_drive_controller",
        "has_mobile_base": True,
        "has_wuji_hands": False,
    },
    "a_wuji": {
        "base_model": "a",
        "initial_positions": "rby1a_wuji.yaml",
        "controllers": "rby1a_wuji_controllers.yaml",
        "mobile_base_controller_package": "diff_drive_controller",
        "has_mobile_base": True,
        "has_wuji_hands": True,
    },
    "m_wuji": {
        "base_model": "m",
        "initial_positions": "rby1m_wuji.yaml",
        "controllers": "rby1m_wuji_controllers.yaml",
        "mobile_base_controller_package": "mecanum_drive_controller",
        "has_mobile_base": True,
        "has_wuji_hands": True,
    },
}


def make_robot_description(xacro_file, robot_model, robot_version, initial_positions_file):
    return {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    xacro_file,
                    " robot_model:=",
                    robot_model,
                    " robot_version:=",
                    robot_version,
                    " initial_positions_file:=",
                    initial_positions_file,
                ]
            ),
            value_type=str,
        )
    }


def make_hand_robot_description(xacro_file):
    return {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    xacro_file,
                ]
            ),
            value_type=str,
        )
    }


def make_joint_state_broadcaster_spawner(
    controller_name,
    joint_states_topic,
    dynamic_joint_states_topic=None,
    log_level="info",
):
    arguments = [
        controller_name,
        "--controller-ros-args",
        f"--ros-args --remap joint_states:={joint_states_topic}",
    ]
    if dynamic_joint_states_topic:
        arguments.extend(
            [
                "--controller-ros-args",
                f"--ros-args --remap dynamic_joint_states:={dynamic_joint_states_topic}",
            ]
        )
    arguments.extend(["--ros-args", "--log-level", log_level])

    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/control/body",
        output="screen",
        arguments=arguments,
    )


def make_controller_spawner(
    controller_name,
    command_topic,
    action_topic=None,
    controller_namespace=None,
    joint_states_topic="/sensors/proprio/body/joint_states",
    log_level="info",
):
    arguments = [
        controller_name,
        "--controller-ros-args",
        f"--ros-args --remap ~/joint_states:={joint_states_topic}",
    ]
    if controller_namespace:
        arguments.extend(
            [
                "--controller-ros-args",
                f"--ros-args -r __ns:={controller_namespace}",
            ]
        )
    arguments.extend(
        [
            "--controller-ros-args",
            f"--ros-args --remap ~/joint_trajectory:={command_topic}",
        ]
    )
    if action_topic:
        arguments.extend(
            [
                "--controller-ros-args",
                f"--ros-args --remap ~/follow_joint_trajectory:={action_topic}",
            ]
        )
    arguments.extend(["--ros-args", "--log-level", log_level])

    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/control/body",
        output="screen",
        arguments=arguments,
    )


def has_package(package_name):
    try:
        get_package_prefix(package_name)
    except PackageNotFoundError:
        return False
    return True


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_version = LaunchConfiguration("robot_version")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    log_level = LaunchConfiguration("log_level")

    robot_model_value = robot_model.perform(context)
    robot_version_value = robot_version.perform(context)
    controllers_yaml_value = controllers_yaml.perform(context)
    model_config = ROBOT_MODELS[robot_model_value]

    description_share = get_package_share_directory("rby1_mujoco_description")
    if controllers_yaml_value == "auto":
        controllers_yaml_value = os.path.join(
            description_share,
            "config",
            "ros2_control",
            model_config["controllers"],
        )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rby1_mujoco_description"), "urdf", "rby1.urdf.xacro"]
    )
    initial_positions_file = os.path.join(
        description_share,
        "config",
        "initial_positions",
        model_config["initial_positions"],
    )
    body_initial_positions_file = os.path.join(
        description_share,
        "config",
        "initial_positions",
        f"rby1{model_config['base_model']}.yaml",
    )

    control_robot_description = make_robot_description(
        xacro_file,
        robot_model_value,
        robot_version_value,
        initial_positions_file,
    )
    body_robot_description = make_robot_description(
        xacro_file,
        model_config["base_model"],
        robot_version_value,
        body_initial_positions_file,
    )

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="/control/body",
            parameters=[controllers_yaml_value],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("robot_description", "/control/body/robot_description"),
            ],
        ),
        Node(
            package="rby1_mujoco_bringup",
            executable="robot_description_publisher.py",
            namespace="/control/body",
            parameters=[control_robot_description],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="/sensors/proprio/body",
            parameters=[body_robot_description],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[
                ("robot_description", "/sensors/proprio/body/robot_description"),
                ("joint_states", "/sensors/proprio/body/joint_states"),
            ],
        ),
    ]

    if model_config["has_wuji_hands"]:
        left_hand_xacro_file = PathJoinSubstitution(
            [
                FindPackageShare("rby1_mujoco_description"),
                "urdf",
                "wuji_hand",
                "left_with_docking.urdf.xacro",
            ]
        )
        right_hand_xacro_file = PathJoinSubstitution(
            [
                FindPackageShare("rby1_mujoco_description"),
                "urdf",
                "wuji_hand",
                "right_with_docking.urdf.xacro",
            ]
        )
        nodes.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace="/sensors/proprio/hand_left",
                    parameters=[make_hand_robot_description(left_hand_xacro_file)],
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ("robot_description", "/control/hand_left/robot_description"),
                        ("joint_states", "/sensors/proprio/hand_left/joint_states"),
                    ],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace="/sensors/proprio/hand_right",
                    parameters=[make_hand_robot_description(right_hand_xacro_file)],
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ("robot_description", "/control/hand_right/robot_description"),
                        ("joint_states", "/sensors/proprio/hand_right/joint_states"),
                    ],
                ),
                make_joint_state_broadcaster_spawner(
                    "body_joint_state_broadcaster",
                    "/sensors/proprio/body/joint_states",
                    "/sensors/proprio/body/dynamic_joint_states",
                    log_level,
                ),
                make_joint_state_broadcaster_spawner(
                    "hand_left_joint_state_broadcaster",
                    "/sensors/proprio/hand_left/joint_states",
                    "/sensors/proprio/hand_left/dynamic_joint_states",
                    log_level,
                ),
                make_joint_state_broadcaster_spawner(
                    "hand_right_joint_state_broadcaster",
                    "/sensors/proprio/hand_right/joint_states",
                    "/sensors/proprio/hand_right/dynamic_joint_states",
                    log_level,
                ),
            ]
        )
    else:
        nodes.append(
            make_joint_state_broadcaster_spawner(
                "joint_state_broadcaster",
                "/sensors/proprio/body/joint_states",
                log_level=log_level,
            )
        )

    nodes.extend(
        [
            make_controller_spawner(
                "arm_right_controller",
                "/control/body/arm_right_controller/joint_trajectory",
                log_level=log_level,
            ),
            make_controller_spawner(
                "arm_left_controller",
                "/control/body/arm_left_controller/joint_trajectory",
                log_level=log_level,
            ),
            make_controller_spawner(
                "torso_controller",
                "/control/body/torso_controller/joint_trajectory",
                log_level=log_level,
            ),
            make_controller_spawner(
                "head_controller",
                "/control/body/head_controller/joint_trajectory",
                log_level=log_level,
            ),
        ]
    )

    if model_config["has_mobile_base"] and has_package(
        model_config["mobile_base_controller_package"]
    ):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="/control/body",
                output="screen",
                arguments=[
                    "mobile_base_controller",
                    "--controller-ros-args",
                    "--ros-args --remap ~/cmd_vel:=/cmd_vel --remap ~/reference:=/cmd_vel",
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
            )
        )
    elif model_config["has_mobile_base"]:
        nodes.append(
            LogInfo(
                msg=(
                    "Skipping mobile_base_controller: package "
                    f"'{model_config['mobile_base_controller_package']}' is not installed. "
                    "Install it to enable /cmd_vel mobile-base control."
                )
            )
        )

    if model_config["has_wuji_hands"]:
        nodes.extend(
            [
                make_controller_spawner(
                    "hand_left_controller",
                    "/control/hand_left/hand_left_controller/joint_trajectory",
                    "/control/hand_left/hand_left_controller/follow_joint_trajectory",
                    "/control/hand_left",
                    "/sensors/proprio/hand_left/joint_states",
                    log_level,
                ),
                make_controller_spawner(
                    "hand_right_controller",
                    "/control/hand_right/hand_right_controller/joint_trajectory",
                    "/control/hand_right/hand_right_controller/follow_joint_trajectory",
                    "/control/hand_right",
                    "/sensors/proprio/hand_right/joint_states",
                    log_level,
                ),
            ]
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="a",
                choices=list(ROBOT_MODELS.keys()),
                description="RBY1 MuJoCo model",
            ),
            DeclareLaunchArgument(
                "robot_version",
                default_value="v1.2",
                description="RBY1 model version",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value="auto",
                description="Controller configuration YAML, or 'auto' to select by robot_model",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="ROS log level",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
