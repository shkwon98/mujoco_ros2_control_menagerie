#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    log_level = LaunchConfiguration("log_level")
    robot_model_value = robot_model.perform(context)
    controllers_yaml_value = controllers_yaml.perform(context)

    controller_file_by_model = {
        "ffw_bg2": "ai_worker_ffw_bg2_controllers.yaml",
        "ffw_bh5": "ai_worker_ffw_bh5_controllers.yaml",
        "ffw_sg2": "ai_worker_ffw_sg2_controllers.yaml",
        "ffw_sh5": "ai_worker_ffw_sh5_controllers.yaml",
    }
    if controllers_yaml_value == "auto":
        controllers_yaml_value = os.path.join(
            get_package_share_directory("ai_worker_mujoco_description"),
            "config",
            "ros2_control",
            controller_file_by_model[robot_model_value],
        )

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("ai_worker_mujoco_description"),
            "urdf",
            "ai_worker_mujoco.urdf.xacro",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " robot_model:=",
            robot_model,
            " initial_positions_file:=",
            initial_positions_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/control/body",
        parameters=[robot_description, controllers_yaml_value],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("robot_description", "/control/body/robot_description"),
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/sensors/proprio/body",
        parameters=[robot_description],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("robot_description", "/control/body/robot_description"),
            ("joint_states", "/sensors/proprio/body/joint_states"),
        ],
    )

    def make_joint_state_broadcaster_spawner(
        controller_name,
        joint_states_topic,
        dynamic_joint_states_topic,
    ):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            output="screen",
            arguments=[
                controller_name,
                "--controller-ros-args",
                f"--ros-args --remap joint_states:={joint_states_topic}",
                "--controller-ros-args",
                f"--ros-args --remap dynamic_joint_states:={dynamic_joint_states_topic}",
                "--ros-args",
                "--log-level",
                log_level,
            ],
        )

    def make_controller_spawner(
        controller_name,
        command_topic=None,
        action_topic=None,
        controller_namespace=None,
        joint_states_topic="/sensors/proprio/body/joint_states",
    ):
        controller_args = [
            controller_name,
            "--controller-ros-args",
            f"--ros-args --remap ~/joint_states:={joint_states_topic}",
        ]
        if controller_namespace:
            controller_args.extend(
                [
                    "--controller-ros-args",
                    f"--ros-args -r __ns:={controller_namespace}",
                ]
            )
        if command_topic:
            controller_args.extend(
                [
                    "--controller-ros-args",
                    f"--ros-args --remap ~/joint_trajectory:={command_topic}",
                ]
            )
        if action_topic:
            controller_args.extend(
                [
                    "--controller-ros-args",
                    f"--ros-args --remap ~/follow_joint_trajectory:={action_topic}",
                ]
            )
        controller_args.extend(["--ros-args", "--log-level", log_level])
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            output="screen",
            arguments=controller_args,
        )

    def make_forward_controller_spawner(controller_name, command_topic):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            output="screen",
            arguments=[
                controller_name,
                "--controller-ros-args",
                f"--ros-args --remap ~/commands:={command_topic}",
                "--ros-args",
                "--log-level",
                log_level,
            ],
        )

    arm_right_controller_spawner = make_controller_spawner(
        "arm_right_controller",
        "/control/body/arm_right_controller/joint_trajectory",
        "/control/body/arm_right_controller/follow_joint_trajectory",
    )
    arm_left_controller_spawner = make_controller_spawner(
        "arm_left_controller",
        "/control/body/arm_left_controller/joint_trajectory",
        "/control/body/arm_left_controller/follow_joint_trajectory",
    )
    torso_controller_spawner = make_controller_spawner(
        "torso_controller",
        "/control/body/torso_controller/joint_trajectory",
        "/control/body/torso_controller/follow_joint_trajectory",
    )
    head_controller_spawner = make_controller_spawner(
        "head_controller",
        "/control/body/head_controller/joint_trajectory",
        "/control/body/head_controller/follow_joint_trajectory",
    )
    hand_left_controller_spawner = make_controller_spawner(
        "hand_left_controller",
        "/control/hand_left/hand_left_controller/joint_trajectory",
        "/control/hand_left/hand_left_controller/follow_joint_trajectory",
        "/control/hand_left",
        "/sensors/proprio/hand_left/joint_states",
    )
    hand_right_controller_spawner = make_controller_spawner(
        "hand_right_controller",
        "/control/hand_right/hand_right_controller/joint_trajectory",
        "/control/hand_right/hand_right_controller/follow_joint_trajectory",
        "/control/hand_right",
        "/sensors/proprio/hand_right/joint_states",
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        make_joint_state_broadcaster_spawner(
            "body_joint_state_broadcaster",
            "/sensors/proprio/body/joint_states",
            "/sensors/proprio/body/dynamic_joint_states",
        ),
        make_joint_state_broadcaster_spawner(
            "hand_left_joint_state_broadcaster",
            "/sensors/proprio/hand_left/joint_states",
            "/sensors/proprio/hand_left/dynamic_joint_states",
        ),
        make_joint_state_broadcaster_spawner(
            "hand_right_joint_state_broadcaster",
            "/sensors/proprio/hand_right/joint_states",
            "/sensors/proprio/hand_right/dynamic_joint_states",
        ),
        arm_right_controller_spawner,
        arm_left_controller_spawner,
        torso_controller_spawner,
        head_controller_spawner,
        hand_left_controller_spawner,
        hand_right_controller_spawner,
    ]

    if robot_model_value in ("ffw_sg2", "ffw_sh5"):
        nodes.extend(
            [
                make_controller_spawner(
                    "base_steer_controller",
                    "/control/body/base_steer_controller/joint_trajectory",
                    "/control/body/base_steer_controller/follow_joint_trajectory",
                ),
                make_forward_controller_spawner(
                    "base_drive_controller",
                    "/control/body/base_drive_controller/commands",
                ),
            ]
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="ffw_bg2",
                choices=[
                    "ffw_bg2",
                    "ffw_bh5",
                    "ffw_sg2",
                    "ffw_sh5",
                ],
                description="AI Worker FFW model",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value="auto",
                description="Controller configuration YAML, or 'auto' to select by robot_model",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ai_worker_mujoco_description"),
                        "config",
                        "initial_positions.yaml",
                    ]
                ),
                description="Initial joint positions YAML",
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
