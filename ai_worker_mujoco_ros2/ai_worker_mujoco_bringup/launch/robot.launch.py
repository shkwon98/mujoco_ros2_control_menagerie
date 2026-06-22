#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    log_level = LaunchConfiguration("log_level")

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
        parameters=[robot_description, controllers_yaml],
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/control/body",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-ros-args",
            "--ros-args --remap joint_states:=/sensors/proprio/body/joint_states",
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    def make_controller_spawner(controller_name, command_topic):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            output="screen",
            arguments=[
                controller_name,
                "--controller-ros-args",
                "--ros-args --remap ~/joint_states:=/sensors/proprio/body/joint_states",
                "--controller-ros-args",
                f"--ros-args --remap ~/joint_trajectory:={command_topic}",
                "--ros-args",
                "--log-level",
                log_level,
            ],
        )

    arm_right_controller_spawner = make_controller_spawner(
        "arm_right_controller",
        "/control/body/arm_right_controller/joint_trajectory",
    )
    arm_left_controller_spawner = make_controller_spawner(
        "arm_left_controller",
        "/control/body/arm_left_controller/joint_trajectory",
    )
    torso_controller_spawner = make_controller_spawner(
        "torso_controller",
        "/control/body/torso_controller/joint_trajectory",
    )

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
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ai_worker_mujoco_description"),
                        "config",
                        "ros2_control",
                        "ai_worker_controllers.yaml",
                    ]
                ),
                description="Controller configuration YAML",
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
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            arm_right_controller_spawner,
            arm_left_controller_spawner,
            torso_controller_spawner,
            make_controller_spawner(
                "head_controller",
                "/control/body/head_controller/joint_trajectory",
            ),
        ]
    )
