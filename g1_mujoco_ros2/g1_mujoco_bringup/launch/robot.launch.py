#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mujoco_model_file = LaunchConfiguration("mujoco_model_file")
    log_level = LaunchConfiguration("log_level")

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("g1_mujoco_description"),
            "urdf",
            "g1_mujoco.urdf.xacro",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " initial_positions_file:=",
            initial_positions_file,
            " mujoco_model_file:=",
            mujoco_model_file,
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
    leg_controller_spawner = make_controller_spawner(
        "leg_controller",
        "/control/body/leg_controller/joint_trajectory",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("g1_mujoco_description"),
                        "config",
                        "ros2_control",
                        "g1_controllers.yaml",
                    ]
                ),
                description="Controller configuration YAML",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("g1_mujoco_description"),
                        "config",
                        "initial_positions.yaml",
                    ]
                ),
                description="Initial joint positions YAML",
            ),
            DeclareLaunchArgument(
                "mujoco_model_file",
                default_value="g1_29dof_fixed.xml",
                choices=["g1_29dof_fixed.xml", "g1_29dof.xml"],
                description="MuJoCo model file. Use the floating model only with a balance controller.",
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
            leg_controller_spawner,
        ]
    )
