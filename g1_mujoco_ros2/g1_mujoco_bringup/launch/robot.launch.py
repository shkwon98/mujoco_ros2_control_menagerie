#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mujoco_model_file = LaunchConfiguration("mujoco_model_file")
    log_level = LaunchConfiguration("log_level")

    robot_model_value = robot_model.perform(context)
    controllers_yaml_value = controllers_yaml.perform(context)
    mujoco_model_file_value = mujoco_model_file.perform(context)

    description_share = get_package_share_directory("g1_mujoco_description")
    controller_file_by_model = {
        "g1": "g1_controllers.yaml",
        "g1_with_hands": "g1_with_hands_controllers.yaml",
    }
    mujoco_model_file_by_model = {
        "g1": "scene.xml",
        "g1_with_hands": "scene_with_hands_fixed.xml",
    }

    if controllers_yaml_value == "auto":
        controllers_yaml_value = os.path.join(
            description_share,
            "config",
            "ros2_control",
            controller_file_by_model[robot_model_value],
        )

    if mujoco_model_file_value == "auto":
        mujoco_model_file_value = mujoco_model_file_by_model[robot_model_value]

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
            " robot_model:=",
            robot_model,
            " initial_positions_file:=",
            initial_positions_file,
            " mujoco_model_file:=",
            mujoco_model_file_value,
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
        command_topic,
        action_topic,
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
        controller_args.extend(
            [
                "--controller-ros-args",
                f"--ros-args --remap ~/joint_trajectory:={command_topic}",
                "--controller-ros-args",
                f"--ros-args --remap ~/follow_joint_trajectory:={action_topic}",
                "--ros-args",
                "--log-level",
                log_level,
            ]
        )
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/control/body",
            output="screen",
            arguments=controller_args,
        )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        make_joint_state_broadcaster_spawner(
            "body_joint_state_broadcaster",
            "/sensors/proprio/body/joint_states",
            "/sensors/proprio/body/dynamic_joint_states",
        ),
        make_controller_spawner(
            "arm_right_controller",
            "/control/body/arm_right_controller/joint_trajectory",
            "/control/body/arm_right_controller/follow_joint_trajectory",
        ),
        make_controller_spawner(
            "arm_left_controller",
            "/control/body/arm_left_controller/joint_trajectory",
            "/control/body/arm_left_controller/follow_joint_trajectory",
        ),
        make_controller_spawner(
            "torso_controller",
            "/control/body/torso_controller/joint_trajectory",
            "/control/body/torso_controller/follow_joint_trajectory",
        ),
        make_controller_spawner(
            "leg_controller",
            "/control/body/leg_controller/joint_trajectory",
            "/control/body/leg_controller/follow_joint_trajectory",
        ),
    ]

    if robot_model_value == "g1_with_hands":
        nodes.extend(
            [
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
                make_controller_spawner(
                    "hand_left_controller",
                    "/control/hand_left/hand_left_controller/joint_trajectory",
                    "/control/hand_left/hand_left_controller/follow_joint_trajectory",
                    "/control/hand_left",
                    "/sensors/proprio/hand_left/joint_states",
                ),
                make_controller_spawner(
                    "hand_right_controller",
                    "/control/hand_right/hand_right_controller/joint_trajectory",
                    "/control/hand_right/hand_right_controller/follow_joint_trajectory",
                    "/control/hand_right",
                    "/sensors/proprio/hand_right/joint_states",
                ),
            ]
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="g1",
                choices=["g1", "g1_with_hands"],
                description="Unitree G1 MuJoCo Menagerie model",
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
                        FindPackageShare("g1_mujoco_description"),
                        "config",
                        "initial_positions.yaml",
                    ]
                ),
                description="Initial joint positions YAML",
            ),
            DeclareLaunchArgument(
                "mujoco_model_file",
                default_value="auto",
                description=(
                    "MuJoCo model file under g1_mujoco_description/mjcf, "
                    "or 'auto' to select by robot_model"
                ),
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
