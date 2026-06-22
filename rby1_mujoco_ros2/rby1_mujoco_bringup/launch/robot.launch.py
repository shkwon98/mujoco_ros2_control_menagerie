#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description with declared arguments."""
    robot_model = LaunchConfiguration("robot_model")
    robot_version = LaunchConfiguration("robot_version")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    log_level = LaunchConfiguration("log_level")
    has_mobile_base = PythonExpression(["'", robot_model, "' in ['a', 'm', 'a_wuji']"])
    has_wuji_hands = PythonExpression(["'", robot_model, "' == 'a_wuji'"])

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rby1_mujoco_description"), "urdf", "rby1.urdf.xacro"]
    )
    initial_positions_file = PathJoinSubstitution(
        [
            FindPackageShare("rby1_mujoco_description"),
            "config",
            "initial_positions",
            PythonExpression(["'rby1", robot_model, ".yaml'"]),
        ]
    )

    robot_description_content = Command(
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
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    ros2_controllers_config = controllers_yaml

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/control/body",
        parameters=[robot_description, ros2_controllers_config],
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
        condition=UnlessCondition(has_wuji_hands),
    )

    def make_joint_state_broadcaster_spawner(
        controller_name: str,
        joint_states_topic: str,
        dynamic_joint_states_topic: str,
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
            condition=IfCondition(has_wuji_hands),
        )

    def make_controller_spawner(
        controller_name: str,
        command_topic: str,
        action_topic: str = "",
        controller_namespace: str = "",
        joint_states_topic: str = "/sensors/proprio/body/joint_states",
        condition=None,
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
            condition=condition,
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
    head_controller_spawner = make_controller_spawner(
        "head_controller",
        "/control/body/head_controller/joint_trajectory",
    )

    mobile_base_controller_spawner = Node(
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
        condition=IfCondition(has_mobile_base),
    )

    body_joint_state_broadcaster_spawner = make_joint_state_broadcaster_spawner(
        "body_joint_state_broadcaster",
        "/sensors/proprio/body/joint_states",
        "/sensors/proprio/body/dynamic_joint_states",
    )
    hand_left_joint_state_broadcaster_spawner = make_joint_state_broadcaster_spawner(
        "hand_left_joint_state_broadcaster",
        "/sensors/proprio/hand_left/joint_states",
        "/sensors/proprio/hand_left/dynamic_joint_states",
    )
    hand_right_joint_state_broadcaster_spawner = make_joint_state_broadcaster_spawner(
        "hand_right_joint_state_broadcaster",
        "/sensors/proprio/hand_right/joint_states",
        "/sensors/proprio/hand_right/dynamic_joint_states",
    )
    hand_left_controller_spawner = make_controller_spawner(
        "hand_left_controller",
        "/control/hand_left/hand_left_controller/joint_trajectory",
        "/control/hand_left/hand_left_controller/follow_joint_trajectory",
        "/control/hand_left",
        "/sensors/proprio/hand_left/joint_states",
        IfCondition(has_wuji_hands),
    )
    hand_right_controller_spawner = make_controller_spawner(
        "hand_right_controller",
        "/control/hand_right/hand_right_controller/joint_trajectory",
        "/control/hand_right/hand_right_controller/follow_joint_trajectory",
        "/control/hand_right",
        "/sensors/proprio/hand_right/joint_states",
        IfCondition(has_wuji_hands),
    )

    actions = [
        DeclareLaunchArgument(
            "robot_model",
            default_value="a",
            choices=["a", "m", "ub", "a_wuji"],
            description="Robot model",
        ),
        DeclareLaunchArgument(
            "robot_version",
            default_value="v1.2",
            description="Robot version",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("rby1_mujoco_description"),
                    "config",
                    "ros2_control",
                    PythonExpression(
                        ["'rby1", robot_model, "_controllers.yaml'"]
                    ),
                ]
            ),
            description="Controller configuration YAML",
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
        body_joint_state_broadcaster_spawner,
        hand_left_joint_state_broadcaster_spawner,
        hand_right_joint_state_broadcaster_spawner,
        arm_right_controller_spawner,
        arm_left_controller_spawner,
        torso_controller_spawner,
        head_controller_spawner,
        mobile_base_controller_spawner,
        hand_left_controller_spawner,
        hand_right_controller_spawner,
    ]

    return LaunchDescription(actions)
