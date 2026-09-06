#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mujoco_model_file = LaunchConfiguration("mujoco_model_file")
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")

    robot_model_value = robot_model.perform(context)
    controllers_yaml_value = controllers_yaml.perform(context)
    mujoco_model_file_value = mujoco_model_file.perform(context)

    description_share = get_package_share_directory("g1_mujoco_description")
    controller_file_by_model = {
        "g1": "g1_controllers.yaml",
        "g1_with_hands": "g1_with_hands_controllers.yaml",
        "g1_with_inspire_hands": "g1_with_inspire_hands_controllers.yaml",
    }
    mujoco_model_file_by_model = {
        "g1": "scene.xml",
        "g1_with_hands": "scene_with_hands_fixed.xml",
        "g1_with_inspire_hands": "scene_inspire_hand_fixed.xml",
    }
    hand_xacro_files_by_model = {
        "g1_with_hands": (
            "g1_hand_left.urdf.xacro",
            "g1_hand_right.urdf.xacro",
        ),
        "g1_with_inspire_hands": (
            "g1_inspire_hand_left.urdf.xacro",
            "g1_inspire_hand_right.urdf.xacro",
        ),
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

    control_robot_description_content = Command(
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
            " headless:=",
            headless,
        ]
    )
    control_robot_description = {
        "robot_description": ParameterValue(
            control_robot_description_content, value_type=str
        )
    }

    body_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " robot_model:=g1",
            " initial_positions_file:=",
            initial_positions_file,
            " mujoco_model_file:=scene.xml",
            " headless:=",
            headless,
        ]
    )
    body_robot_description = {
        "robot_description": ParameterValue(
            body_robot_description_content, value_type=str
        )
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/",
        parameters=[controllers_yaml_value],
        output="screen",
        ros_arguments=["--log-level", log_level],
        remappings=[
            ("robot_description", "/control/body/robot_description"),
        ],
    )
    robot_description_publisher_node = Node(
        package="g1_mujoco_bringup",
        executable="robot_description_publisher.py",
        namespace="/control/body",
        parameters=[control_robot_description],
        output="screen",
        ros_arguments=["--log-level", log_level],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/sensors/proprio/body",
        parameters=[body_robot_description],
        output="screen",
        ros_arguments=["--log-level", log_level],
        remappings=[
            ("robot_description", "/sensors/proprio/body/robot_description"),
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
            namespace="/",
            output="screen",
            arguments=[
                controller_name,
                "--controller-ros-args",
                f"--ros-args -r __ns:=/control/body --remap joint_states:={joint_states_topic}",
                "--controller-ros-args",
                f"--ros-args --remap dynamic_joint_states:={dynamic_joint_states_topic}",
            ],
            ros_arguments=["--log-level", log_level],
        )

    def make_controller_spawner(
        controller_name,
        command_topic,
        action_topic,
        controller_namespace="/control/body",
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
            ]
        )
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/",
            output="screen",
            arguments=controller_args,
            ros_arguments=["--log-level", log_level],
        )

    arm_left_controller_spawner = make_controller_spawner(
        "arm_left_controller",
        "/control/body/arm_left_controller/joint_trajectory",
        "/control/body/arm_left_controller/follow_joint_trajectory",
    )

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_pelvis",
            # fmt: off
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0.793",
                "--yaw", "0",
                "--pitch", "0",
                "--roll", "0",
                "--frame-id", "map",
                "--child-frame-id", "pelvis",
            ],
            # fmt: on
        ),
        ros2_control_node,
        robot_description_publisher_node,
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
        arm_left_controller_spawner,
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

    if robot_model_value in ["g1_with_hands", "g1_with_inspire_hands"]:
        left_hand_xacro_file, right_hand_xacro_file = hand_xacro_files_by_model[
            robot_model_value
        ]
        left_hand_robot_description = {
            "robot_description": ParameterValue(
                Command(
                    [
                        PathJoinSubstitution([FindExecutable(name="xacro")]),
                        " ",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("g1_mujoco_description"),
                                "urdf",
                                left_hand_xacro_file,
                            ]
                        ),
                    ]
                ),
                value_type=str,
            )
        }
        right_hand_robot_description = {
            "robot_description": ParameterValue(
                Command(
                    [
                        PathJoinSubstitution([FindExecutable(name="xacro")]),
                        " ",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("g1_mujoco_description"),
                                "urdf",
                                right_hand_xacro_file,
                            ]
                        ),
                    ]
                ),
                value_type=str,
            )
        }
        nodes.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace="/sensors/proprio/hand_left",
                    parameters=[left_hand_robot_description],
                    output="screen",
                    ros_arguments=["--log-level", log_level],
                    remappings=[
                        ("robot_description", "/control/hand_left/robot_description"),
                        ("joint_states", "/sensors/proprio/hand_left/joint_states"),
                    ],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace="/sensors/proprio/hand_right",
                    parameters=[right_hand_robot_description],
                    output="screen",
                    ros_arguments=["--log-level", log_level],
                    remappings=[
                        ("robot_description", "/control/hand_right/robot_description"),
                        ("joint_states", "/sensors/proprio/hand_right/joint_states"),
                    ],
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

    rqt_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace="/",
        arguments=[
            "--perspective-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("g1_mujoco_bringup"),
                    "config",
                    "g1_mujoco.perspective",
                ]
            ),
            "--force-discover",
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("robot_description", "/control/body/robot_description"),
            *[
                (
                    f"/hand_{side}_controller/{topic}",
                    f"/control/hand_{side}/hand_{side}_controller/{topic}",
                )
                for side in ("left", "right")
                for topic in ("controller_state", "joint_trajectory")
            ],
            *[
                (
                    f"/{name}/{topic}",
                    f"/control/body/{name}/{topic}",
                )
                for name in (
                    "arm_left_controller",
                    "arm_right_controller",
                    "torso_controller",
                    "leg_controller",
                )
                for topic in ("controller_state", "joint_trajectory")
            ],
        ],
        output="screen",
    )
    # Restore the RQT selection only after its controller is active.
    rqt_after_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_left_controller_spawner,
            on_exit=lambda event, _: [rqt_node] if event.returncode == 0 else [],
        ),
        condition=IfCondition(LaunchConfiguration("use_rqt")),
    )

    return [rqt_after_spawner, *nodes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="g1",
                choices=["g1", "g1_with_hands", "g1_with_inspire_hands"],
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
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its graphical window",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="ROS log level",
            ),
            DeclareLaunchArgument(
                "use_rqt",
                default_value="false",
                choices=["true", "false"],
                description="Launch RQT joint trajectory controller for the robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
