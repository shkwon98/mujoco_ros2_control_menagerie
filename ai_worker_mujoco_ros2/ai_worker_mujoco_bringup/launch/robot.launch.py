#!/usr/bin/env python3

from launch import LaunchContext, LaunchDescription, Substitution
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AnySubstitution,
    Command,
    EqualsSubstitution,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def described_path(value: Substitution, **placeholders: str) -> Substitution:
    context = LaunchContext()
    context.launch_configurations.update(placeholders)
    value.describe = lambda: value.perform(context)
    return value


def make_robot_description(xacro_file, **mappings):
    command = [
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
    ]
    for name, value in mappings.items():
        command.extend([" ", name, ":=", value])

    return {
        "robot_description": ParameterValue(
            Command(command),
            value_type=str,
        )
    }


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")
    controllers_yaml_value = IfElseSubstitution(
        EqualsSubstitution(controllers_yaml, "auto"),
        PathSubstitution(FindPackageShare("ai_worker_mujoco_description"))
        / "config"
        / "ros2_control"
        / ["ai_worker_", robot_model, "_controllers.yaml"],
        controllers_yaml,
    )

    xacro_file = (
        PathSubstitution(FindPackageShare("ai_worker_mujoco_description"))
        / "urdf"
        / "ai_worker_mujoco.urdf.xacro"
    )
    full_robot_description = make_robot_description(
        xacro_file,
        robot_model=robot_model,
        initial_positions_file=initial_positions_file,
        headless=headless,
        include_hands="true",
        include_ros2_control="true",
    )

    body_robot_description = make_robot_description(
        xacro_file,
        robot_model=robot_model,
        initial_positions_file=initial_positions_file,
        headless=headless,
        include_hands="false",
        include_ros2_control="false",
    )

    hand_xacro_file = (
        PathSubstitution(FindPackageShare("ai_worker_mujoco_description"))
        / "urdf"
        / "ai_worker_hand.urdf.xacro"
    )
    left_hand_robot_description = make_robot_description(
        hand_xacro_file, robot_model=robot_model, side="left"
    )

    right_hand_robot_description = make_robot_description(
        hand_xacro_file, robot_model=robot_model, side="right"
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
        command_topic=None,
        action_topic=None,
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
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace="/",
            output="screen",
            arguments=controller_args,
            ros_arguments=["--log-level", log_level],
        )

    # Keep the process instance used by the RQT startup event.
    arm_left_controller_spawner = make_controller_spawner(
        "arm_left_controller",
        "/control/body/arm_left_controller/joint_trajectory",
        "/control/body/arm_left_controller/follow_joint_trajectory",
    )

    is_swerve = AnySubstitution(
        EqualsSubstitution(robot_model, "ffw_sg2"),
        EqualsSubstitution(robot_model, "ffw_sh5"),
    )

    fixed_frame_child = IfElseSubstitution(is_swerve, "odom", "base_link")

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
                default_value=described_path(
                    PathSubstitution(FindPackageShare(
                        "ai_worker_mujoco_description"))
                    / "config" / "initial_positions.yaml",
                ),
                description="Initial joint positions YAML",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its graphical window",
            ),
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                choices=["true", "false"],
                description="Start Nav2 for mobile-base models",
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
            # Open RQT only after its selected controller is active.
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_left_controller_spawner,
                    on_exit=lambda event, _: (
                        [
                            Node(
                                package="rqt_gui",
                                executable="rqt_gui",
                                namespace="/",
                                arguments=[
                                    "--perspective-file",
                                    PathSubstitution(
                                        FindPackageShare(
                                            "ai_worker_mujoco_bringup")
                                    )
                                    / "config"
                                    / "ai_worker_mujoco.perspective",
                                    "--force-discover",
                                ],
                                parameters=[{"use_sim_time": True}],
                                remappings=[
                                    ("robot_description", "/robot_description"),
                                    *[
                                        (
                                            f"/hand_{side}_controller/{topic}",
                                            f"/control/hand_{side}/hand_{side}_controller/{topic}",
                                        )
                                        for side in ("left", "right")
                                        for topic in (
                                            "controller_state",
                                            "joint_trajectory",
                                        )
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
                                            "head_controller",
                                        )
                                        for topic in (
                                            "controller_state",
                                            "joint_trajectory",
                                        )
                                    ],
                                ],
                                output="screen",
                            )
                        ]
                        if event.returncode == 0
                        else []
                    ),
                ),
                condition=IfCondition(LaunchConfiguration("use_rqt")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=["map_to_", fixed_frame_child],
                # fmt: off
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--yaw", "0",
                    "--pitch", "0",
                    "--roll", "0",
                    "--frame-id", "map",
                    "--child-frame-id", fixed_frame_child,
                ],
                # fmt: on
            ),
            Node(
                package="ai_worker_mujoco_bringup",
                executable="robot_description_publisher.py",
                parameters=[full_robot_description],
                output="screen",
                ros_arguments=["--log-level", log_level],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="/",
                parameters=[controllers_yaml_value],
                output="screen",
                ros_arguments=["--log-level", log_level],
                remappings=[
                    ("robot_description", "/robot_description"),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="/sensors/proprio/body",
                parameters=[body_robot_description],
                output="screen",
                ros_arguments=["--log-level", log_level],
                remappings=[
                    ("robot_description", "/control/body/robot_description"),
                    ("joint_states", "/sensors/proprio/body/joint_states"),
                ],
            ),
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
                "head_controller",
                "/control/body/head_controller/joint_trajectory",
                "/control/body/head_controller/follow_joint_trajectory",
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
            GroupAction(
                condition=IfCondition(is_swerve),
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        namespace="/",
                        output="screen",
                        arguments=[
                            "swerve_drive_controller",
                            "--controller-ros-args",
                            "--ros-args -r __ns:=/control/body --remap /control/body/odom:=/odom",
                        ],
                        ros_arguments=["--log-level", log_level],
                    ),
                    IncludeLaunchDescription(
                        PathSubstitution(FindPackageShare(
                            "ai_worker_mujoco_nav"))
                        / "launch"
                        / "navigation.launch.py",
                        condition=IfCondition(
                            LaunchConfiguration("use_navigation")),
                    ),
                ],
            ),
        ]
    )
