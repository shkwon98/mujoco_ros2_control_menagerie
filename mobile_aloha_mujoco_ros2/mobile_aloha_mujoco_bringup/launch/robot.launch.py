#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
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
    arguments = [
        name,
        "--controller-ros-args",
        "--ros-args -r __ns:=/control/body",
    ]
    if remappings:
        arguments += ["--controller-ros-args", f"--ros-args {remappings}"]
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/",
        arguments=arguments,
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    robot_model = LaunchConfiguration("robot_model")
    headless = LaunchConfiguration("headless")
    is_piper = EqualsSubstitution(robot_model, "piper")
    description_share = PathSubstitution(
        FindPackageShare("mobile_aloha_mujoco_description")
    )
    xacro_file = IfElseSubstitution(
        is_piper, ROBOT_MODELS["piper"]["xacro"], ROBOT_MODELS["vx300s"]["xacro"]
    )

    controller_file = IfElseSubstitution(
        is_piper,
        ROBOT_MODELS["piper"]["controllers"],
        ROBOT_MODELS["vx300s"]["controllers"],
    )

    description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    description_share / "urdf" / xacro_file,
                    " headless:=",
                    headless,
                ]
            ),
            value_type=str,
        )
    }

    controllers = description_share / "config" / "ros2_control" / controller_file
    # Keep the process instance used by the RQT startup event.
    arm_left_controller_spawner = controller_spawner("arm_left_controller")

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
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                choices=["true", "false"],
                description="Start the Nav2 control pipeline",
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
                                condition=IfCondition(
                                    EqualsSubstitution(robot_model, name)
                                ),
                                package="rqt_gui",
                                executable="rqt_gui",
                                namespace="/",
                                arguments=[
                                    "--perspective-file",
                                    PathSubstitution(
                                        FindPackageShare(
                                            "mobile_aloha_mujoco_bringup")
                                    )
                                    / "config"
                                    / "mobile_aloha_mujoco.perspective",
                                    "--force-discover",
                                ],
                                parameters=[{"use_sim_time": True}],
                                remappings=[
                                    ("robot_description", "/robot_description"),
                                    *[
                                        (
                                            f"/gripper_{side}_controller/joint_trajectory",
                                            f"/control/hand_{side}/gripper_controller/joint_trajectory",
                                        )
                                        for side in ("left", "right")
                                    ],
                                    *[
                                        (
                                            f"/{name}/{topic}",
                                            f"/control/body/{name}/{topic}",
                                        )
                                        for name in (
                                            "arm_left_controller",
                                            "arm_right_controller",
                                            *model["leader_controllers"],
                                        )
                                        for topic in (
                                            "controller_state",
                                            "joint_trajectory",
                                        )
                                    ],
                                    *[
                                        (
                                            f"/gripper_{side}_controller/controller_state",
                                            f"/control/body/gripper_{side}_controller/controller_state",
                                        )
                                        for side in ("left", "right")
                                    ],
                                ],
                                output="screen",
                            )
                            for name, model in ROBOT_MODELS.items()
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
                namespace="/",
                parameters=[controllers],
                remappings=[("robot_description", "/robot_description")],
                output="screen",
            ),
            controller_spawner(
                "joint_state_broadcaster",
                "--remap joint_states:=/sensors/proprio/body/joint_states",
            ),
            arm_left_controller_spawner,
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
            GroupAction(
                condition=IfCondition(is_piper),
                actions=[
                    controller_spawner(name)
                    for name in ROBOT_MODELS["piper"]["leader_controllers"]
                ],
            ),
            IncludeLaunchDescription(
                PathSubstitution(FindPackageShare("mobile_aloha_mujoco_nav"))
                / "launch"
                / "navigation.launch.py",
                condition=IfCondition(LaunchConfiguration("use_navigation")),
            ),
        ]
    )
