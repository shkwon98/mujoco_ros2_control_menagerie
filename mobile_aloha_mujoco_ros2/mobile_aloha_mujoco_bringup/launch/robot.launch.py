#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
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


def launch_setup(context):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    headless = LaunchConfiguration("headless")
    model = ROBOT_MODELS[robot_model]
    description_share = get_package_share_directory(
        "mobile_aloha_mujoco_description")

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

    arm_left_controller_spawner = controller_spawner("arm_left_controller")

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
    ]
    nodes.extend(controller_spawner(name)
                 for name in model["leader_controllers"])
    rqt_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace="/",
        arguments=[
            "--perspective-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("mobile_aloha_mujoco_bringup"),
                    "config",
                    "mobile_aloha_mujoco.perspective",
                ]
            ),
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
                for topic in ("controller_state", "joint_trajectory")
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
    # Restore the RQT selection only after its controller is active.
    rqt_after_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_left_controller_spawner,
            on_exit=lambda event, _: [rqt_node] if event.returncode == 0 else [],
        ),
        condition=IfCondition(LaunchConfiguration("use_rqt")),
    )

    return [rqt_after_spawner, *nodes]


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
            OpaqueFunction(function=launch_setup),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("mobile_aloha_mujoco_nav"),
                            "launch",
                            "navigation.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("use_navigation")),
            ),
        ]
    )
