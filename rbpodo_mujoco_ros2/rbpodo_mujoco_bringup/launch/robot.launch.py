#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROBOT_MODELS = (
    "rb1_500es_u", "rb3_730es_u", "rb3_1200e", "rb3_1200e_u",
    "rb5_850e", "rb5_850e_u", "rb6_1700e_u", "rb10_1300e", "rb10_1300e_u",
    "rb16_900e", "rb16_900e_u", "rb20_1800e_u", "rb20_1900es_u", "rb30_1400es_u",
)


def generate_launch_description() -> LaunchDescription:
    description_share = PathSubstitution(FindPackageShare("rbpodo_mujoco_description"))
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        description_share / "urdf" / "rbpodo_mujoco.urdf.xacro",
        " robot_model:=", LaunchConfiguration("robot_model"),
        " headless:=", LaunchConfiguration("headless"),
    ]), value_type=str)
    arm_spawner = Node(
        package="controller_manager", executable="spawner", namespace="/",
        arguments=["arm_controller", "--controller-ros-args",
                   "--ros-args -r __ns:=/control/body"], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", choices=list(ROBOT_MODELS),
                              description="Rainbow Robotics RB-Series model"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Run MuJoCo without its GUI."),
        DeclareLaunchArgument("use_rqt", default_value="false", choices=["true", "false"],
                              description="Launch RQT joint trajectory controller."),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            remappings=[("joint_states", "/sensors/proprio/body/joint_states")],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="ros2_control_node", namespace="/",
            parameters=[
                description_share / "config" / "ros2_control" / "rbpodo_controllers.yaml",
                {"use_sim_time": True},
            ],
            remappings=[("robot_description", "/robot_description")], output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner", namespace="/",
            arguments=["body_joint_state_broadcaster", "--controller-ros-args",
                       "--ros-args -r __ns:=/control/body "
                       "--remap joint_states:=/sensors/proprio/body/joint_states"],
            output="screen",
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_spawner, on_exit=lambda event, _: [Node(
                package="rqt_gui", executable="rqt_gui", namespace="/",
                arguments=["--perspective-file",
                           PathSubstitution(FindPackageShare("rbpodo_mujoco_bringup"))
                           / "config" / "rbpodo_mujoco.perspective", "--force-discover"],
                parameters=[{"use_sim_time": True}],
                remappings=[(f"/arm_controller/{topic}", f"/control/body/arm_controller/{topic}")
                            for topic in ("controller_state", "joint_trajectory")],
                output="screen",
            )] if event.returncode == 0 else []),
            condition=IfCondition(LaunchConfiguration("use_rqt")),
        ),
        arm_spawner,
    ])
