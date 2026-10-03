#!/usr/bin/env python3
"""Declare this robot's nodes; prepare its full and partitioned URDFs once."""

import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROBOT_MODELS = (
    "rb1_500es_u",
    "rb3_730es_u",
    "rb3_1200e",
    "rb3_1200e_u",
    "rb5_850e",
    "rb5_850e_u",
    "rb6_1700e_u",
    "rb10_1300e",
    "rb10_1300e_u",
    "rb16_900e",
    "rb16_900e_u",
    "rb20_1800e_u",
    "rb20_1900es_u",
    "rb30_1400es_u",
)


def split_description(xml: str, hand_roots: dict[str, str]) -> dict[str, str]:
    """Keep each hand's attachment link in the body and as an empty hand root.

    Every original link geometry and joint belongs to exactly one partition.
    All partitions retain global materials and omit ros2_control metadata.
    """
    full = ET.fromstring(xml)
    links = {link.get("name") for link in full.findall("link")}
    joints = full.findall("joint")
    children: dict[str, list[str]] = {}
    for joint in joints:
        children.setdefault(joint.find("parent").get("link"), []).append(
            joint.find("child").get("link")
        )

    hands: dict[str, set[str]] = {}
    for side, root in hand_roots.items():
        if not root:
            continue
        if root not in links:
            raise ValueError(f"Unknown {side} hand attachment link: {root}")
        descendants: set[str] = set()
        pending = list(children.get(root, []))
        while pending:
            child = pending.pop()
            if child == root or child in descendants:
                raise ValueError(f"Cyclic URDF subtree at {root}")
            descendants.add(child)
            pending.extend(children.get(child, []))
        hands[side] = descendants
    removed = set().union(*hands.values())
    if sum(map(len, hands.values())) != len(removed) or removed.intersection(
        hand_roots.values()
    ):
        raise ValueError("Hand URDF subtrees must be disjoint")

    result = {}
    for part, selected in {"body": links - removed, **hands}.items():
        robot = ET.Element("robot", name=f"{full.get('name')}_{part}")
        if part != "body":
            ET.SubElement(robot, "link", name=hand_roots[part])
        for element in full:
            if (
                element.tag == "material"
                or element.tag == "link"
                and element.get("name") in selected
                or element.tag == "joint"
                and element.find("child").get("link") in selected
            ):
                robot.append(deepcopy(element))
        result[part] = ET.tostring(robot, encoding="unicode")
    return result


def description_nodes() -> list:
    log_level = LaunchConfiguration("log_level")
    return [
        Node(
            package="rb_mujoco_bringup",
            executable="robot_description_publisher.py",
            namespace="/",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        LaunchConfiguration("_full_robot_description"), value_type=str
                    )
                }
            ],
            ros_arguments=["--log-level", log_level],
            output="screen",
        ),
        *[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=f"/sensors/proprio/{segment}",
                condition=(
                    IfCondition(LaunchConfiguration(f"_has_{segment}"))
                    if segment != "body"
                    else None
                ),
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            LaunchConfiguration(
                                f"_{segment}_robot_description"),
                            value_type=str,
                        )
                    }
                ],
                remappings=[
                    ("robot_description",
                     f"/control/{segment}/robot_description"),
                    ("joint_states",
                     f"/sensors/proprio/{segment}/joint_states"),
                ],
                ros_arguments=["--log-level", log_level],
                output="screen",
            )
            for segment in ("body", "hand_left", "hand_right")
        ],
    ]


def controller_spawner(
    name: str, namespace: str = "/control/body", remappings: tuple = (), condition=None
) -> Node:
    controller_args = ["--ros-args", "-r", f"__ns:={namespace}"]
    for source, target in remappings:
        controller_args.extend(["-r", f"{source}:={target}"])
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/",
        arguments=[name, "--controller-ros-args", " ".join(controller_args)],
        condition=condition,
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )


def prepare_descriptions(context) -> list:
    """Resolve model files and split XML; node construction stays declarative."""
    description = PathSubstitution(
        FindPackageShare("rb_mujoco_description"))
    model = LaunchConfiguration("robot_model").perform(context)
    controllers = (
        description / "config" / "ros2_control" / "rb_controllers.yaml"
    ).perform(context)
    xml = xacro.process_file(
        (description / "urdf" / "rb_mujoco.urdf.xacro").perform(context),
        mappings={
            "robot_model": model,
            "headless": LaunchConfiguration("headless").perform(context),
        },
    ).toxml()
    hand = LaunchConfiguration("hand_model").perform(context)
    side = LaunchConfiguration("hand_side").perform(context)
    hand_roots, temporary = {}, None
    if hand != "none":
        temporary = TemporaryDirectory(prefix="rb_wuji_")
        compose = run_path(
            (description / "urdf" / "compose_wuji.py").perform(context)
        )["compose_wuji"]
        xml, controllers = compose(
            xml, Path(description.perform(context)),
            model, hand, side, Path(temporary.name),
        )
        hand_roots[side] = f"{side}_hand_base"
    parts = split_description(xml, hand_roots)
    values = {
        "_full_robot_description": xml,
        "_controllers_yaml": str(controllers),
        **{f"_has_hand_{side}": str(side in hand_roots).lower()
           for side in ("left", "right")},
        "_body_robot_description": parts["body"],
        **{
            f"_hand_{side}_robot_description": value
            for side, value in parts.items()
            if side != "body"
        },
    }
    actions = [SetLaunchConfiguration(name, value)
               for name, value in values.items()]
    if temporary is not None:
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(
                function=lambda _: temporary.cleanup())]
        )))
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup = PathSubstitution(FindPackageShare("rb_mujoco_bringup"))
    log_level = LaunchConfiguration("log_level")
    body_controllers = ("arm_controller",)
    body_spawners = {
        name: controller_spawner(
            name, remappings=(
                ("~/joint_states", "/sensors/proprio/body/joint_states"),)
        )
        for name in body_controllers
    }
    trajectory_names = [(name, "body") for name in body_controllers]
    trajectory_names.extend(
        (f"hand_{side}_controller", f"hand_{side}") for side in ("left", "right")
    )
    placement_frame = "link0"
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace="/",
        arguments=[
            "--perspective-file",
            bringup / "config" / "rb_mujoco.perspective",
            "--force-discover",
        ],
        remappings=[
            ("robot_description", "/robot_description"),
            *[
                (f"/{name}/{topic}", f"/control/{segment}/{name}/{topic}")
                for name, segment in trajectory_names
                for topic in ("controller_state", "joint_trajectory")
            ],
        ],
        ros_arguments=["--log-level", log_level],
        output="screen",
    )
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument(
                "robot_model",
                choices=list(ROBOT_MODELS),
                description="Rainbow Robotics RB-Series model",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its graphical window",
            ),
            DeclareLaunchArgument(
                "hand_model", default_value="none",
                choices=["none", "wuji_hand",
                         "wuji_hand2_beta1", "wuji_hand2_beta2"],
                description="Optional Wuji hand at the RB TCP (simulation mount)",
            ),
            DeclareLaunchArgument(
                "hand_side", default_value="right", choices=["left", "right"],
                description="Side of the single attached Wuji hand",
            ),
            DeclareLaunchArgument(
                "use_rqt",
                default_value="false",
                choices=["true", "false"],
                description="Launch RQT joint trajectory controller",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
                description="ROS log level",
            ),
            OpaqueFunction(function=prepare_descriptions),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=body_spawners[body_controllers[0]],
                    on_exit=lambda event, _: [
                        rqt] if event.returncode == 0 else [],
                ),
                condition=IfCondition(LaunchConfiguration("use_rqt")),
            ),
            *description_nodes(),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="/",
                parameters=[LaunchConfiguration("_controllers_yaml")],
                remappings=[("robot_description", "/robot_description")],
                ros_arguments=["--log-level", log_level],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=["map_to_", placement_frame],
                arguments=["--frame-id", "map",
                           "--child-frame-id", placement_frame],
                ros_arguments=["--log-level", log_level],
            ),
            controller_spawner(
                "body_joint_state_broadcaster",
                remappings=(
                    ("joint_states", "/sensors/proprio/body/joint_states"),
                    (
                        "dynamic_joint_states",
                        "/sensors/proprio/body/dynamic_joint_states",
                    ),
                ),
            ),
            *body_spawners.values(),
            *[
                GroupAction(
                    condition=IfCondition(
                        LaunchConfiguration(f"_has_hand_{side}")),
                    actions=[
                        controller_spawner(
                            f"hand_{side}_joint_state_broadcaster", f"/control/hand_{side}",
                            remappings=(
                                ("joint_states",
                                 f"/sensors/proprio/hand_{side}/joint_states"),
                                ("dynamic_joint_states",
                                 f"/sensors/proprio/hand_{side}/dynamic_joint_states"),
                            ),
                        ),
                        controller_spawner(
                            f"hand_{side}_controller", f"/control/hand_{side}",
                            remappings=(
                                ("~/joint_states", f"/sensors/proprio/hand_{side}/joint_states"),),
                        ),
                    ],
                ) for side in ("left", "right")
            ],
        ]
    )
