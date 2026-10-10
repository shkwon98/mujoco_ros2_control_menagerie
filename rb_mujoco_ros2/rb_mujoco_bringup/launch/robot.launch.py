#!/usr/bin/env python3
"""Declare this robot's nodes; prepare its full and partitioned URDFs once."""

import re
import xml.etree.ElementTree as ET
from copy import deepcopy
from math import isfinite
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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


def launch_robot(context):
    arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "instance_id",
            "hand_instance_id",
            "robot_model",
            "hand_model",
            "hand_side",
            "base_xyz",
            "base_rpy",
            "use_sim_time",
            "headless",
            "log_level",
            "use_rqt",
        )
    }
    instance, hand_instance = arguments["instance_id"], arguments["hand_instance_id"]
    model, hand, side = (
        arguments["robot_model"],
        arguments["hand_model"],
        arguments["hand_side"],
    )
    for value in (instance, hand_instance):
        if value and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value):
            raise ValueError(
                "instance_id must be a single ROS namespace token")
    if not instance:
        interface = {
            "manager": "/",
            "internal": "/robot_description",
            "body": "/sensors/proprio/body",
            "body_description": "/control/body/robot_description",
            "arm": "/control/body",
            "hand": f"/sensors/proprio/hand_{side}",
            "hand_description": f"/control/hand_{side}/robot_description",
            "eef": f"/control/hand_{side}",
            "eef_controller": f"hand_{side}_controller",
            "body_prefix": "",
            "hand_prefix": "",
            "clock": "/clock",
        }
    else:
        if not hand_instance:
            raise ValueError("hand_instance_id is required")
        interface = {
            "manager": f"/{instance}",
            "internal": f"/{instance}/internal/robot_description",
            "body": f"/{instance}",
            "body_description": f"/{instance}/robot_description",
            "arm": f"/{instance}",
            "hand": f"/{hand_instance}/{side}_eef",
            "hand_description": f"/{hand_instance}/{side}_eef/robot_description",
            "eef": f"/{hand_instance}",
            "eef_controller": f"{side}_eef_controller",
            "body_prefix": instance + "_",
            "hand_prefix": f"{hand_instance}_{side}_",
            "clock": f"/{instance}/clock",
        }
    placement = []
    for name in ("base_xyz", "base_rpy"):
        numbers = [float(value) for value in arguments[name].split()]
        if len(numbers) != 3 or not all(map(isfinite, numbers)):
            raise ValueError(f"{name} must contain three finite numbers")
        placement.extend(numbers)
    use_sim_time = arguments["use_sim_time"]
    use_sim_time = (
        not bool(instance) if use_sim_time == "auto" else use_sim_time == "true"
    )
    description = Path(FindPackageShare(
        "rb_mujoco_description").perform(context))
    xml = xacro.process_file(
        str(description / "urdf/rb_mujoco.urdf.xacro"),
        mappings={"robot_model": model, "headless": arguments["headless"]},
    ).toxml()
    directory = TemporaryDirectory(prefix="rb_mujoco_")
    controllers = description / "config/ros2_control/rb_controllers.yaml"
    hand_roots = {}
    if hand != "none":
        compose = run_path(
            str(description / "urdf/compose_wuji.py"))["compose_wuji"]
        xml, controllers = compose(
            xml, description, model, hand, side, Path(directory.name)
        )
        hand_roots[side] = "flange"
    document = yaml.safe_load(Path(controllers).read_text())
    config = document["/**"]
    if instance and hand != "none":
        old, new = f"hand_{side}_controller", interface["eef_controller"]
        manager = config["controller_manager"]["ros__parameters"]
        manager[new] = manager.pop(old)
        config[new] = config.pop(old)
    controller_file = Path(directory.name) / "controllers.yaml"
    controller_file.write_text(yaml.safe_dump(document, sort_keys=False))
    parts = split_description(xml, hand_roots)
    log = ["--log-level", arguments["log_level"]]
    clock_remap = [("/clock", interface["clock"])]
    if instance:
        clock_remap.append(
            (
                "/mujoco_robot_description",
                f"/{instance}/internal/mujoco_robot_description",
            )
        )
    parameters = [{"use_sim_time": use_sim_time}]

    def spawner(name, namespace, remappings=()):
        controller_args = ["--ros-args", "-r", f"__ns:={namespace}"]
        for source, target in [*clock_remap, *remappings]:
            controller_args.extend(["-r", f"{source}:={target}"])
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=interface["manager"],
            arguments=[
                name,
                "--controller-manager",
                interface["manager"].rstrip("/") + "/controller_manager",
                "--controller-ros-args",
                " ".join(controller_args),
            ],
            parameters=parameters,
            remappings=clock_remap,
            ros_arguments=log,
            output="screen",
        )

    arm = spawner(
        "arm_controller",
        interface["arm"],
        [("~/joint_states", interface["body"] + "/joint_states")],
    )
    actions = [
        RegisterEventHandler(
            OnShutdown(on_shutdown=lambda event, context: directory.cleanup())
        ),
        Node(
            package="rb_mujoco_bringup",
            executable="robot_description_publisher.py",
            namespace=interface["manager"],
            parameters=[*parameters, {"robot_description": xml}],
            remappings=[
                ("robot_description", interface["internal"]), *clock_remap],
            ros_arguments=log,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=interface["body"],
            parameters=[
                *parameters,
                {
                    "robot_description": parts["body"],
                    "frame_prefix": interface["body_prefix"],
                },
            ],
            remappings=[
                ("robot_description", interface["body_description"]),
                ("joint_states", interface["body"] + "/joint_states"),
                *clock_remap,
            ],
            ros_arguments=log,
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=interface["manager"],
            parameters=[str(controller_file), *parameters],
            remappings=[
                ("robot_description", interface["internal"]), *clock_remap],
            ros_arguments=log,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace=interface["manager"],
            name="base_placement",
            parameters=parameters,
            remappings=clock_remap,
            arguments=[
                "--frame-id",
                "map",
                "--child-frame-id",
                interface["body_prefix"] + "link0",
                "--x",
                str(placement[0]),
                "--y",
                str(placement[1]),
                "--z",
                str(placement[2]),
                "--roll",
                str(placement[3]),
                "--pitch",
                str(placement[4]),
                "--yaw",
                str(placement[5]),
            ],
            ros_arguments=log,
        ),
        spawner(
            "body_joint_state_broadcaster",
            interface["arm"],
            [
                ("joint_states", interface["body"] + "/joint_states"),
                ("dynamic_joint_states",
                 interface["body"] + "/dynamic_joint_states"),
            ],
        ),
        arm,
    ]
    if hand != "none":
        actions.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=interface["hand"],
                    parameters=[
                        *parameters,
                        {
                            "robot_description": parts[side],
                            "frame_prefix": interface["hand_prefix"],
                        },
                    ],
                    remappings=[
                        ("robot_description", interface["hand_description"]),
                        ("joint_states", interface["hand"] + "/joint_states"),
                        *clock_remap,
                    ],
                    ros_arguments=log,
                ),
                spawner(
                    f"hand_{side}_joint_state_broadcaster",
                    interface["eef"],
                    [
                        ("joint_states", interface["hand"] + "/joint_states"),
                        (
                            "dynamic_joint_states",
                            interface["hand"] + "/dynamic_joint_states",
                        ),
                    ],
                ),
                spawner(
                    interface["eef_controller"],
                    interface["eef"],
                    [("~/joint_states", interface["hand"] + "/joint_states")],
                ),
            ]
        )
        if instance:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    namespace=interface["hand"],
                    name="attachment",
                    parameters=parameters,
                    remappings=clock_remap,
                    arguments=[
                        "--frame-id",
                        interface["body_prefix"] + "flange",
                        "--child-frame-id",
                        interface["hand_prefix"] + "flange",
                    ],
                    ros_arguments=log,
                )
            )
    if instance:
        visual_parts = [("body", interface["body"], interface["body_prefix"])]
        if hand != "none":
            visual_parts.append((side, interface["hand"], interface["hand_prefix"]))
        for part, namespace, prefix in visual_parts:
            visual = ET.fromstring(parts[part])
            # RViz inserts a slash in TF Prefix; match RSP's literal prefix instead.
            for link in visual.findall("link"):
                link.set("name", prefix + link.get("name"))
            for reference in (
                visual.findall("joint/parent") + visual.findall("joint/child")
            ):
                reference.set("link", prefix + reference.get("link"))
            actions.append(
                Node(
                    package="rb_mujoco_bringup",
                    executable="robot_description_publisher.py",
                    name="visual_description_publisher",
                    namespace=namespace,
                    parameters=[
                        *parameters,
                        {"robot_description": ET.tostring(visual, encoding="unicode")},
                    ],
                    remappings=[
                        ("robot_description", namespace + "/visualization/robot_description"),
                        *clock_remap,
                    ],
                    ros_arguments=log,
                )
            )
    if arguments["use_rqt"] == "true":
        rqt = Node(
            package="rqt_gui",
            executable="rqt_gui",
            namespace=interface["manager"],
            arguments=[
                "--perspective-file",
                Path(FindPackageShare("rb_mujoco_bringup").perform(context))
                / "config/rb_mujoco.perspective",
                "--force-discover",
            ],
            parameters=parameters,
            remappings=[
                ("robot_description", interface["internal"]),
                *clock_remap,
                *[
                    (f"/{name}/{topic}", f"{namespace}/{name}/{topic}")
                    for name, namespace in [
                        ("arm_controller", interface["arm"]),
                        (interface["eef_controller"], interface["eef"]),
                    ]
                    for topic in ("controller_state", "joint_trajectory")
                ],
            ],
            ros_arguments=log,
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm,
                    on_exit=lambda event, context: (
                        [rqt] if event.returncode == 0 else []
                    ),
                )
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", choices=list(ROBOT_MODELS)),
            DeclareLaunchArgument(
                "instance_id",
                default_value="",
                description="Device namespace and TF prefix; empty retains legacy paths.",
            ),
            DeclareLaunchArgument(
                "hand_instance_id",
                default_value="wuji_hand_01",
                description="Attached hand's device namespace.",
            ),
            DeclareLaunchArgument(
                "hand_model",
                default_value="none",
                choices=["none", "wuji_hand", "wuji_hand2"],
            ),
            DeclareLaunchArgument(
                "hand_side", default_value="right", choices=["left", "right"]
            ),
            DeclareLaunchArgument(
                "base_xyz",
                default_value="0 0 0",
                description="Robot base placement in map, metres.",
            ),
            DeclareLaunchArgument(
                "base_rpy",
                default_value="0 0 0",
                description="Robot base orientation in map, radians.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                choices=["auto", "true", "false"],
                description="auto uses system time for named instances and simulation time for legacy execution.",
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_rqt", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                choices=["debug", "info", "warn", "error", "fatal"],
            ),
            OpaqueFunction(function=launch_robot),
        ]
    )
