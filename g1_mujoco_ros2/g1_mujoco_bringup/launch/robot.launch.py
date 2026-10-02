#!/usr/bin/env python3
"""Declare this robot's nodes; prepare its full and partitioned URDFs once."""

import xml.etree.ElementTree as ET
from copy import deepcopy

import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROBOT_MODELS = {
    "g1": "scene.xml",
    "g1_with_hands": "scene_with_hands_fixed.xml",
    "g1_with_inspire_hands": "scene_inspire_hand_fixed.xml",
}


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
            package="g1_mujoco_bringup",
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
                    IfCondition(LaunchConfiguration("_has_hands"))
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
    description = PathSubstitution(FindPackageShare("g1_mujoco_description"))
    model = LaunchConfiguration("robot_model").perform(context)
    controllers = LaunchConfiguration("controllers_yaml").perform(context)
    if controllers == "auto":
        controllers = (
            description / "config" / "ros2_control" /
            f"{model}_controllers.yaml"
        ).perform(context)
    scene = LaunchConfiguration("mujoco_model_file").perform(context)
    if scene == "auto":
        scene = ROBOT_MODELS[model]
    xml = xacro.process_file(
        (description / "urdf" / "g1_mujoco.urdf.xacro").perform(context),
        mappings={
            "robot_model": model,
            "initial_positions_file": LaunchConfiguration(
                "initial_positions_file"
            ).perform(context),
            "mujoco_model_file": scene,
            "headless": LaunchConfiguration("headless").perform(context),
        },
    ).toxml()
    hand_roots = (
        {"left": "left_hand_base", "right": "right_hand_base"} if model != "g1" else {}
    )
    parts = split_description(xml, hand_roots)
    values = {
        "_full_robot_description": xml,
        "_controllers_yaml": str(controllers),
        "_has_hands": str(bool(hand_roots)).lower(),
        "_body_robot_description": parts["body"],
        **{
            f"_hand_{side}_robot_description": value
            for side, value in parts.items()
            if side != "body"
        },
    }
    values["_fixed_base"] = str(scene.endswith("_fixed.xml")).lower()
    actions = [SetLaunchConfiguration(name, value)
               for name, value in values.items()]
    return actions


def generate_launch_description() -> LaunchDescription:
    bringup = PathSubstitution(FindPackageShare("g1_mujoco_bringup"))
    log_level = LaunchConfiguration("log_level")
    body_controllers = (
        "arm_left_controller",
        "arm_right_controller",
        "torso_controller",
        "leg_controller",
    )
    body_spawners = {
        name: controller_spawner(
            name, remappings=(
                ("~/joint_states", "/sensors/proprio/body/joint_states"),)
        )
        for name in body_controllers
    }
    trajectory_names = [(name, "body") for name in body_controllers]
    trajectory_names += [
        (f"hand_{side}_controller", f"hand_{side}") for side in ("left", "right")
    ]
    placement_frame = "pelvis"
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace="/",
        arguments=[
            "--perspective-file",
            bringup / "config" / "g1_mujoco.perspective",
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
                default_value="g1",
                choices=list(ROBOT_MODELS),
                description="Unitree G1 model",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value="auto",
                description="Controller YAML, or 'auto' to select by robot_model",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=PathSubstitution(
                    FindPackageShare("g1_mujoco_description")
                )
                / "config"
                / "initial_positions.yaml",
                description="Initial joint positions YAML",
            ),
            DeclareLaunchArgument(
                "mujoco_model_file",
                default_value="auto",
                description="MJCF file under g1_mujoco_description/mjcf, or 'auto'",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Run MuJoCo without its graphical window",
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
                arguments=[
                    "--z",
                    "0.793",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    placement_frame,
                ],
                condition=IfCondition(LaunchConfiguration("_fixed_base")),
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
            GroupAction(
                condition=IfCondition(LaunchConfiguration("_has_hands")),
                actions=[
                    *[
                        controller_spawner(
                            f"hand_{side}_joint_state_broadcaster",
                            namespace="/control/body",
                            remappings=(
                                (
                                    "joint_states",
                                    f"/sensors/proprio/hand_{side}/joint_states",
                                ),
                                (
                                    "dynamic_joint_states",
                                    f"/sensors/proprio/hand_{side}/dynamic_joint_states",
                                ),
                            ),
                        )
                        for side in ("left", "right")
                    ],
                    *[
                        controller_spawner(
                            f"hand_{side}_controller",
                            namespace=f"/control/hand_{side}",
                            remappings=(
                                (
                                    "~/joint_states",
                                    f"/sensors/proprio/hand_{side}/joint_states",
                                ),
                            ),
                        )
                        for side in ("left", "right")
                    ],
                ],
            ),
            controller_spawner(
                "base_pose_broadcaster",
                namespace="/sensors/proprio/body",
                remappings=(("~/pose", "/sensors/proprio/body/base_pose"),),
                condition=UnlessCondition(LaunchConfiguration("_fixed_base")),
            ),
        ]
    )
