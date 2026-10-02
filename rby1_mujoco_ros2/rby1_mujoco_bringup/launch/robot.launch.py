#!/usr/bin/env python3
"""Launch this robot with its own descriptions, state publishers and controllers."""

from copy import deepcopy
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

ROBOT_MODELS = {
    "a": {
        "base_model": "a",
        "initial_positions": "rby1a.yaml",
        "controllers": "rby1a_controllers.yaml",
        "has_wuji_hands": False,
        "versions": ("v1.0", "v1.1", "v1.2"),
    },
    "m": {
        "base_model": "m",
        "initial_positions": "rby1m.yaml",
        "controllers": "rby1m_controllers.yaml",
        "has_wuji_hands": False,
        "versions": ("v1.0", "v1.1", "v1.2", "v1.3"),
    },
    "a_wuji": {
        "base_model": "a",
        "initial_positions": "rby1a_wuji.yaml",
        "controllers": "rby1a_wuji_controllers.yaml",
        "has_wuji_hands": True,
        "versions": ("v1.0", "v1.1", "v1.2"),
    },
    "m_wuji": {
        "base_model": "m",
        "initial_positions": "rby1m_wuji.yaml",
        "controllers": "rby1m_wuji_controllers.yaml",
        "has_wuji_hands": True,
        "versions": ("v1.0", "v1.1", "v1.2", "v1.3"),
    },
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


def description_nodes(xml: str, hand_roots: dict[str, str]) -> list:
    descriptions = split_description(xml, hand_roots)
    log_level = LaunchConfiguration("log_level")
    nodes = [
        Node(
            package="rby1_mujoco_bringup",
            executable="robot_description_publisher.py",
            namespace="/",
            parameters=[{"robot_description": ParameterValue(xml, value_type=str)}],
            ros_arguments=["--log-level", log_level],
            output="screen",
        )
    ]
    for part, description in descriptions.items():
        segment = "body" if part == "body" else f"hand_{part}"
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=f"/sensors/proprio/{segment}",
                parameters=[
                    {"robot_description": ParameterValue(description, value_type=str)}
                ],
                remappings=[
                    ("robot_description", f"/control/{segment}/robot_description"),
                    ("joint_states", f"/sensors/proprio/{segment}/joint_states"),
                ],
                ros_arguments=["--log-level", log_level],
                output="screen",
            )
        )
    return nodes


def controller_spawner(
    name: str, namespace: str = "/control/body", remappings: tuple = ()
) -> Node:
    controller_args = ["--ros-args", "-r", f"__ns:={namespace}"]
    for source, target in remappings:
        controller_args.extend(["-r", f"{source}:={target}"])
    return Node(
        package="controller_manager",
        executable="spawner",
        namespace="/",
        arguments=[name, "--controller-ros-args", " ".join(controller_args)],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )


def launch_setup(context) -> list:
    description = Path(get_package_share_directory("rby1_mujoco_description"))
    bringup = Path(get_package_share_directory("rby1_mujoco_bringup"))
    log_level = LaunchConfiguration("log_level")
    model = LaunchConfiguration("robot_model").perform(context)
    version = LaunchConfiguration("robot_version").perform(context)
    hand = LaunchConfiguration("hand_model").perform(context)
    config = ROBOT_MODELS[model]
    if version not in config["versions"]:
        raise RuntimeError(
            f"Unsupported robot_version '{version}' for robot_model '{model}'"
        )
    if hand != "wuji_hand" and not config["has_wuji_hands"]:
        raise RuntimeError("hand_model requires robot_model:=a_wuji or m_wuji")
    controllers = LaunchConfiguration("controllers_yaml").perform(context)
    if controllers == "auto":
        controllers = description / "config/ros2_control" / config["controllers"]
    initial = description / "config/initial_positions" / config["initial_positions"]
    temporary, scene = None, ""
    if hand in ("wuji_hand2_beta1", "wuji_hand2_beta2"):
        temporary = TemporaryDirectory(prefix="rby1_hand2_")
        compose = run_path(description / "urdf/compose_hand2.py")["compose_hand2"]
        scene, initial, controllers = compose(
            description,
            config["base_model"],
            version,
            Path(controllers),
            Path(temporary.name),
            hand_model=hand,
        )
    xml = xacro.process_file(
        str(description / "urdf/rby1.urdf.xacro"),
        mappings={
            "robot_model": model,
            "robot_version": version,
            "hand_model": hand,
            "initial_positions_file": str(initial),
            "mujoco_model_file": scene,
            "headless": LaunchConfiguration("headless").perform(context),
        },
    ).toxml()
    placement_frame = "odom"
    body_controllers = (
        "arm_left_controller",
        "arm_right_controller",
        "torso_controller",
        "head_controller",
    )
    hand_roots = (
        {"left": "left_hand_base", "right": "right_hand_base"}
        if config["has_wuji_hands"]
        else {}
    )
    segments = [
        "body",
        *[f"hand_{side}" for side, anchor in hand_roots.items() if anchor],
    ]
    body_spawners = {
        name: controller_spawner(
            name, remappings=(("~/joint_states", "/sensors/proprio/body/joint_states"),)
        )
        for name in body_controllers
    }
    selected_spawner = body_spawners[body_controllers[0]]
    trajectory_names = [(name, "body") for name in body_controllers]
    trajectory_names += [
        (f"hand_{side}_controller", f"hand_{side}")
        for side, anchor in hand_roots.items()
        if anchor
    ]
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace="/",
        arguments=[
            "--perspective-file",
            str(bringup / "config/rby1_mujoco.perspective"),
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
    nodes = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=selected_spawner,
                on_exit=lambda event, _: [rqt] if event.returncode == 0 else [],
            ),
            condition=IfCondition(LaunchConfiguration("use_rqt")),
        ),
        *description_nodes(xml, hand_roots),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="/",
            parameters=[str(controllers)],
            remappings=[("robot_description", "/robot_description")],
            ros_arguments=[
                "--log-level",
                log_level,
                "--log-level",
                "control.body.base_controller:=error",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"map_to_{placement_frame}",
            arguments=["--frame-id", "map", "--child-frame-id", placement_frame],
            ros_arguments=["--log-level", log_level],
        ),
        *[
            controller_spawner(
                f"{segment}_joint_state_broadcaster",
                namespace="/control/body",
                remappings=(
                    ("joint_states", f"/sensors/proprio/{segment}/joint_states"),
                    (
                        "dynamic_joint_states",
                        f"/sensors/proprio/{segment}/dynamic_joint_states",
                    ),
                ),
            )
            for segment in segments
        ],
        *body_spawners.values(),
        *[
            controller_spawner(
                f"hand_{side}_controller",
                namespace=f"/control/hand_{side}",
                remappings=(
                    ("~/joint_states", f"/sensors/proprio/hand_{side}/joint_states"),
                ),
            )
            for side, anchor in hand_roots.items()
            if anchor
        ],
    ]
    nodes.append(
        controller_spawner(
            "base_controller",
            remappings=(
                ("~/cmd_vel", "/cmd_vel"),
                ("~/reference", "/cmd_vel"),
                ("~/odom", "/odom"),
                ("~/odometry", "/odom"),
                ("~/tf_odometry", "/tf"),
            ),
        )
    )
    if temporary is not None:
        nodes.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[OpaqueFunction(function=lambda _: temporary.cleanup())]
                )
            )
        )
    if LaunchConfiguration("use_navigation").perform(context) == "true":
        nodes.append(
            IncludeLaunchDescription(
                str(
                    Path(get_package_share_directory("rby1_mujoco_nav"))
                    / "launch/navigation.launch.py"
                ),
                launch_arguments={"robot_model": config["base_model"]}.items(),
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument(
                "robot_model",
                default_value="a",
                choices=list(ROBOT_MODELS),
                description="RBY1 model",
            ),
            DeclareLaunchArgument(
                "robot_version",
                default_value="v1.2",
                choices=sorted(
                    {v for model in ROBOT_MODELS.values() for v in model["versions"]}
                ),
                description="RBY1 model version",
            ),
            DeclareLaunchArgument(
                "hand_model",
                default_value="wuji_hand",
                choices=["wuji_hand", "wuji_hand2_beta1", "wuji_hand2_beta2"],
                description="Hand model for a_wuji/m_wuji, used on both sides",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value="auto",
                description="Controller YAML, or 'auto' to select by robot_model",
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
            DeclareLaunchArgument(
                "use_navigation",
                default_value="true",
                choices=["true", "false"],
                description="Start Nav2 for mobile-base models",
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Nav2 node namespace"
            ),
            DeclareLaunchArgument(
                "params_file", default_value="", description="Nav2 parameter YAML"
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Stamped velocity command output topic",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
