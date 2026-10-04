"""Attach one Wuji hand directly to the RB flange."""
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def compose_wuji(robot_xml: str, description: Path,
                 robot_model: str, hand_model: str, side: str,
                 output: Path) -> tuple[str, str]:
    """Write the combined MJCF/controller YAML in the launch-owned directory.

    Mount coordinates are metres/radians in the +Z-outward flange frame.
    Both sides use the single flange's thumbward +X and fingerward +Z axes.
    At arm joint zero the right dorsum points up and the left dorsum points down.
    This is a simulation mount, not a model of a manufactured RB adapter.
    """
    if side not in ("left", "right") or hand_model not in (
        "wuji_hand", "wuji_hand2"
    ):
        raise ValueError("Unsupported Wuji hand model or side")
    asset_model = "wuji_hand2_beta2" if hand_model == "wuji_hand2" else "wuji_hand"
    variant = side if hand_model == "wuji_hand" else f"{side}_with_mount"
    hand = ET.parse(description / "urdf" / asset_model /
                    f"{variant}-ros.urdf").getroot()
    hand_mjcf = ET.parse(description / "mjcf" / asset_model /
                         f"{variant}.xml").getroot()
    source = description / "mjcf" / f"{robot_model}.xml"
    model = ET.parse(source).getroot()
    robot = ET.fromstring(robot_xml)
    hand_body = hand_mjcf.find("worldbody/body")
    names = [joint.get("name") for joint in hand_body.iter("joint")]
    joints = {joint.get("name"): joint for joint in hand.findall("joint")
              if joint.get("type") != "fixed"}
    if len(names) != 20 or set(names) != set(joints) or names != [
        actuator.get("joint") for actuator in hand_mjcf.findall("actuator/*")
    ]:
        raise ValueError(
            "Wuji URDF, MJCF and actuators must cover the same 20 joints")
    roots = {link.get("name") for link in hand.findall("link")} - {
        joint.find("child").get("link") for joint in hand.findall("joint")
    }
    if roots != {hand_body.get("name")}:
        raise ValueError("Wuji URDF and MJCF attachment roots differ")

    # Keep resource lookup at launch; generated models use install-space paths.
    for tree, directory in ((model, source.parent), (hand_mjcf, description / "mjcf" / asset_model)):
        compiler = tree.find("compiler")
        meshdir = directory / compiler.get("meshdir", "")
        for mesh in tree.findall("asset/mesh"):
            mesh.set("file", str((meshdir / mesh.get("file")).resolve()))
        compiler.attrib.pop("meshdir", None)
    assets = model.find("asset")
    assets.extend(hand_mjcf.findall("asset/*"))
    # Explicit hand defaults cannot change the arm's dynamics.
    defaults = hand_mjcf.find("default/joint").attrib
    for joint in hand_body.iter("joint"):
        for key, value in defaults.items():
            joint.attrib.setdefault(key, value)

    flange_origin = robot.find("joint[@name='flange_joint']/origin")
    flange = ET.SubElement(model.find(".//body[@name='tcp']"), "body", name="flange",
                           pos=flange_origin.get("xyz"), euler=flange_origin.get("rpy"))
    if hand_model == "wuji_hand":
        docking = ET.parse(
            description / "urdf/wuji_hand/docking-ros.urdf").find("link")
        docking.set("name", f"{side}_hand_docking_link")
        robot.append(docking)
        dock_body = ET.Element("body", name=docking.get("name"))
        inertial = docking.find("inertial")
        inertia = inertial.find("inertia")
        ET.SubElement(dock_body, "inertial", pos=inertial.find("origin").get("xyz"),
                      mass=inertial.find("mass").get("value"),
                      fullinertia=" ".join(inertia.get(key) for key in (
                          "ixx", "iyy", "izz", "ixy", "ixz", "iyz")))
        mesh = docking.find("visual/geometry/mesh")
        relative = mesh.get("filename").split("/", 3)[-1]
        ET.SubElement(assets, "mesh", name=docking.get("name"),
                      file=str(description / relative))
        for group, collide in ((2, "0"), (3, "1")):
            ET.SubElement(dock_body, "geom", type="mesh", mesh=docking.get("name"),
                          group=str(group), contype=collide, conaffinity=collide,
                          density="0", rgba="0.8 0.8 0.8 1")
        # Keep the docking-to-palm geometry; align both thumbs with flange +X.
        dock_rpy = "0 0 3.141592653589793"
        palm_rpy = "0 0 -1.5707963267948966" if side == "left" else "0 0 1.5707963267948966"
        mounts = [
            (flange, "0 0 0.02725",
             dock_rpy, dock_body),
            (dock_body, "0.00065 0 0.022",
             palm_rpy, hand_body),
        ]
    else:
        mounts = [(flange, "0 0 0",
                   "3.141592653589793 0 0", hand_body)]
    model.find("compiler").set("eulerseq", "XYZ")
    for parent, xyz, rpy, body in mounts:
        child = body.get("name")
        joint = ET.SubElement(
            robot, "joint", name=f"{child}_mount_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent.get("name"))
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
        body.set("pos", xyz)
        body.set("euler", rpy)
        parent.append(body)
    for element in hand:
        if element.tag in ("link", "joint", "material"):
            robot.append(element)
    # Preserve official contacts and keep collision overlays hidden by default.
    contacts = model.find("contact")
    contacts.extend(hand_mjcf.findall("contact/*"))
    colors = {}
    for geom in hand_body.iter("geom"):
        geom.set("group", "2" if geom.get("group") == "1" else "3")
        mesh = geom.get("mesh", "")
        color = ("0.1 0.1 0.1 1" if any(part in mesh for part in ("palm", "distal", "sensor"))
                 else "0.75 0.75 0.75 1")
        if hand_model == "wuji_hand" and "tip" in mesh:
            color = "0.95 0.95 0.95 1"
        geom.set("rgba", color)
        colors[mesh] = color
    for visual in hand.findall("link/visual"):
        mesh = Path(visual.find("geometry/mesh").get("filename")).stem
        visual.find("material/color").set("rgba", colors[mesh])
    model.find("actuator").extend(hand_mjcf.findall("actuator/*"))
    control = robot.find("ros2_control")
    initial = []
    for name in names:
        limit = joints[name].find("limit")
        value = max(float(limit.get("lower")), min(
            0.0, float(limit.get("upper"))))
        initial.append(value)
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="position")
        for interface in ("position", "velocity", "effort"):
            state = ET.SubElement(joint, "state_interface", name=interface)
            ET.SubElement(state, "param", name="initial_value").text = str(
                value if interface == "position" else 0.0)
    positions = " ".join(map(str, [0.0] * 6 + initial))
    ET.SubElement(ET.SubElement(model, "keyframe"), "key", name="home",
                  qpos=positions, ctrl=positions)
    model_file = output / "model.xml"
    ET.ElementTree(model).write(model_file, encoding="unicode")
    control.find("hardware/param[@name='mujoco_model']").text = str(model_file)
    controllers = yaml.safe_load(
        (description / "config/ros2_control/rb_controllers.yaml").read_text())
    config = controllers["/**"]
    broadcaster = f"hand_{side}_joint_state_broadcaster"
    controller = f"hand_{side}_controller"
    config["controller_manager"]["ros__parameters"].update({
        broadcaster: {"type": "joint_state_broadcaster/JointStateBroadcaster"},
        controller: {"type": "joint_trajectory_controller/JointTrajectoryController"},
    })
    # Separate lists avoid YAML aliases, which the ROS parameter parser rejects.
    config[broadcaster] = {"ros__parameters": {
        "joints": names.copy(), "interfaces": ["position", "velocity", "effort"],
    }}
    config[controller] = {"ros__parameters": {
        "joints": names.copy(), "command_interfaces": ["position"],
        "state_interfaces": ["position", "velocity"],
        "allow_nonzero_velocity_at_trajectory_end": True,
        "set_last_command_interface_value_as_state_on_activation": False,
    }}
    controllers_file = output / "controllers.yaml"
    controllers_file.write_text(yaml.safe_dump(controllers, sort_keys=False))
    return ET.tostring(robot, encoding="unicode"), str(controllers_file)
