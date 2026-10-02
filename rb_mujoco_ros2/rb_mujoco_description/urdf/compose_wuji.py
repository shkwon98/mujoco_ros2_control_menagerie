"""Attach one Wuji hand to an RB TCP without changing the arm model."""
import xml.etree.ElementTree as ET
from math import pi
from pathlib import Path

import yaml


def compose_wuji(robot_xml: str, description: Path, hand_description: Path,
                 robot_model: str, hand_model: str, side: str,
                 output: Path) -> tuple[str, str]:
    """Write the combined MJCF/controller YAML in the launch-owned directory.

    Mount coordinates are metres/radians. The Wuji mount's negative Z axis
    points along the RB flange's negative Y axis. This is a simulation mount,
    not a model of a manufactured RB adapter.
    """
    if side not in ("left", "right") or hand_model not in (
        "wuji_hand", "wuji_hand2_beta1", "wuji_hand2_beta2"
    ):
        raise ValueError("Unsupported Wuji hand model or side")
    variant = side if hand_model == "wuji_hand" else f"{side}_with_mount"
    hand = ET.parse(hand_description / "urdf" /
                    f"{variant}-ros.urdf").getroot()
    hand_mjcf = ET.parse(hand_description / "mjcf" /
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
    for tree, directory in ((model, source.parent), (hand_mjcf, hand_description / "mjcf")):
        meshdir = directory / tree.find("compiler").get("meshdir", "")
        for mesh in tree.findall("asset/mesh"):
            mesh.set("file", str((meshdir / mesh.get("file")).resolve()))
        tree.find("compiler").attrib.pop("meshdir", None)
    assets = model.find("asset")
    assets.extend(hand_mjcf.findall("asset/*"))
    # Explicit hand defaults cannot change the arm's dynamics.
    defaults = hand_mjcf.find("default/joint").attrib
    for joint in hand_body.iter("joint"):
        for key, value in defaults.items():
            joint.attrib.setdefault(key, value)

    base = f"{side}_hand_base"
    base_link = ET.SubElement(robot, "link", name=base)
    base_body = ET.Element("body", name=base)
    mounts = [("tcp", base, "0 0 0", "-1.5707963267948966 0 0", base_body)]
    if hand_model == "wuji_hand":
        docking = ET.parse(
            hand_description / "attachment/impact-resistant-attachment/urdf/docking-ros.urdf").find("link")
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
                      file=str(hand_description / relative))
        for group, collide in ((2, "0"), (3, "1")):
            ET.SubElement(dock_body, "geom", type="mesh", mesh=docking.get("name"),
                          group=str(group), contype=collide, conaffinity=collide,
                          density="0", rgba="0.8 0.8 0.8 1")
        dock_rpy = ("3.141592653589793 0 2.356194490192345" if side == "left"
                    else "-3.141592653589793 0 -2.356194490192345")
        palm_rpy = "0 0 -1.5707963267948966" if side == "left" else "0 0 1.5707963267948966"
        mounts.extend([
            (base, docking.get("name"), "0 0 -0.02725",
             dock_rpy, dock_body),
            (docking.get("name"), hand_body.get("name"), "0.00065 0 0.022",
             palm_rpy, hand_body),
        ])
    else:
        # Nominal aluminium spacer clears the large RB20/RB30 wrist housings.
        # Dimensions and mass describe a simulation adapter, not a physical part.
        mounts[0] = ("tcp", base, "0 -0.02 0",
                     "-1.5707963267948966 0 0", base_body)
        radius, length = 0.025, 0.02
        mass = pi * radius**2 * length * 2700
        transverse = mass * (radius**2 / 4 + length**2 / 12)
        axial = mass * radius**2 / 2
        inertia = ET.SubElement(base_link, "inertial")
        ET.SubElement(inertia, "origin", xyz="0 0 0.01", rpy="0 0 0")
        ET.SubElement(inertia, "mass", value=str(mass))
        ET.SubElement(inertia, "inertia", ixx=str(transverse), iyy=str(transverse),
                      izz=str(axial), ixy="0", ixz="0", iyz="0")
        ET.SubElement(base_body, "inertial", pos="0 0 0.01", mass=str(mass),
                      diaginertia=f"{transverse} {transverse} {axial}")
        for tag, group, collide in (("visual", "2", "0"), ("collision", "3", "1")):
            element = ET.SubElement(base_link, tag)
            ET.SubElement(element, "origin", xyz="0 0 0.01", rpy="0 0 0")
            geometry = ET.SubElement(element, "geometry")
            ET.SubElement(geometry, "cylinder", radius=str(
                radius), length=str(length))
            if tag == "visual":
                ET.SubElement(ET.SubElement(element, "material", name="rb_wuji_spacer"),
                              "color", rgba="0.3 0.3 0.3 1")
            ET.SubElement(base_body, "geom", type="cylinder", size=f"{radius} {length / 2}",
                          pos="0 0 0.01", group=group, contype=collide,
                          conaffinity=collide, density="0", rgba="0.3 0.3 0.3 1")
        mounts.append((base, hand_body.get("name"),
                      "0 0 0", "0 0 0", hand_body))
    model.find("compiler").set("eulerseq", "XYZ")
    for parent, child, xyz, rpy, body in mounts:
        joint = ET.SubElement(
            robot, "joint", name=f"{child}_mount_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
        body.set("pos", xyz)
        body.set("euler", rpy)
        model.find(f".//body[@name='{parent}']").append(body)
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
        material = visual.find("material")
        mesh = Path(visual.find("geometry/mesh").get("filename")).stem
        material.find("color").set("rgba", colors[mesh])
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
