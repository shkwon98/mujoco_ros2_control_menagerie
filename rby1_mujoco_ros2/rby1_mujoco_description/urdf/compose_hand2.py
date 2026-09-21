"""Compose official Hand2 assets with the existing RBY1 Wuji simulation."""
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def compose_hand2(description: Path, base_model: str, version: str,
                  controllers_file: Path, output: Path,
                  hand_model: str = "wuji_hand2_beta1") -> tuple[str, str, str]:
    """Write MJCF, initial positions and controllers into the launch-owned directory.

    Keep the RBY1 body and controller tuning; replace only Hand1 geometry, joints
    and actuators, including the official internal collision exclusions.
    Hand2 joint units are radians and its actuator gains are the
    official simulation gains, not the hardware's current-control gains.
    """
    if hand_model not in ("wuji_hand2_beta1", "wuji_hand2_beta2"):
        raise ValueError(f"Unsupported Hand2 model: {hand_model}")
    source = description / "mjcf" / f"rby1{base_model}"
    body = ET.parse(source / f"rby1_{version}_wuji.xml")
    model = ET.parse(source / f"model_act_{version}_wuji.xml")
    positions = yaml.safe_load((description / "config" / "initial_positions" /
                                f"rby1{base_model}_wuji.yaml").read_text())
    controllers = yaml.safe_load(controllers_file.read_text())

    # Preserve relative references inside existing RBY1 mesh/geometry includes.
    (output / "assets").symlink_to(source / "assets", target_is_directory=True)
    assets = body.find("asset")
    assets.remove(assets.find(
        "include[@file='./assets/wuji_hand/assets.xml']"))
    actuators = model.find("actuator")
    actuators.remove(actuators.find(
        "include[@file='./assets/wuji_hand/actuators.xml']"))

    contacts = model.find("contact")
    if contacts is None:
        contacts = ET.SubElement(model.getroot(), "contact")

    for side in ("left", "right"):
        hand = ET.parse(description / "mjcf" / hand_model / f"{side}_with_mount.xml")
        geometry = ET.parse(description / "urdf" / hand_model /
                            f"{side}_with_mount-ros.urdf")
        names = [j.get("name") for j in geometry.findall(
            "joint") if j.get("type") != "fixed"]
        if len(names) != 20 or set(names) != {j.get("name") for j in hand.findall(".//worldbody//joint")}:
            raise ValueError("Hand2 URDF and MJCF joint names do not match")
        for joint in geometry.findall("joint"):
            if joint.get("name") in names:
                limit = joint.find("limit")
                if not float(limit.get("lower")) <= 0 <= float(limit.get("upper")):
                    raise ValueError(
                        "Hand2 zero initial position is outside the joint limits")
        initial = positions["initial_positions"]
        for name in list(initial):
            if name.startswith(f"{side}_finger"):
                del initial[name]
        initial.update(dict.fromkeys(names, 0.0))
        # A custom controller YAML keeps its tuning; names follow the selected hand.
        for controller in (f"hand_{side}_controller", f"hand_{side}_joint_state_broadcaster"):
            controllers["/**"][controller]["ros__parameters"]["joints"] = names.copy()

        meshdir = description / "mjcf" / hand_model / hand.find("compiler").get("meshdir")
        for mesh in hand.findall("asset/mesh"):
            mesh.set("file", str((meshdir / mesh.get("file")).resolve()))
            assets.append(mesh)
        mount = hand.find("worldbody/body")
        # RBY1's joint defaults include large damping; use the hand's own defaults.
        defaults = hand.find("default/joint").attrib
        for joint in mount.iter("joint"):
            for name, value in defaults.items():
                if name not in joint.attrib:
                    joint.set(name, value)
        for parent in body.iter("body"):
            include = parent.find(
                f"include[@file='./assets/wuji_hand/{side}_body.xml']")
            if include is not None:
                parent.remove(include)
                parent.append(mount)
                break
        else:
            raise ValueError(f"Missing RBY1 {side} hand attachment")
        actuators.extend(hand.findall("actuator/*"))
        # Keep the official wrist/finger exclusions when attaching the hand.
        contacts.extend(hand.findall("contact/*"))

    model.find("include").set("file", "body.xml")
    paths = [
        output / name for name in ("model.xml", "initial_positions.yaml", "controllers.yaml")]
    body.write(output / "body.xml", encoding="unicode")
    model.write(paths[0], encoding="unicode")
    paths[1].write_text(yaml.safe_dump(positions, sort_keys=False))
    paths[2].write_text(yaml.safe_dump(controllers, sort_keys=False))
    return tuple(map(str, paths))
