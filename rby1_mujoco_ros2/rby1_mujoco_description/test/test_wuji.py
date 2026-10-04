"""Check hand control mapping and agreement between URDF and MuJoCo mounts."""
import xml.etree.ElementTree as ET
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

root = Path(get_package_share_directory("rby1_mujoco_description"))
compose = run_path(root / "urdf/compose_hand2.py")["compose_hand2"]


def pose(xyz, rpy):
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return result


def fixed_transform(robot, child, parent):
    joints = {joint.find("child").get(
        "link"): joint for joint in robot.findall("joint")}
    transform = np.eye(4)
    while child != parent:
        joint = joints[child]
        assert joint.get("type") == "fixed"
        origin = joint.find("origin")
        transform = pose(np.fromstring(origin.get("xyz", "0 0 0"), sep=" "),
                         np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")) @ transform
        child = joint.find("parent").get("link")
    return transform


# Cover every supported version, including M v1.3's 66.384 mm mounting offset.
for base, version in (("a", "v1.0"), ("a", "v1.1"), ("a", "v1.2"),
                      ("m", "v1.0"), ("m", "v1.1"), ("m", "v1.2"), ("m", "v1.3")):
    for hand in ("wuji_hand", "wuji_hand2"):
        with TemporaryDirectory() as output:
            scene = root / f"mjcf/rby1{base}/model_act_{version}_wuji.xml"
            initial = root / f"config/initial_positions/rby1{base}_wuji.yaml"
            controllers = root / \
                f"config/ros2_control/rby1{base}_wuji_controllers.yaml"
            if hand != "wuji_hand":
                scene, initial, controllers = compose(
                    root, base, version, controllers, Path(output), hand_model=hand)
            model = mujoco.MjModel.from_xml_path(str(scene))
            robot = ET.fromstring(xacro.process_file(str(root / "urdf/rby1.urdf.xacro"), mappings={
                "robot_model": f"{base}_wuji", "robot_version": version, "hand_model": hand,
                "initial_positions_file": str(initial), "mujoco_model_file": str(scene),
                "headless": "true"}).toxml())
            if hand == "wuji_hand2":
                assert any("/wuji_hand2_beta2/" in mesh.get("filename")
                           for mesh in robot.findall(".//mesh"))
            controlled = {joint.get("name")
                          for joint in robot.findall("ros2_control/joint")}
            assert controlled == set(yaml.safe_load(
                Path(initial).read_text())["initial_positions"])
            assert model.nu == len(controlled)
            assert {model.joint(
                int(j)).name for j in model.actuator_trnid[:, 0]} == controlled
            config = yaml.safe_load(Path(controllers).read_text())["/**"]
            exclusions = {tuple(sorted((e.get("body1"), e.get("body2"))))
                          for e in ET.parse(scene).findall("contact/exclude")}
            assert not any(link.get("name").endswith("_hand_base")
                           for link in robot.findall("link"))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            offset = 0.066384 if base == "m" and version == "v1.3" else 0.0
            for side in ("left", "right"):
                flange = side + "_flange"
                mount = side + \
                    "_hand_docking_link" if hand == "wuji_hand" else side[0] + "_mount"
                attachment = next(j for j in robot.findall(
                    "joint") if j.find("child").get("link") == mount)
                assert attachment.find("parent").get("link") == flange
                np.testing.assert_allclose(fixed_transform(robot, flange, "ee_" + side),
                                           pose([0, 0, 0], [np.pi, 0, 0]),
                                           atol=1e-12)
                native = side + "_palm_link" if hand == "wuji_hand" else mount
                expected = pose([0, 0, offset], [0, 0, 0])
                if hand == "wuji_hand":
                    sign = 1 if side == "left" else -1
                    expected = expected @ pose([0,
                                               0, -0.02725], [np.pi, 0, np.pi])
                    expected = expected @ pose([0.00065, 0, 0.022],
                                               [0, 0, -sign * np.pi / 2])
                np.testing.assert_allclose(fixed_transform(
                    robot, native, "ee_" + side), expected, atol=1e-9)
                ee = data.body("EE_BODY_" + side[0].upper())
                native_body = data.body(native)
                rotation = ee.xmat.reshape(3, 3).T
                expected[2, 3] -= offset
                np.testing.assert_allclose(
                    rotation @ (native_body.xpos - ee.xpos), expected[:3, 3], atol=1e-9)
                np.testing.assert_allclose(
                    rotation @ native_body.xmat.reshape(3, 3), expected[:3, :3], atol=1e-9)
                flange_rotation = ee.xmat.reshape(
                    3, 3) @ fixed_transform(robot, flange, "ee_" + side)[:3, :3]
                native_rotation = native_body.xmat.reshape(3, 3)
                thumb = [0, -1 if side == "left" else 1,
                         0] if hand == "wuji_hand" else [1, 0, 0]
                fingers = [0, 0, 1] if hand == "wuji_hand" else [0, 0, -1]
                dorsum = [-1, 0, 0] if hand == "wuji_hand" else [0,
                                                                 1 if side == "left" else -1, 0]
                np.testing.assert_allclose(
                    flange_rotation.T @ native_rotation @ thumb, [1, 0, 0], atol=1e-9)
                np.testing.assert_allclose(
                    flange_rotation.T @ native_rotation @ fingers, [0, 0, 1], atol=1e-9)
                np.testing.assert_allclose(
                    native_rotation @ dorsum, [0, 1 if side == "left" else -1, 0], atol=1e-9)
                if hand != "wuji_hand":
                    official = ET.parse(
                        root / f"mjcf/wuji_hand2_beta2/{side}_with_mount.xml")
                    assert {tuple(sorted((e.get("body1"), e.get("body2"))))
                            for e in official.findall("contact/exclude")} <= exclusions
                    names = {j.get("name")
                             for j in official.findall(".//worldbody//joint")}
                    assert len(names) == 20
                    for suffix in ("controller", "joint_state_broadcaster"):
                        assert set(
                            config[f"hand_{side}_{suffix}"]["ros__parameters"]["joints"]) == names

print("PASS hand composition: thumb/finger flange axes, control mapping, contact exclusions and ROS/MuJoCo mounts")
