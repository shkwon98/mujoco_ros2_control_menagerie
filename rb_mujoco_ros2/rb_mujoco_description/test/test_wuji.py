#!/usr/bin/env python3
"""Check hand composition, initial clearance and actuator tracking."""
import xml.etree.ElementTree as ET
from itertools import product
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
import xacro
import yaml
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
compose = run_path(str(root / "urdf/compose_wuji.py"))["compose_wuji"]
cases = (("rb5_850e", "wuji_hand"), ("rb5_850e", "wuji_hand2"),
         ("rb20_1900es_u", "wuji_hand2"))
collisions = []
for (arm, hand), side in product(cases, ("left", "right")):
    xml = xacro.process_file(str(root / "urdf/rb_mujoco.urdf.xacro"),
                             mappings={"robot_model": arm, "headless": "true"}).toxml()
    with TemporaryDirectory() as directory:
        xml, controllers = compose(xml, root,
                                   arm, hand, side, Path(directory))
        robot = ET.fromstring(xml)
        if hand == "wuji_hand2":
            assert any("/wuji_hand2_beta2/" in mesh.get("filename")
                       for mesh in robot.findall(".//mesh"))
        for mesh in robot.findall(".//mesh"):
            package, relative = mesh.get("filename").removeprefix(
                "package://").split("/", 1)
            assert package == "rb_mujoco_description", mesh.get("filename")
            assert (root / relative).is_file(), relative
        assert not any(link.get("name").endswith("_hand_base")
                       for link in robot.findall("link"))
        mount = f"{side}_hand_docking_link" if hand == "wuji_hand" else f"{side[0]}_mount"
        attachment = next(j for j in robot.findall("joint")
                          if j.find("child").get("link") == mount)
        assert attachment.find("parent").get("link") == "flange"
        assert {link.get("name") for link in robot.findall("link")
                if "flange" in link.get("name")} == {"flange"}
        control = robot.find("ros2_control")
        model = mujoco.MjModel.from_xml_path(
            control.findtext("hardware/param[@name='mujoco_model']"))
        assert (model.nq, model.nv, model.nu) == (26, 26, 26)
        names = [model.joint(i).name for i in range(model.nq)]
        assert [j.get("name") for j in control.findall("joint")] == names
        config = yaml.safe_load(Path(controllers).read_text())["/**"]
        assert not any(isinstance(event, yaml.AliasEvent)
                       for event in yaml.parse(Path(controllers).read_text()))
        assert config[f"hand_{side}_controller"]["ros__parameters"]["joints"] == names[6:]
        assert len(config["body_joint_state_broadcaster"]
                   ["ros__parameters"]["joints"]) == 6
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        assert np.all(data.qpos >= model.jnt_range[:, 0]) and np.all(
            data.qpos <= model.jnt_range[:, 1])
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.body("flange").xmat.reshape(3, 3),
                                   [[1, 0, 0], [0, 0, -1], [0, 1, 0]], atol=1e-12)
        flange_rotation = data.body("flange").xmat.reshape(3, 3)
        native_rotation = data.body(
            f"{side}_palm_link" if hand == "wuji_hand" else mount).xmat.reshape(3, 3)
        thumb = [0, -1 if side == "left" else 1,
                 0] if hand == "wuji_hand" else [1, 0, 0]
        fingers = [0, 0, 1] if hand == "wuji_hand" else [0, 0, -1]
        dorsum = [-1, 0, 0] if hand == "wuji_hand" else [0,
                                                         1 if side == "left" else -1, 0]
        np.testing.assert_allclose(flange_rotation.T @ native_rotation @ thumb,
                                   [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(
            flange_rotation.T @ native_rotation @ fingers, [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(native_rotation @ dorsum,
                                   [0, 0, -1 if side == "left" else 1], atol=1e-12)
        data.qpos[:6] = [0.2, -0.3, 0.4, -0.2, 0.3, -0.1]
        data.ctrl[:] = data.qpos
        mujoco.mj_forward(model, data)
        flange = data.body("flange")
        native = data.body(f"{side}_palm_link" if hand ==
                           "wuji_hand" else mount)
        rotation = flange.xmat.reshape(3, 3).T
        if hand == "wuji_hand":
            position = [-0.00065, 0, 0.04925]
            orientation = Rotation.from_euler(
                "z", np.pi / 2 if side == "left" else -np.pi / 2)
        else:
            position = [0, 0, 0]
            orientation = Rotation.from_euler("xyz", [np.pi, 0, 0])
        np.testing.assert_allclose(
            rotation @ (native.xpos - flange.xpos), position, atol=1e-12)
        np.testing.assert_allclose(
            rotation @ native.xmat.reshape(3, 3), orientation.as_matrix(), atol=1e-12)
        if data.ncon:
            collisions.append((arm, hand, side, data.ncon))
        data.ctrl[-1] += 0.15
        mujoco.mj_step(model, data, 1000)
        assert np.isfinite(data.qpos).all() and abs(
            data.qpos[-1] - data.ctrl[-1]) < 0.05, hand
print("PASS Wuji compositions: thumb/finger flange axes, control mapping and hand tracking")
assert not collisions, ("Hand clearance failures", collisions)
