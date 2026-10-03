#!/usr/bin/env python3
"""Check hand composition, initial clearance and actuator tracking."""
import xml.etree.ElementTree as ET
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
import xacro
import yaml

root = Path(__file__).resolve().parents[1]
compose = run_path(str(root / "urdf/compose_wuji.py"))["compose_wuji"]
cases = (("rb5_850e", "wuji_hand"), ("rb5_850e", "wuji_hand2_beta1"),
         ("rb20_1900es_u", "wuji_hand2_beta2"))
for arm, hand, side in ((arm, hand, side) for arm, hand in cases
                        for side in ("left", "right")):
    xml = xacro.process_file(str(root / "urdf/rb_mujoco.urdf.xacro"),
                             mappings={"robot_model": arm, "headless": "true"}).toxml()
    with TemporaryDirectory() as directory:
        xml, controllers = compose(xml, root,
                                   arm, hand, side, Path(directory))
        robot = ET.fromstring(xml)
        for mesh in robot.findall(".//mesh"):
            package, relative = mesh.get("filename").removeprefix(
                "package://").split("/", 1)
            assert package == "rb_mujoco_description", mesh.get("filename")
            assert (root / relative).is_file(), relative
        if hand != "wuji_hand":
            mount = robot.find(f"joint[@name='{side}_hand_base_mount_joint']")
            assert mount.find("origin").get("xyz") == "0 0 0"
            assert len(robot.find(f"link[@name='{side}_hand_base']")) == 0
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
        data.qpos[:6] = [0.2, -0.3, 0.4, -0.2, 0.3, -0.1]
        data.ctrl[:] = data.qpos
        mujoco.mj_forward(model, data)
        assert data.ncon == 0, (arm, hand)
        data.ctrl[-1] += 0.15
        mujoco.mj_step(model, data, 1000)
        assert np.isfinite(data.qpos).all() and abs(
            data.qpos[-1] - data.ctrl[-1]) < 0.05, hand
print("PASS Wuji compositions: control mapping, initial clearance and hand tracking")
