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
from ament_index_python.packages import get_package_share_directory

root = Path(__file__).resolve().parents[1]
compose = run_path(str(root / "urdf/compose_wuji.py"))["compose_wuji"]
for arm, hand, side in (("rb5_850e", "wuji_hand", "right"), ("rb5_850e", "wuji_hand2_beta1", "left"), ("rb20_1900es_u", "wuji_hand2_beta2", "right")):
    package = "wuji_description" if hand == "wuji_hand" else f"{hand}_description"
    xml = xacro.process_file(str(root / "urdf/rb_mujoco.urdf.xacro"),
                             mappings={"robot_model": arm, "headless": "true"}).toxml()
    with TemporaryDirectory() as directory:
        xml, controllers = compose(xml, root, Path(get_package_share_directory(package)),
                                   arm, hand, side, Path(directory))
        robot = ET.fromstring(xml)
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
