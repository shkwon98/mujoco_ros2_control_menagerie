#!/usr/bin/env python3
"""Compare gripper geometry through MuJoCo's native URDF and MJCF loaders."""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import xacro

root = Path(__file__).resolve().parents[1]
robot = ET.fromstring(xacro.process_file(
    str(root / "urdf/mobile_aloha_mujoco.urdf.xacro")).toxml())
for mesh in robot.findall(".//mesh"):
    mesh.set("filename", mesh.get("filename").replace(
        "package://mobile_aloha_mujoco_description", str(root)))
ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler",
              fusestatic="false", discardvisual="false", strippath="false")
models = (mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode")),
          mujoco.MjModel.from_xml_path(str(root / "mjcf/mobile_aloha.xml")))
states = [mujoco.MjData(model) for model in models]


def bounds(model, data, geom, attachment):
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    local = (world - data.body(attachment).xpos) @ data.body(attachment).xmat.reshape(3, 3)
    return [local.min(0), local.max(0)]


for roll, opening in ((0.0, 0.002), (0.8, 0.037)):
    for model, data in zip(models, states):
        for side in ("left", "right"):
            for suffix, value in (("wrist_rotate", roll), ("left_finger", opening), ("right_finger", opening)):
                data.qpos[model.joint(f"{side}_{suffix}").qposadr[0]] = value
        mujoco.mj_forward(model, data)
    for side in ("left", "right"):
        for mesh in ("vx300s_7_gripper_prop", "vx300s_7_gripper_bar",
                     "vx300s_8_custom_finger_left", "vx300s_8_custom_finger_right"):
            for groups in ((1, 2), (0, 3)):
                boxes = []
                for model, data, group in zip(models, states, groups):
                    geoms = [g for g in range(model.ngeom) if model.geom_group[g] == group
                             and model.body(model.geom_bodyid[g]).name.startswith(f"{side}_")
                             and model.geom_dataid[g] == model.mesh(mesh).id]
                    assert len(geoms) == 1, (side, mesh, group)
                    boxes.append(bounds(model, data, geoms[0], f"{side}_gripper_link"))
                np.testing.assert_allclose(*boxes, atol=1e-6, rtol=0, err_msg=f"{side} {mesh}")
print("PASS VX300S gripper geometry at closed and open positions with wrist rotation")
