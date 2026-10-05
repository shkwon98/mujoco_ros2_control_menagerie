#!/usr/bin/env python3
"""Check PiPER supports and mesh poses using MuJoCo's native URDF loader."""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import xacro

root = Path(__file__).resolve().parents[1]
robot = ET.fromstring(xacro.process_file(
    str(root / "urdf/mobile_aloha_piper.urdf.xacro")).toxml())
for mesh in robot.findall(".//mesh"):
    mesh.set("filename", mesh.get("filename").replace(
        "package://mobile_aloha_mujoco_description", str(root)))
ET.SubElement(ET.SubElement(robot, "mujoco"), "compiler",
              fusestatic="false", discardvisual="false", strippath="false")
models = (mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode")),
          mujoco.MjModel.from_xml_path(str(root / "mjcf/mobile_aloha_piper.xml")))
states = [mujoco.MjData(model) for model in models]
for model, data in zip(models, states):
    mujoco.mj_forward(model, data)


# Model zero means horizontal tool on all followers/leaders; physical native limits stay fixed.
delta = 0.0872495900012399
for model, data in zip(models, states):
    for prefix in ("fl", "fr", "bl", "br"):
        rotation = data.body("base_link").xmat.reshape(3, 3).T @ data.body(f"{prefix}_link6").xmat.reshape(3, 3)
        np.testing.assert_allclose(rotation[:, 2], [1, 0, 0], atol=2e-5, rtol=0)
        joint = model.joint(f"{prefix}_joint5")
        np.testing.assert_allclose(model.jnt_range[joint.id] + delta, [-1.22, 1.22], atol=1e-12)
# Startup keeps the old native joint5=0 physical posture after re-zeroing.
home = models[1].key_qpos[0]
for prefix in ("fl", "fr", "bl", "br"):
    assert abs(home[models[1].joint(f"{prefix}_joint5").qposadr[0]] + delta) < 1e-12


def vertices(model, data, geom, attachment):
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    local = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    world = local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    return (world - data.body(attachment).xpos) @ data.body(attachment).xmat.reshape(3, 3)


model, data = models[1], states[1]
frame = np.concatenate([vertices(model, data, model.geom(f"mobile_aloha_body_{i}").id,
                                 "base_link") for i in range(7)])
for prefix in ("fl", "fr", "bl", "br"):
    mount = (data.body(f"{prefix}_base_link").xpos - data.body("base_link").xpos) @ data.body("base_link").xmat.reshape(3, 3)
    distance = np.linalg.norm(frame - mount, axis=1)
    assert distance.min() < 0.05 and abs(frame[distance.argmin(), 2] - mount[2]) < 0.001, prefix

for angle, opening in ((0, 0), (0.4, 0.015)):
    for model, data in zip(models, states):
        for prefix in ("fl", "fr", "bl", "br"):
            for number, position in ((6, angle), (7, opening), (8, -opening)):
                data.qpos[model.joint(f"{prefix}_joint{number}").qposadr[0]] = position
        mujoco.mj_forward(model, data)
    pairs = [("body_link", "frame", "base_link", f"body_{i}", f"mobile_aloha_body_{i}") for i in range(7)]
    pairs.extend((f"{prefix}_link{n}", f"{prefix}_link{n}", f"{prefix}_link5",
                  f"{'follower' if prefix.startswith('f') else 'leader'}_link{n}",
                  f"{prefix}_link{n}")
                 for prefix in ("fl", "fr", "bl", "br") for n in (6, 7, 8))
    for urdf_body, mjcf_body, attachment, urdf_mesh, mjcf_mesh in pairs:
        boxes = []
        for model, data, mesh, body in zip(models, states, (urdf_mesh, mjcf_mesh), (urdf_body, mjcf_body)):
            geoms = np.flatnonzero((model.geom_bodyid == model.body(body).id)
                                   & (model.geom_contype == 0) & (model.geom_conaffinity == 0)
                                   & (model.geom_dataid == model.mesh(mesh).id))
            assert len(geoms) == 1, (body, mesh)
            points = vertices(model, data, geoms[0], attachment)
            boxes.append([points.min(0), points.max(0)])
        np.testing.assert_allclose(*boxes, atol=1e-5, rtol=0, err_msg=body)
print("PASS PiPER supports and ROS/MuJoCo mesh coordinates")
