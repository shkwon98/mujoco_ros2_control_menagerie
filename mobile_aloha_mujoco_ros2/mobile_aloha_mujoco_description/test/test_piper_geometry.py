#!/usr/bin/env python3
"""Check PiPER support contact and agreement between URDF and MuJoCo meshes."""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import xacro

root = Path(__file__).resolve().parents[1]
model = mujoco.MjModel.from_xml_path(str(root / "mjcf/mobile_aloha_piper.xml"))
robot = ET.fromstring(xacro.process_file(
    str(root / "urdf/mobile_aloha_piper.urdf.xacro")).toxml())
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)


def stl_vertices(path):
    triangles = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(path.read_bytes(), dtype=triangles, offset=84)["vertices"].reshape(-1, 3)


def origin_transform(element):
    origin = element.find("origin")
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    quat, matrix = np.empty(4), np.empty(9)
    mujoco.mju_euler2Quat(quat, np.fromstring(
        origin.get("rpy", "0 0 0"), sep=" "), "XYZ")
    mujoco.mju_quat2Mat(matrix, quat)
    return xyz, matrix.reshape(3, 3)


def geom_vertices(geom):
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    local = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    return local @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]


frame = np.concatenate(
    [geom_vertices(model.geom(f"mobile_aloha_body_{i}").id) for i in range(7)])
for prefix in ("fl", "fr", "bl", "br"):
    mount = data.xpos[model.body(f"{prefix}_base_link").id]
    distance = np.linalg.norm(frame - mount, axis=1)
    # The plate has a central hole; its rim must touch the mount plane.
    assert distance.min() < 0.05 and abs(
        frame[distance.argmin(), 2] - mount[2]) < 0.001, prefix

base = model.body("base_link").id
base_xyz, base_rotation = data.xpos[base], data.xmat[base].reshape(3, 3)
body_xyz, body_rotation = origin_transform(
    robot.find("joint[@name='body_joint']"))
for i, visual in enumerate(robot.findall("link[@name='body_link']/visual")):
    xyz, rotation = origin_transform(visual)
    vertices = stl_vertices(root / "mjcf/official" / f"body_{i}.STL")
    expected = ((vertices @ rotation.T + xyz) @ body_rotation.T +
                body_xyz) @ base_rotation.T + base_xyz
    actual = geom_vertices(model.geom(f"mobile_aloha_body_{i}").id)
    np.testing.assert_allclose([actual.min(0), actual.max(0)], [
                               expected.min(0), expected.max(0)], atol=1e-6)

for angle, opening in ((0, 0), (0.4, 0.015)):
    for prefix in ("fl", "fr", "bl", "br"):
        for number, position in ((6, angle), (7, opening), (8, -opening)):
            data.qpos[model.jnt_qposadr[model.joint(
                f"{prefix}_joint{number}").id]] = position
        mujoco.mj_forward(model, data)
        wrist = model.body(f"{prefix}_link5").id
        xyz6, rotation6 = origin_transform(
            robot.find(f"joint[@name='{prefix}_joint6']"))
        wrist_rotation = data.xmat[wrist].reshape(3, 3)
        origin6 = data.xpos[wrist] + wrist_rotation @ xyz6
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        rotation6 = wrist_rotation @ rotation6 @ rotation
        for number in (6, 7, 8):
            xyz, orientation = origin6, rotation6
            if number != 6:
                joint = robot.find(f"joint[@name='{prefix}_joint{number}']")
                offset, finger_rotation = origin_transform(joint)
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                orientation = rotation6 @ finger_rotation
                xyz = origin6 + rotation6 @ offset + \
                    orientation @ (axis * (opening if number ==
                                   7 else -opening))
            kind = "follower" if prefix.startswith("f") else "leader"
            expected = stl_vertices(
                root / "mjcf/piper/assets" / f"{kind}_link{number}.stl") @ orientation.T + xyz
            body = model.body(f"{prefix}_link{number}").id
            geoms = np.flatnonzero(
                (model.geom_bodyid == body) & (model.geom_group == 2))
            assert len(geoms) == 1
            actual = geom_vertices(geoms[0])
            np.testing.assert_allclose([actual.min(0), actual.max(0)], [
                                       expected.min(0), expected.max(0)], atol=1e-5)
print("PASS PiPER supports and ROS/MuJoCo mesh coordinates")
