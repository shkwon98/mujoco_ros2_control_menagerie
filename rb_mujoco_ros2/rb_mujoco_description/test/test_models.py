#!/usr/bin/env python3
"""Check RB model loading, ROS/MuJoCo kinematics and joint tracking."""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import xacro
import yaml
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
joints = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
controllers = yaml.safe_load(
    (root / "config/ros2_control/rb_controllers.yaml").read_text())["/**"]
assert controllers["arm_controller"]["ros__parameters"]["joints"] == list(
    joints)
palette = np.array([[0.8, 0.8, 0.8, 1], [0.3, 0.3, 0.3, 1], [0.1, 0.1, 0.1, 1],
                    [0.0722719, 0.0908417, 0.0975874, 1]])
for path in sorted((root / "mjcf").glob("rb*.xml")):
    robot = ET.fromstring(xacro.process_file(str(root / "urdf/rb_mujoco.urdf.xacro"),
                                             mappings={"robot_model": path.stem, "headless": "true"}).toxml())
    control = robot.find("ros2_control")
    assert control.findtext(
        "hardware/plugin") == "mujoco_ros2_control/MujocoSystemInterface"
    assert [j.get("name") for j in control.findall("joint")] == list(joints)
    model = mujoco.MjModel.from_xml_path(
        control.findtext("hardware/param[@name='mujoco_model']"))
    assert (model.nq, model.nv, model.nu) == (6, 6, 6)
    assert [model.joint(i).name for i in range(6)] == list(joints)
    limits = [robot.find(f"joint[@name='{name}']/limit") for name in joints]
    ranges = np.array([[float(limit.get("lower")), float(limit.get("upper"))]
                       for limit in limits])
    np.testing.assert_allclose(model.jnt_range, ranges, atol=1e-12)
    np.testing.assert_allclose(model.actuator_ctrlrange, ranges, atol=1e-12)
    visuals = np.flatnonzero(model.geom_group == 2)
    assert len(visuals) and (model.geom_contype[visuals] == 0).all() and (
        model.geom_conaffinity[visuals] == 0).all()
    assert (model.geom_matid[visuals] >= 0).all()
    for color in model.mat_rgba[model.geom_matid[visuals]]:
        assert np.isclose(
            palette, color, atol=1e-6).all(axis=1).any(), (path.stem, color)
    data = mujoco.MjData(model)
    data.qpos[:] = [0.2, -0.3, 0.4, -0.2, 0.3, -0.1]
    data.ctrl[:] = data.qpos
    mujoco.mj_forward(model, data)
    parents = {joint.find("child").get(
        "link"): joint for joint in robot.findall("joint")}
    chain, tip = [], "tcp"
    while tip in parents:
        joint = parents[tip]
        chain.append(joint)
        tip = joint.find("parent").get("link")
    transform = np.eye(4)
    for joint in reversed(chain):
        origin, offset = joint.find("origin"), np.eye(4)
        offset[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        offset[:3, :3] = Rotation.from_euler("xyz", np.fromstring(
            origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
        transform = transform @ offset
        if joint.get("type") == "revolute":
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                                                    * data.qpos[joints.index(joint.get("name"))]).as_matrix()
            transform = transform @ rotation
    np.testing.assert_allclose(
        data.body("tcp").xpos, transform[:3, 3], atol=1e-7)
    np.testing.assert_allclose(data.body("tcp").xmat.reshape(
        3, 3), transform[:3, :3], atol=1e-7)
    data.ctrl[0] += 0.1
    mujoco.mj_step(model, data, 1000)
    assert np.isfinite(data.qpos).all() and np.max(
        np.abs(data.qpos - data.ctrl)) < 0.05, path.stem
print("PASS all RB models: kinematics, materials and tracking")
