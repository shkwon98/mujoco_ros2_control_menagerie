#!/usr/bin/env python3
"""Check official RB visuals, ROS kinematics and simulated joint tracking."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml


ROOT = Path(__file__).resolve().parents[1]
MODELS = (
    "rb1_500es_u", "rb3_730es_u", "rb3_1200e", "rb3_1200e_u",
    "rb5_850e", "rb5_850e_u", "rb6_1700e_u", "rb10_1300e", "rb10_1300e_u",
    "rb16_900e", "rb16_900e_u", "rb20_1800e_u", "rb20_1900es_u", "rb30_1400es_u",
)
JOINTS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")


class ModelTest(unittest.TestCase):
    def test_default_scene_frames_the_complete_robot(self):
        for name in MODELS:
            with self.subTest(model=name):
                model = mujoco.MjModel.from_xml_path(str(ROOT / "mjcf" / f"{name}.xml"))
                camera = mujoco.MjvCamera()
                mujoco.mjv_defaultFreeCamera(model, camera)
                self.assertEqual((camera.azimuth, camera.elevation), (140, -20))
                np.testing.assert_allclose(model.vis.headlight.diffuse, [0.6] * 3)
                np.testing.assert_allclose(model.vis.headlight.ambient, [0.3] * 3)
                np.testing.assert_allclose(model.vis.headlight.specular, [0] * 3)
                self.assertTrue((model.tex_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX).any())
                self.assertGreaterEqual(model.geom("floor").matid, 0)
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                bounds = []
                for geom in np.flatnonzero(model.geom_group == 2):
                    mesh = model.geom_dataid[geom]
                    start = model.mesh_vertadr[mesh]
                    local = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
                    world = local @ data.geom_xmat[geom].reshape(3, 3).T
                    world += data.geom_xpos[geom]
                    bounds.extend([world.min(axis=0), world.max(axis=0)])
                bounds = np.vstack(bounds)
                low, high = bounds.min(axis=0), bounds.max(axis=0)
                np.testing.assert_allclose(camera.lookat, (low + high) / 2, atol=1e-6)
                self.assertGreater(camera.distance, np.linalg.norm(high - low))

    def test_visuals_use_rb5_finish_and_preserve_link_coordinates(self):
        palette = np.array([
            [0.8, 0.8, 0.8, 1], [0.3, 0.3, 0.3, 1], [0.1, 0.1, 0.1, 1],
            [0.0722719, 0.0908417, 0.0975874, 1],  # RB5 base connector
        ])
        # Official extents in metres, including the DAE's link-local node transforms.
        extents = {
            ("rb5_850e", 2): [
                [-0.059861, -0.21819398, -0.0618],
                [0.059861, -0.06206795, 0.48680001],
            ],
            ("rb3_1200e", 3): [
                [-0.05899995, 0.07382284946, -0.06097431269],
                [0.05899995, 0.20464790750, 0.56374338218],
            ],
        }
        for name in MODELS:
            with self.subTest(model=name):
                model = mujoco.MjModel.from_xml_path(str(ROOT / "mjcf" / f"{name}.xml"))
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                for link in range(7):
                    body = model.body(f"link{link}").id
                    geoms = np.flatnonzero((model.geom_bodyid == body)
                                           & (model.geom_group == 2))
                    self.assertGreater(len(geoms), 0, f"link{link} has no visual mesh")
                    np.testing.assert_array_equal(model.geom_contype[geoms], 0)
                    np.testing.assert_array_equal(model.geom_conaffinity[geoms], 0)
                    self.assertTrue((model.geom_matid[geoms] >= 0).all())
                    actual = model.mat_rgba[model.geom_matid[geoms]]
                    for color in actual:
                        self.assertTrue(np.isclose(palette, color, atol=1e-6).all(axis=1).any(),
                                        f"link{link} retains a CAD color: {color}")
                    self.assertTrue(np.isclose(actual, palette[0], atol=1e-6)
                                    .all(axis=1).any(), f"link{link} has no silver housing")
                    if 1 <= link <= 5:
                        self.assertTrue(np.isclose(actual, palette[1], atol=1e-6)
                                        .all(axis=1).any(), f"link{link} has no gray cover")
                    for geom in geoms:
                        mesh = model.geom_dataid[geom]
                        # These upper-arm covers were exported with a silver CAD color.
                        if (name in ("rb20_1900es_u", "rb30_1400es_u")
                                and model.mesh(mesh).name == "link4_visual_3"):
                            np.testing.assert_allclose(model.mat_rgba[model.geom_matid[geom]],
                                                       palette[1], atol=1e-6)
                    if name == "rb1_500es_u":
                        # RB1 paint boundaries may split triangles, but not alter its surface.
                        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
                        source = ET.parse(ROOT / "meshes" / name / "visual"
                                          / f"link{link}.dae").getroot()
                        points = np.fromstring(source.findtext(
                            ".//c:source[@id='link" + str(link) + "-mesh-positions']"
                            "/c:float_array", namespaces=ns), sep=" ").reshape(-1, 3)
                        triangles = source.find(".//c:triangles", ns)
                        stride = 1 + max(int(i.get("offset"))
                                         for i in triangles.findall("c:input", ns))
                        faces = np.fromstring(triangles.findtext("c:p", namespaces=ns),
                                              sep=" ", dtype=int).reshape(-1, 3, stride)[:, :, 0]
                        edges = points[faces[:, 1:]] - points[faces[:, :1]]
                        expected_area = np.linalg.norm(np.cross(edges[:, 0], edges[:, 1]),
                                                       axis=1).sum() / 2
                        actual_area = 0.0
                        for geom in geoms:
                            mesh = model.geom_dataid[geom]
                            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                            points = model.mesh_vert[start:start + count]
                            start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                            faces = model.mesh_face[start:start + count]
                            edges = points[faces[:, 1:]] - points[faces[:, :1]]
                            actual_area += np.linalg.norm(np.cross(edges[:, 0], edges[:, 1]),
                                                          axis=1).sum() / 2
                        np.testing.assert_allclose(actual_area, expected_area, rtol=1e-5)
                    if (name, link) in extents:
                        vertices = []
                        for geom in geoms:
                            mesh = model.geom_dataid[geom]
                            start = model.mesh_vertadr[mesh]
                            local = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
                            world = local @ data.geom_xmat[geom].reshape(3, 3).T
                            world += data.geom_xpos[geom]
                            vertices.append((world - data.xpos[body])
                                            @ data.xmat[body].reshape(3, 3))
                        vertices = np.vstack(vertices)
                        np.testing.assert_allclose(
                            [vertices.min(axis=0), vertices.max(axis=0)],
                            extents[name, link], atol=1e-6)

    def test_models_load_and_follow_the_ros_kinematics(self):
        wrapper = ROOT / "urdf" / "rbpodo_mujoco.urdf.xacro"
        self.assertTrue(wrapper.is_file(), "RB MuJoCo description is missing")
        controllers = yaml.safe_load(
            (ROOT / "config" / "ros2_control" / "rbpodo_controllers.yaml").read_text()
        )["/**"]
        self.assertEqual(
            controllers["arm_controller"]["ros__parameters"]["joints"], list(JOINTS)
        )
        for name in MODELS:
            with self.subTest(model=name):
                robot = ET.fromstring(xacro.process_file(
                    str(wrapper), mappings={"robot_model": name, "headless": "true"}
                ).toxml())
                control = robot.find("ros2_control")
                self.assertEqual(control.findtext("hardware/plugin"),
                                 "mujoco_ros2_control/MujocoSystemInterface")
                self.assertEqual([j.get("name") for j in control.findall("joint")],
                                 list(JOINTS))
                model = mujoco.MjModel.from_xml_path(control.findtext(
                    "hardware/param[@name='mujoco_model']"))
                self.assertEqual((model.nq, model.nv, model.nu), (6, 6, 6))
                self.assertEqual([model.joint(i).name for i in range(6)], list(JOINTS))
                data = mujoco.MjData(model)
                data.qpos[:] = [0.2, -0.3, 0.4, -0.2, 0.3, -0.1]
                data.ctrl[:] = data.qpos
                mujoco.mj_forward(model, data)
                transform = np.eye(4)
                for joint in robot.findall("joint"):
                    origin = joint.find("origin")
                    offset = np.eye(4)
                    offset[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                    offset[:3, :3] = Rotation.from_euler(
                        "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                    ).as_matrix()
                    transform = transform @ offset
                    if joint.get("type") == "revolute":
                        index = JOINTS.index(joint.get("name"))
                        rotation = np.eye(4)
                        rotation[:3, :3] = Rotation.from_rotvec(
                            np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                            * data.qpos[index]
                        ).as_matrix()
                        transform = transform @ rotation
                np.testing.assert_allclose(data.body("tcp").xpos,
                                           transform[:3, 3], atol=1e-7)
                np.testing.assert_allclose(data.body("tcp").xmat.reshape(3, 3),
                                           transform[:3, :3], atol=1e-7)
                target = data.qpos.copy()
                mujoco.mj_step(model, data, 500)
                self.assertTrue(np.isfinite(data.qpos).all())
                self.assertLess(np.max(np.abs(data.qpos - target)), 0.05)
                data.ctrl[0] += 0.1
                mujoco.mj_step(model, data, 1000)
                self.assertLess(abs(data.qpos[0] - target[0] - 0.1), 0.03)


if __name__ == "__main__":
    unittest.main()
