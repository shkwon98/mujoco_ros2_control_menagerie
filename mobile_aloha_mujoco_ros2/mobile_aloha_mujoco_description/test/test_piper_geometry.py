#!/usr/bin/env python3
"""Check Mobile ALOHA supports and official end-effector geometry."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import xacro


ROOT = Path(__file__).resolve().parents[1]


def stl_vertices(path):
    triangles = np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2"),
    ])
    return np.frombuffer(path.read_bytes(), dtype=triangles, offset=84)[
        "vertices"
    ].reshape(-1, 3)


def origin_transform(joint):
    origin = joint.find("origin")
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
    quat, matrix = np.empty(4), np.empty(9)
    mujoco.mju_euler2Quat(quat, rpy, "XYZ")  # URDF uses fixed-axis roll/pitch/yaw.
    mujoco.mju_quat2Mat(matrix, quat)
    return xyz, matrix.reshape(3, 3)


class PiperGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(
            str(ROOT / "mjcf" / "mobile_aloha_piper.xml")
        )
        cls.urdf = ET.fromstring(xacro.process_file(
            str(ROOT / "urdf" / "mobile_aloha_piper.urdf.xacro")
        ).toxml())

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

    def geom_vertices(self, geom):
        mesh = self.model.geom_dataid[geom]
        start = self.model.mesh_vertadr[mesh]
        local = self.model.mesh_vert[start:start + self.model.mesh_vertnum[mesh]]
        return local @ self.data.geom_xmat[geom].reshape(3, 3).T + self.data.geom_xpos[geom]

    def test_all_four_arm_bases_touch_the_support_plates(self):
        frame = np.concatenate([
            self.geom_vertices(self.model.geom(f"mobile_aloha_body_{i}").id)
            for i in range(7)
        ])
        for prefix in ("fl", "fr", "bl", "br"):
            with self.subTest(arm=prefix):
                mount = self.data.xpos[self.model.body(f"{prefix}_base_link").id]
                # The plate has a central hole; its rim is within 50 mm of the mount.
                distance = np.linalg.norm(frame - mount, axis=1)
                nearest = frame[distance.argmin()]
                self.assertLess(distance.min(), 0.05)
                self.assertLess(abs(nearest[2] - mount[2]), 0.001)

    def test_ros_and_mujoco_body_visuals_agree(self):
        base = self.model.body("base_link").id
        base_xyz = self.data.xpos[base]
        base_rotation = self.data.xmat[base].reshape(3, 3)
        body_xyz, body_rotation = origin_transform(
            self.urdf.find("joint[@name='body_joint']")
        )
        for i, visual in enumerate(self.urdf.findall("link[@name='body_link']/visual")):
            with self.subTest(mesh=i):
                xyz, rotation = origin_transform(visual)
                vertices = stl_vertices(ROOT / "mjcf" / "official" / f"body_{i}.STL")
                expected = (vertices @ rotation.T + xyz) @ body_rotation.T + body_xyz
                expected = expected @ base_rotation.T + base_xyz
                actual = self.geom_vertices(self.model.geom(f"mobile_aloha_body_{i}").id)
                np.testing.assert_allclose(
                    [actual.min(0), actual.max(0)],
                    [expected.min(0), expected.max(0)], atol=1e-6,
                )

    def test_all_grippers_follow_the_official_wrist_frame(self):
        for angle, opening in ((0, 0), (0.4, 0.015)):
            for prefix in ("fl", "fr", "bl", "br"):
                with self.subTest(arm=prefix, angle=angle, opening=opening):
                    for number, position in ((6, angle), (7, opening), (8, -opening)):
                        joint = self.model.joint(f"{prefix}_joint{number}").id
                        self.data.qpos[self.model.jnt_qposadr[joint]] = position
                    mujoco.mj_forward(self.model, self.data)
                    # Use the native wrist as the common parent of both hand descriptions.
                    wrist = self.model.body(f"{prefix}_link5").id
                    xyz6, rotation6 = origin_transform(
                        self.urdf.find(f"joint[@name='{prefix}_joint6']")
                    )
                    wrist_rotation = self.data.xmat[wrist].reshape(3, 3)
                    origin6 = self.data.xpos[wrist] + wrist_rotation @ xyz6
                    rotation = np.array([
                        [np.cos(angle), -np.sin(angle), 0],
                        [np.sin(angle), np.cos(angle), 0], [0, 0, 1],
                    ])
                    rotation6 = wrist_rotation @ rotation6 @ rotation
                    for number in (6, 7, 8):
                        xyz, orientation = origin6, rotation6
                        if number != 6:
                            joint = self.urdf.find(f"joint[@name='{prefix}_joint{number}']")
                            offset, finger_rotation = origin_transform(joint)
                            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                            position = opening if number == 7 else -opening
                            xyz = origin6 + rotation6 @ offset
                            orientation = rotation6 @ finger_rotation
                            xyz = xyz + orientation @ (axis * position)
                        kind = "follower" if prefix.startswith("f") else "leader"
                        vertices = stl_vertices(
                            ROOT / "mjcf" / "piper" / "assets" / f"{kind}_link{number}.stl"
                        )
                        expected = vertices @ orientation.T + xyz
                        body = self.model.body(f"{prefix}_link{number}").id
                        geoms = np.flatnonzero(
                            (self.model.geom_bodyid == body) & (self.model.geom_group == 2)
                        )
                        self.assertEqual(len(geoms), 1)
                        actual = self.geom_vertices(geoms[0])
                        np.testing.assert_allclose(
                            [actual.min(0), actual.max(0)],
                            [expected.min(0), expected.max(0)], atol=1e-5,
                        )


if __name__ == "__main__":
    unittest.main()
