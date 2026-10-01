#!/usr/bin/env python3

"""Checks G2 collision geometry and preservation of the source visual materials."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = DESCRIPTION_ROOT / "urdf" / "g2.urdf"
MJCF_PATH = DESCRIPTION_ROOT / "mjcf" / "g2.xml"

BODY_LINKS = {
    "base_link",
    *(f"body_link{index}" for index in range(1, 6)),
    *(f"head_link{index}" for index in range(1, 4)),
    *(f"arm_{side}_link{index}" for side in ("l", "r") for index in range(1, 8)),
}
END_EFFECTOR_LINKS = {
    *(f"gripper_{side}_base_link" for side in ("l", "r")),
    *(
        f"gripper_{side}_{finger}_link"
        for side in ("l", "r")
        for finger in (
            "left_inner",
            "left_outer",
            "left_support",
            "right_inner",
            "right_outer",
            "right_support",
        )
    ),
}
WHEEL_LINKS = {
    "chassis_lwheel_front_link2",
    "chassis_lwheel_rear_link2",
    "chassis_rwheel_front_link2",
    "chassis_rwheel_rear_link2",
}
REQUIRED_COLLISION_LINKS = BODY_LINKS | END_EFFECTOR_LINKS | WHEEL_LINKS


class CollisionModelTest(unittest.TestCase):
    def test_mjcf_preserves_source_visual_colors(self) -> None:
        model = ET.parse(MJCF_PATH).getroot()
        assets = MJCF_PATH.parent / "assets" / "full"
        meshes = {
            mesh.get("name"): Path(mesh.get("file"))
            for mesh in model.findall("asset/mesh")
        }
        colors = {
            material.get("name"): tuple(map(float, material.get("rgba").split()))
            for material in model.findall("asset/material") if material.get("rgba")
        }

        for body in model.findall(".//body"):
            visuals = [geom for geom in body.findall("geom") if geom.get("group") == "1"]
            if not visuals:
                continue
            mesh_file = meshes[visuals[0].get("mesh")]
            link = mesh_file.stem if mesh_file.parent.name == "full" else mesh_file.parent.name
            with self.subTest(body=body.get("name")):
                if link == "swiftpicker_base_link":
                    source = ET.parse(assets / "swiftpicker_base_link.dae")
                    expected = [
                        tuple(map(float, color.text.split()))
                        for color in source.findall(".//{*}diffuse/{*}color")
                    ]
                else:
                    source = ET.parse(assets / link / f"{link}.xml")
                    source_colors = {
                        material.get("name"): tuple(map(float, material.get("rgba").split()))
                        for material in source.findall("asset/material")
                    }
                    expected = [
                        source_colors[geom.get("material")]
                        for geom in source.findall("worldbody/body/geom[@class='visual']")
                    ]
                self.assertEqual(
                    sorted(colors.get(geom.get("material"), ()) for geom in visuals),
                    sorted(expected),
                )
                for geom in visuals:
                    self.assertEqual(
                        (geom.get("contype"), geom.get("conaffinity"), geom.get("density")),
                        ("0", "0", "0"),
                    )

    def test_urdf_defines_standard_collision_geometry(self) -> None:
        robot = ET.parse(URDF_PATH).getroot()
        collision_links = {
            link.get("name") for link in robot.findall("link") if link.findall("collision")
        }

        self.assertEqual(REQUIRED_COLLISION_LINKS - collision_links, set())

    def test_mjcf_enables_collision_for_physical_links(self) -> None:
        model = ET.parse(MJCF_PATH).getroot()
        collision_links = {
            body.get("name")
            for body in model.findall(".//body")
            if any(
                geom.get("contype") != "0" or geom.get("conaffinity") != "0"
                for geom in body.findall("geom")
            )
        }

        self.assertEqual(REQUIRED_COLLISION_LINKS - collision_links, set())

    def test_rotated_arm_meshes_share_visual_and_collision_origins(self) -> None:
        robot = ET.parse(URDF_PATH).getroot()

        for side in ("l", "r"):
            for index in (2, 4):
                link = robot.find(f"./link[@name='arm_{side}_link{index}']")
                self.assertEqual(
                    link.find("visual/origin").attrib,
                    link.find("collision/origin").attrib,
                )


if __name__ == "__main__":
    unittest.main()
