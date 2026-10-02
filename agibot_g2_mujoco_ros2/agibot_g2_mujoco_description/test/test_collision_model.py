#!/usr/bin/env python3
"""Keep physical collisions and source visual materials intact."""
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path(__file__).resolve().parents[1]
robot = ET.parse(root / "urdf/g2.urdf")
model = ET.parse(root / "mjcf/g2.xml")
required = {"base_link", *(f"body_link{i}" for i in range(1, 6)),
            *(f"head_link{i}" for i in range(1, 4)),
            *(f"arm_{side}_link{i}" for side in ("l", "r")
              for i in range(1, 8)),
            *(f"gripper_{side}_base_link" for side in ("l", "r")),
            *(f"gripper_{side}_{finger}_link" for side in ("l", "r")
              for finger in ("left_inner", "left_outer", "left_support", "right_inner", "right_outer", "right_support")),
            *(f"chassis_{side}wheel_{end}_link2" for side in ("l", "r") for end in ("front", "rear"))}
assert required <= {link.get("name") for link in robot.findall(
    "link") if link.find("collision") is not None}
assert required <= {body.get("name") for body in model.findall(".//body")
                    if any(g.get("contype") != "0" or g.get("conaffinity") != "0" for g in body.findall("geom"))}
for side in ("l", "r"):
    for i in (2, 4):
        link = robot.find(f"link[@name='arm_{side}_link{i}']")
        assert link.find(
            "visual/origin").attrib == link.find("collision/origin").attrib

meshes = {m.get("name"): Path(m.get("file"))
          for m in model.findall("asset/mesh")}
colors = {m.get("name"): m.get("rgba")
          for m in model.findall("asset/material")}
assets = root / "mjcf/assets/full"
for body in model.findall(".//body"):
    visuals = body.findall("geom[@group='1']")
    if not visuals:
        continue
    mesh = meshes[visuals[0].get("mesh")]
    link = mesh.stem if mesh.parent.name == "full" else mesh.parent.name
    if link == "swiftpicker_base_link":
        source = ET.parse(assets / "swiftpicker_base_link.dae")
        expected = [tuple(map(float, color.text.split()))
                    for color in source.findall(".//{*}diffuse/{*}color")]
    else:
        source = ET.parse(assets / link / f"{link}.xml")
        palette = {m.get("name"): m.get("rgba")
                   for m in source.findall("asset/material")}
        expected = [tuple(map(float, palette[g.get("material")].split()))
                    for g in source.findall("worldbody/body/geom[@class='visual']")]
    assert sorted(tuple(map(float, colors[g.get("material")].split()))
                  for g in visuals) == sorted(expected), link
    assert all((g.get("contype"), g.get("conaffinity"), g.get(
        "density")) == ("0", "0", "0") for g in visuals)
print("PASS G2 collision geometry and source colors")
