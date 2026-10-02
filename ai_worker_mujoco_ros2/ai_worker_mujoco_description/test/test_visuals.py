"""Keep AI Worker shells and tires on their product material palette."""
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path(__file__).parents[1] / "mjcf"
for name in ("ffw_bg2", "ffw_bh5", "ffw_sg2", "ffw_sh5"):
    model = ET.parse(root / f"{name}.xml")
    for link in ("body_arm_assy", "head_link2"):
        for color in ("white", "black"):
            geom = model.find(f".//geom[@mesh='{link}_{color}']")
            assert geom is not None and geom.get("material") == color
            assert geom.get("class") == "visual"
    assert all(g.get("material")
               for g in model.findall(".//geom[@class='visual']"))
    for wheel in ("left_wheel", "right_wheel", "rear_wheel"):
        geom = model.find(f".//geom[@mesh='{wheel}']")
        assert geom is None or geom.get("material") == "black"
print("PASS AI Worker visual materials")
