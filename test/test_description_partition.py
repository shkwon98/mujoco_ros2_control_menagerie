"""Check partition ownership, attachment frames, materials and control separation."""
import xml.etree.ElementTree as ET
from pathlib import Path
from runpy import run_path

xml = '''<robot name="test">
<material name="silver"><color rgba="0.7 0.7 0.7 1"/></material>
<link name="base"/><link name="wrist"/><link name="finger"><visual/></link>
<joint name="arm" type="revolute"><parent link="base"/><child link="wrist"/></joint>
<joint name="hand" type="prismatic"><parent link="wrist"/><child link="finger"/></joint>
<ros2_control name="control" type="system"/></robot>'''
root = Path(__file__).parents[1]
for launch in sorted(root.glob("*/*_mujoco_bringup/launch/robot.launch.py")):
    split_description = run_path(str(launch))["split_description"]
    parts = {key: ET.fromstring(value) for key, value in
             split_description(xml, {"left": "wrist", "right": ""}).items()}
    assert set(parts) == {"body", "left"}
    assert [j.get("name") for j in parts["body"].findall("joint")] == ["arm"]
    assert [j.get("name") for j in parts["left"].findall("joint")] == ["hand"]
    assert parts["left"].find("link[@name='wrist']").find("visual") is None
    assert parts["left"].find("link[@name='finger']/visual") is not None
    assert all(p.find("ros2_control") is None and p.find(
        "material") is not None for p in parts.values())
    assert set(split_description(xml, {})) == {"body"}
    for roots in ({"left": "missing"}, {"left": "base", "right": "wrist"}):
        try:
            split_description(xml, roots)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Invalid partition accepted: {roots}")
print("All robot description partitions passed")
