"""Check Hand2 composition, actuator mapping and official contact exclusions."""
import xml.etree.ElementTree as ET
from pathlib import Path
from runpy import run_path
from tempfile import TemporaryDirectory

import mujoco
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

root = Path(get_package_share_directory("rby1_mujoco_description"))
compose = run_path(root / "urdf/compose_hand2.py")["compose_hand2"]
for base, version, hand in (("a", "v1.2", "wuji_hand2_beta1"), ("m", "v1.3", "wuji_hand2_beta2")):
    with TemporaryDirectory() as output:
        model_path, initial_path, controllers_path = compose(
            root, base, version, root /
            f"config/ros2_control/rby1{base}_wuji_controllers.yaml",
            Path(output), hand_model=hand)
        model = mujoco.MjModel.from_xml_path(model_path)
        robot = ET.fromstring(xacro.process_file(str(root / "urdf/rby1.urdf.xacro"), mappings={
            "robot_model": f"{base}_wuji", "robot_version": version, "hand_model": hand,
            "initial_positions_file": initial_path, "mujoco_model_file": model_path, "headless": "true"}).toxml())
        controlled = {j.get("name")
                      for j in robot.findall("ros2_control/joint")}
        assert controlled == set(yaml.safe_load(
            Path(initial_path).read_text())["initial_positions"])
        assert model.nu == len(controlled) and {model.joint(
            int(j)).name for j in model.actuator_trnid[:, 0]} == controlled
        controllers = yaml.safe_load(Path(controllers_path).read_text())["/**"]
        exclusions = {tuple(sorted((e.get("body1"), e.get("body2"))))
                      for e in ET.parse(model_path).findall("contact/exclude")}
        for side in ("left", "right"):
            official = ET.parse(root / f"mjcf/{hand}/{side}_with_mount.xml")
            assert {tuple(sorted((e.get("body1"), e.get("body2"))))
                    for e in official.findall("contact/exclude")} <= exclusions
            names = {j.get("name")
                     for j in official.findall(".//worldbody//joint")}
            assert len(names) == 20
            for suffix in ("controller", "joint_state_broadcaster"):
                assert set(
                    controllers[f"hand_{side}_{suffix}"]["ros__parameters"]["joints"]) == names
print("PASS Hand2 composition and control mapping")
