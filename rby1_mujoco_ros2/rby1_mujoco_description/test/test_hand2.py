"""Check the Hand2 composition through the installed model and controller interfaces."""
import xml.etree.ElementTree as ET
from pathlib import Path
from runpy import run_path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

mujoco = pytest.importorskip("mujoco")

DESCRIPTION = Path(get_package_share_directory("rby1_mujoco_description"))
COMPOSE = run_path(DESCRIPTION / "urdf" / "compose_hand2.py")["compose_hand2"]


@pytest.mark.parametrize("hand_model", ["wuji_hand2_beta1", "wuji_hand2_beta2"])
@pytest.mark.parametrize("base,version", [("a", "v1.2"), ("m", "v1.3")])
def test_hand2_model_and_controllers(tmp_path, base, version, hand_model):
    model_path, initial_path, controllers_path = COMPOSE(
        DESCRIPTION, base, version,
        DESCRIPTION / "config" / "ros2_control" /
        f"rby1{base}_wuji_controllers.yaml",
        tmp_path, hand_model=hand_model,
    )
    robot = ET.fromstring(xacro.process_file(str(DESCRIPTION / "urdf" / "rby1.urdf.xacro"),
                                             mappings={"robot_model": f"{base}_wuji", "robot_version": version,
                                                       "hand_model": hand_model, "initial_positions_file": initial_path,
                                                       "mujoco_model_file": model_path, "headless": "true"}).toxml())
    model = mujoco.MjModel.from_xml_path(model_path)
    initial = yaml.safe_load(Path(initial_path).read_text())[
        "initial_positions"]
    controller_text = Path(controllers_path).read_text()
    assert not any(isinstance(token, (yaml.AliasToken, yaml.AnchorToken))
                   for token in yaml.scan(controller_text))
    controllers = yaml.safe_load(controller_text)["/**"]
    controlled = {j.get("name") for j in robot.findall("ros2_control/joint")}
    assert controlled == set(initial)
    assert model.nu == len(controlled)
    assert {model.joint(
        int(j)).name for j in model.actuator_trnid[:, 0]} == controlled
    hand_mjcf = DESCRIPTION / "mjcf" / hand_model
    expected_exclusions = set()
    for side in ("left", "right"):
        official = ET.parse(hand_mjcf / f"{side}_with_mount.xml")
        expected_exclusions.update(
            tuple(sorted((e.get("body1"), e.get("body2"))))
            for e in official.findall("contact/exclude")
        )
        hand = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / "urdf" / "wuji_hand2" / "hand.urdf.xacro"),
            mappings={"side": side, "hand_model": hand_model}).toxml())
        names = {j.get("name") for j in hand.findall(
            "joint") if j.get("type") != "fixed"}
        assert len(names) == 20
        for controller in (f"hand_{side}_controller", f"hand_{side}_joint_state_broadcaster"):
            assert set(controllers[controller]
                       ["ros__parameters"]["joints"]) == names

    composed = ET.parse(model_path)
    exclusions = {tuple(sorted((e.get("body1"), e.get("body2"))))
                  for e in composed.findall("contact/exclude")}
    assert expected_exclusions <= exclusions
    assert model.nexclude >= len(expected_exclusions)
