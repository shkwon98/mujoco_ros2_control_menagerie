import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


URDF_DIR = Path(__file__).parents[1] / "urdf"
MODELS = {
    "ffw_bg2": ("gripper_l_joint1", "gripper_r_joint1"),
    "ffw_bh5": ("finger_l_joint1", "finger_r_joint1"),
    "ffw_sg2": ("gripper_l_joint1", "gripper_r_joint1"),
    "ffw_sh5": ("finger_l_joint1", "finger_r_joint1"),
}


def expand_xacro(file_name, **mappings):
    command = ["xacro", str(URDF_DIR / file_name)]
    command.extend(f"{name}:={value}" for name, value in mappings.items())
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return ET.fromstring(result.stdout)


def joint_names(robot):
    return {joint.attrib["name"] for joint in robot.findall("joint")}


def test_body_and_hand_descriptions_partition_each_model():
    for model, (left_joint, right_joint) in MODELS.items():
        full = joint_names(
            expand_xacro("ai_worker_mujoco.urdf.xacro", robot_model=model)
        )
        body = joint_names(
            expand_xacro(
                "ai_worker_mujoco.urdf.xacro",
                robot_model=model,
                include_hands="false",
                include_ros2_control="false",
            )
        )
        left = joint_names(
            expand_xacro("ai_worker_hand.urdf.xacro", robot_model=model, side="left")
        )
        right = joint_names(
            expand_xacro("ai_worker_hand.urdf.xacro", robot_model=model, side="right")
        )

        assert {left_joint, right_joint} <= full
        assert left_joint not in body and right_joint not in body
        assert left_joint in left and right_joint not in left
        assert right_joint in right and left_joint not in right
