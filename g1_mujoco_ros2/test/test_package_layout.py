from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "g1_mujoco_description"
BRINGUP = ROOT / "g1_mujoco_bringup"
META = ROOT / "g1_mujoco_ros2"


def test_workspace_contains_expected_ros_packages():
    for package_dir, package_name in (
        (DESCRIPTION, "g1_mujoco_description"),
        (BRINGUP, "g1_mujoco_bringup"),
        (META, "g1_mujoco_ros2"),
    ):
        assert (package_dir / "package.xml").is_file()
        assert (package_dir / "CMakeLists.txt").is_file()
        assert f"<name>{package_name}</name>" in (package_dir / "package.xml").read_text()


def test_description_provides_urdf_mjcf_meshes_and_ros2_control_xacro():
    assert (DESCRIPTION / "urdf" / "g1_29dof.urdf").is_file()
    assert (DESCRIPTION / "urdf" / "g1_hands.urdf.xacro").is_file()
    assert (DESCRIPTION / "urdf" / "g1_mujoco.urdf.xacro").is_file()
    assert (DESCRIPTION / "urdf" / "g1.ros2_control.xacro").is_file()
    assert (DESCRIPTION / "mjcf" / "g1.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "g1_with_hands.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "scene.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "scene_with_hands.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "scene_with_hands_fixed.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "g1_29dof.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "g1_29dof_fixed.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "assets" / "pelvis.STL").is_file()
    assert (DESCRIPTION / "mjcf" / "assets" / "left_hand_palm_link.STL").is_file()
    assert (DESCRIPTION / "meshes" / "pelvis.STL").is_file()

    xacro_text = (DESCRIPTION / "urdf" / "g1.ros2_control.xacro").read_text()
    assert "mujoco_ros2_control/MujocoSystemInterface" in xacro_text
    assert "mock_components/GenericSystem" not in xacro_text
    assert "hardware_type" not in xacro_text
    assert "selected_mujoco_model_file" in xacro_text
    assert "$(find g1_mujoco_description)/mjcf/${selected_mujoco_model_file}" in xacro_text
    assert "robot_model == 'g1_with_hands'" in xacro_text
    assert "${side}_hand_thumb_0_joint" in xacro_text
    assert "${side}_hand_index_1_joint" in xacro_text

    urdf_text = (DESCRIPTION / "urdf" / "g1_29dof.urdf").read_text()
    assert "package://g1_mujoco_description/meshes/pelvis.STL" in urdf_text
    assert 'joint name="left_shoulder_pitch_joint"' in urdf_text
    assert 'joint name="right_wrist_yaw_joint"' in urdf_text

    wrapper_text = (DESCRIPTION / "urdf" / "g1_mujoco.urdf.xacro").read_text()
    hands_urdf_text = (DESCRIPTION / "urdf" / "g1_hands.urdf.xacro").read_text()
    assert "g1_hands.urdf.xacro" in wrapper_text
    assert "g1_optional_hands" in wrapper_text
    assert "robot_model == 'g1_with_hands'" in hands_urdf_text
    assert 'name="left_hand_thumb_0_joint"' in hands_urdf_text
    assert 'name="right_hand_index_1_joint"' in hands_urdf_text


def test_g1_mjcf_follows_google_deepmind_menagerie_model_split():
    expected = {
        "g1.xml": {
            "model": "g1_29dof_rev_1_0",
            "joints": 29,
            "actuators": {"left_hip_pitch_joint", "right_wrist_yaw_joint"},
        },
        "g1_with_hands.xml": {
            "model": "g1_29dof_with_hand_rev_1_0",
            "joints": 43,
            "actuators": {
                "left_hand_thumb_0_joint",
                "left_hand_middle_1_joint",
                "right_hand_index_1_joint",
            },
        },
    }

    for filename, spec in expected.items():
        root = ET.parse(DESCRIPTION / "mjcf" / filename).getroot()
        assert root.attrib["model"] == spec["model"]
        assert root.find("compiler").attrib["meshdir"] == "assets"
        named_joints = [joint for joint in root.findall(".//joint") if joint.attrib.get("name")]
        assert len(named_joints) == spec["joints"]

        actuator_names = {actuator.attrib["name"] for actuator in root.findall(".//actuator/*")}
        for actuator_name in spec["actuators"]:
            assert actuator_name in actuator_names

    scenes = {
        "scene.xml": "g1.xml",
        "scene_with_hands.xml": "g1_with_hands.xml",
        "scene_with_hands_fixed.xml": "g1_with_hands.xml",
    }
    for scene_name, include_file in scenes.items():
        root = ET.parse(DESCRIPTION / "mjcf" / scene_name).getroot()
        assert root.find("include").attrib["file"] == include_file


def test_controller_config_exposes_joint_trajectory_surfaces():
    config = DESCRIPTION / "config" / "ros2_control" / "g1_controllers.yaml"
    text = config.read_text()

    assert "body_joint_state_broadcaster:" in text
    assert "arm_right_controller:" in text
    assert "arm_left_controller:" in text
    assert "torso_controller:" in text
    assert "leg_controller:" in text
    assert "joint_trajectory_controller/JointTrajectoryController" in text
    assert "right_shoulder_pitch_joint" in text
    assert "left_shoulder_pitch_joint" in text
    assert "waist_yaw_joint" in text

    hand_config = DESCRIPTION / "config" / "ros2_control" / "g1_with_hands_controllers.yaml"
    hand_text = hand_config.read_text()
    assert "body_joint_state_broadcaster:" in hand_text
    assert "hand_left_joint_state_broadcaster:" in hand_text
    assert "hand_right_joint_state_broadcaster:" in hand_text
    assert "hand_left_controller:" in hand_text
    assert "hand_right_controller:" in hand_text
    assert "left_hand_thumb_0_joint" in hand_text
    assert "left_hand_index_1_joint" in hand_text
    assert "right_hand_thumb_0_joint" in hand_text
    assert "right_hand_middle_1_joint" in hand_text


def test_bringup_launch_starts_ros2_control_and_spawners():
    launch = BRINGUP / "launch" / "robot.launch.py"
    text = launch.read_text()

    assert 'DeclareLaunchArgument("hardware_type"' not in text
    assert 'choices=["mujoco", "mock"]' not in text
    assert 'choices=["g1", "g1_with_hands"]' in text
    assert '"controllers_yaml"' in text
    assert '"auto"' in text
    assert "controller_file_by_model" in text
    assert "mujoco_model_file_by_model" in text
    assert "scene.xml" in text
    assert "scene_with_hands_fixed.xml" in text
    assert 'executable="ros2_control_node"' in text
    assert 'executable="robot_state_publisher"' in text
    assert 'body_joint_state_broadcaster' in text
    assert 'hand_left_joint_state_broadcaster' in text
    assert 'hand_right_joint_state_broadcaster' in text
    assert 'arm_right_controller' in text
    assert 'arm_left_controller' in text
    assert 'torso_controller' in text
    assert 'leg_controller' in text
    assert 'hand_left_controller' in text
    assert 'hand_right_controller' in text
    assert "__ns:={controller_namespace}" in text
    assert '"/control/hand_left",' in text
    assert '"/control/hand_right",' in text
    assert "/sensors/proprio/body/joint_states" in text
    assert "/sensors/proprio/body/dynamic_joint_states" in text
    assert "/sensors/proprio/hand_left/joint_states" in text
    assert "/sensors/proprio/hand_left/dynamic_joint_states" in text
    assert "/sensors/proprio/hand_right/joint_states" in text
    assert "/sensors/proprio/hand_right/dynamic_joint_states" in text
    assert "/control/body/robot_description" in text
    assert "/control/body/arm_right_controller/joint_trajectory" in text
    assert "/control/body/arm_left_controller/joint_trajectory" in text
    assert "/control/body/torso_controller/joint_trajectory" in text
    assert "/control/hand_left/hand_left_controller/joint_trajectory" in text
    assert "/control/hand_right/hand_right_controller/joint_trajectory" in text


def test_g1_mujoco_bringup_does_not_embed_dexgraft_config():
    assert not (BRINGUP / "config").exists()

    checked_suffixes = (".py", ".xml", ".yaml", ".yml", ".xacro", ".md")
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix not in checked_suffixes or "test" in path.parts:
            continue
        assert "dexgraft" not in path.read_text(errors="ignore").lower()
