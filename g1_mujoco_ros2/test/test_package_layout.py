from pathlib import Path

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
    assert (DESCRIPTION / "urdf" / "g1_mujoco.urdf.xacro").is_file()
    assert (DESCRIPTION / "urdf" / "g1.ros2_control.xacro").is_file()
    assert (DESCRIPTION / "mjcf" / "g1_29dof.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "g1_29dof_fixed.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "meshes" / "pelvis.STL").is_file()
    assert (DESCRIPTION / "meshes" / "pelvis.STL").is_file()

    xacro_text = (DESCRIPTION / "urdf" / "g1.ros2_control.xacro").read_text()
    assert "mujoco_ros2_control/MujocoSystemInterface" in xacro_text
    assert "mock_components/GenericSystem" in xacro_text
    assert "$(find-pkg-share g1_mujoco_description)/mjcf/g1_29dof_fixed.xml" in xacro_text

    urdf_text = (DESCRIPTION / "urdf" / "g1_29dof.urdf").read_text()
    assert "package://g1_mujoco_description/meshes/pelvis.STL" in urdf_text
    assert 'joint name="left_shoulder_pitch_joint"' in urdf_text
    assert 'joint name="right_wrist_yaw_joint"' in urdf_text


def test_controller_config_exposes_joint_trajectory_surfaces():
    config = DESCRIPTION / "config" / "ros2_control" / "g1_controllers.yaml"
    text = config.read_text()

    assert "joint_state_broadcaster:" in text
    assert "right_arm_controller:" in text
    assert "left_arm_controller:" in text
    assert "waist_controller:" in text
    assert "leg_controller:" in text
    assert "joint_trajectory_controller/JointTrajectoryController" in text
    assert "right_shoulder_pitch_joint" in text
    assert "left_shoulder_pitch_joint" in text
    assert "waist_yaw_joint" in text


def test_bringup_launch_starts_ros2_control_and_spawners():
    launch = BRINGUP / "launch" / "robot.launch.py"
    text = launch.read_text()

    assert 'DeclareLaunchArgument("hardware_type"' in text
    assert 'choices=["mujoco", "mock"]' in text
    assert 'choices=["g1_29dof_fixed.xml", "g1_29dof.xml"]' in text
    assert 'executable="ros2_control_node"' in text
    assert 'executable="robot_state_publisher"' in text
    assert 'joint_state_broadcaster' in text
    assert 'right_arm_controller' in text
    assert 'left_arm_controller' in text
    assert 'waist_controller' in text
    assert 'leg_controller' in text
    assert "/sensors/proprio/g1/joint_states" in text
    assert "/control/g1/right_arm_controller/joint_trajectory" in text
    assert "/control/g1/left_arm_controller/joint_trajectory" in text


def test_dexgraft_body_config_points_to_g1_ros2_control_topics():
    config = BRINGUP / "config" / "dexgraft_g1.yaml"
    text = config.read_text()

    assert 'urdf_path: "$(find-pkg-share g1_mujoco_description)/urdf/g1_29dof.urdf"' in text
    assert 'joint_states_topic: "/sensors/proprio/g1/joint_states"' in text
    assert 'right_traj_topic: "/control/g1/right_arm_controller/joint_trajectory"' in text
    assert 'left_traj_topic: "/control/g1/left_arm_controller/joint_trajectory"' in text
    assert "right_shoulder_pitch_joint" in text
    assert "left_shoulder_pitch_joint" in text
    assert 'right_control_frame: "right_wrist_yaw_link"' in text
    assert 'left_control_frame: "left_wrist_yaw_link"' in text
