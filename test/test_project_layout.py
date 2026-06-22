from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ROBOT_ROOT = ROOT.parent


def test_humanoid_project_contains_g1_and_rby1_subprojects():
    assert (ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_description" / "package.xml").is_file()
    assert (ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_bringup" / "package.xml").is_file()
    assert (ROOT / "g1_mujoco_ros2" / "g1_mujoco_description" / "package.xml").is_file()
    assert (ROOT / "g1_mujoco_ros2" / "g1_mujoco_bringup" / "package.xml").is_file()
    assert (ROOT / "rby1_mujoco_ros2" / "rby1_mujoco_description" / "package.xml").is_file()
    assert (ROOT / "rby1_mujoco_ros2" / "rby1_mujoco_bringup" / "package.xml").is_file()
    assert (ROOT / "humanoid_mujoco_ros2_control" / "package.xml").is_file()


def test_humanoid_project_does_not_vendor_full_rby1_ros2_repository():
    assert not (ROBOT_ROOT / "g1_mujoco_ros2").exists()
    assert not (ROOT / "rby1_ros2").exists()

    rby1_subset = ROOT / "rby1_mujoco_ros2"
    for unrelated_path in (
        "anycam",
        "habilis_robot_manager",
        "physical_ai_interfaces",
        "rby1_hardware",
        ".git",
        ".github",
        ".devcontainer",
        "docs",
    ):
        assert not (rby1_subset / unrelated_path).exists()


def test_humanoid_metapackage_depends_on_robot_metapackages():
    package_xml = (ROOT / "humanoid_mujoco_ros2_control" / "package.xml").read_text()
    cmake = (ROOT / "humanoid_mujoco_ros2_control" / "CMakeLists.txt").read_text()

    assert "<name>humanoid_mujoco_ros2_control</name>" in package_xml
    assert "<exec_depend>ai_worker_mujoco_ros2</exec_depend>" in package_xml
    assert "<exec_depend>g1_mujoco_ros2</exec_depend>" in package_xml
    assert "<exec_depend>rby1_mujoco_ros2</exec_depend>" in package_xml
    assert "project(humanoid_mujoco_ros2_control)" in cmake


def test_g1_and_rby1_mujoco_ros2_control_entrypoints_are_documented():
    readme = (ROOT / "README.md").read_text()
    assert "ros2 launch ai_worker_mujoco_bringup robot.launch.py" in readme
    assert "ros2 launch g1_mujoco_bringup robot.launch.py" in readme
    assert "ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=a" in readme
    assert "hardware_type" not in readme

    g1_control = (
        ROOT
        / "g1_mujoco_ros2"
        / "g1_mujoco_description"
        / "urdf"
        / "g1.ros2_control.xacro"
    ).read_text()
    rby1_control = (
        ROOT
        / "rby1_mujoco_ros2"
        / "rby1_mujoco_description"
        / "urdf"
        / "rby1.ros2_control.xacro"
    ).read_text()

    assert "mujoco_ros2_control/MujocoSystemInterface" in g1_control
    assert "mujoco_ros2_control/MujocoSystemInterface" in rby1_control
    assert "mock_components/GenericSystem" not in g1_control
    assert "mock_components/GenericSystem" not in rby1_control
    assert "rby1_hardware" not in rby1_control


def test_ai_worker_uses_official_ffw_description_variants():
    ai_worker = ROOT / "ai_worker_mujoco_ros2"
    description = ai_worker / "ai_worker_mujoco_description"
    bringup = ai_worker / "ai_worker_mujoco_bringup"
    variants = {
        "ffw_bg2_rev2_follower": "ffw_bg2_follower.urdf.xacro",
        "ffw_bg2_rev3_follower": "ffw_bg2_follower.urdf.xacro",
        "ffw_bg2_rev4_follower": "ffw_bg2_follower.urdf.xacro",
        "ffw_bh5_rev1_follower": "ffw_bh5_follower.urdf.xacro",
        "ffw_sg2_rev1_follower": "ffw_sg2_follower.urdf.xacro",
        "ffw_sh5_rev1_follower": "ffw_sh5_follower.urdf.xacro",
    }

    assert not (description / "urdf" / "ai_worker.urdf").exists()
    assert (description / "urdf" / "common" / "follower" / "ffw_follower_body.xacro").is_file()
    assert (description / "urdf" / "common" / "rh_p12_rn_a" / "rh_p12_rn_a.urdf.xacro").is_file()
    assert (description / "urdf" / "common" / "hx5_d20" / "hx5_d20_left.urdf.xacro").is_file()
    assert (description / "meshes" / "common" / "follower" / "body_arm_assy.stl").is_file()
    assert (description / "meshes" / "common" / "follower" / "base" / "base_mobile_assy.stl").is_file()
    assert (description / "meshes" / "common" / "follower" / "swerve" / "base_mobile_assy.stl").is_file()

    wrapper = (description / "urdf" / "ai_worker_mujoco.urdf.xacro").read_text()
    launch_text = (bringup / "launch" / "robot.launch.py").read_text()
    controllers = (description / "config" / "ros2_control" / "ai_worker_controllers.yaml").read_text()
    control_xacro = (description / "urdf" / "ai_worker.ros2_control.xacro").read_text()

    assert "robot_model" in wrapper
    assert "robot_model" in launch_text
    for model_name in ("ffw_bg2", "ffw_bh5", "ffw_sg2", "ffw_sh5"):
        assert model_name in wrapper
        assert model_name in launch_text
    assert "head_controller:" in controllers
    assert "head_joint1" in controllers
    assert "head_joint2" in controllers
    assert 'joint_name="head_joint1"' in control_xacro
    assert 'joint_name="head_joint2"' in control_xacro
    assert "scene_${robot_model}.xml" in control_xacro

    for variant, xacro_name in variants.items():
        variant_xacro = description / "urdf" / variant / xacro_name
        assert variant_xacro.is_file()

    copied_xacros = "\n".join(path.read_text() for path in (description / "urdf").rglob("*.xacro"))
    for forbidden in (
        "realsense2_description",
        "sensor_d405",
        "gz_ros2_control",
        "dynamixel_hardware_interface",
    ):
        assert forbidden not in copied_xacros
        assert forbidden not in control_xacro


def test_ai_worker_mjcf_follows_robotis_menagerie_model_split():
    mjcf_dir = ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_description" / "mjcf"
    expected = {
        "ffw_bg2.xml": {
            "model": "ffw_bg2",
            "min_meshes": 23,
            "actuators": {"lift_joint", "gripper_l_joint1", "gripper_r_joint1"},
            "meshes": {"base_mobile_assy", "gripper_base"},
        },
        "ffw_bh5.xml": {
            "model": "ffw_bh5",
            "min_meshes": 36,
            "actuators": {"lift_joint", "finger_l_joint1", "finger_r_joint20"},
            "meshes": {"base_mobile_assy", "hx5_l_base", "hx5_r_base"},
        },
        "ffw_sg2.xml": {
            "model": "ffw_sg2",
            "min_meshes": 27,
            "actuators": {"left_wheel_steer", "left_wheel_drive", "gripper_l_joint1"},
            "meshes": {"base_mobile_assy", "left_wheel", "lift_link", "gripper_base"},
        },
        "ffw_sh5.xml": {
            "model": "ffw_sh5",
            "min_meshes": 40,
            "actuators": {"left_wheel_steer", "finger_l_joint1", "finger_r_joint20"},
            "meshes": {"base_mobile_assy", "left_wheel", "lift_link", "hx5_l_base", "hx5_r_base"},
        },
    }

    for filename, spec in expected.items():
        path = mjcf_dir / filename
        assert path.is_file()
        root = ET.parse(path).getroot()
        assert root.attrib["model"] == spec["model"]
        assert root.find("compiler").attrib["meshdir"] == "../meshes"
        assert root.find("option").attrib["integrator"] == "implicitfast"
        assert len(root.findall(".//asset/mesh")) >= spec["min_meshes"]

        mesh_names = {mesh.attrib["name"] for mesh in root.findall(".//asset/mesh")}
        actuator_names = {actuator.attrib["name"] for actuator in root.findall(".//actuator/*")}
        for mesh_name in spec["meshes"]:
            assert mesh_name in mesh_names
        for actuator_name in spec["actuators"]:
            assert actuator_name in actuator_names

    for filename in ("scene_ffw_bg2.xml", "scene_ffw_bh5.xml", "scene_ffw_sg2.xml", "scene_ffw_sh5.xml"):
        path = mjcf_dir / filename
        assert path.is_file()
        assert ET.parse(path).getroot().find("include").attrib["file"] == filename.removeprefix("scene_")


def test_ai_worker_exposes_hand_and_wheel_control_interfaces():
    description = ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_description"
    bringup = ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_bringup"
    control_xacro = (description / "urdf" / "ai_worker.ros2_control.xacro").read_text()
    launch_text = (bringup / "launch" / "robot.launch.py").read_text()

    for joint_name in (
        "gripper_l_joint1",
        "gripper_r_joint1",
        "finger_l_joint1",
        "finger_l_joint20",
        "finger_r_joint1",
        "finger_r_joint20",
        "left_wheel_steer_joint",
        "left_wheel_drive_joint",
        "right_wheel_steer_joint",
        "right_wheel_drive_joint",
        "rear_wheel_steer_joint",
        "rear_wheel_drive_joint",
    ):
        assert joint_name in control_xacro

    assert 'command_interface name="velocity"' in control_xacro
    assert "controller_file_by_model" in launch_text
    assert "hand_left_controller" in launch_text
    assert "hand_right_controller" in launch_text
    assert "base_steer_controller" in launch_text
    assert "base_drive_controller" in launch_text

    configs = {
        "ffw_bg2": {
            "file": "ai_worker_ffw_bg2_controllers.yaml",
            "include": {"hand_left_controller", "hand_right_controller", "gripper_l_joint1", "gripper_r_joint1"},
            "exclude": {"base_drive_controller", "finger_l_joint1"},
        },
        "ffw_bh5": {
            "file": "ai_worker_ffw_bh5_controllers.yaml",
            "include": {"hand_left_controller", "hand_right_controller", "finger_l_joint1", "finger_l_joint20"},
            "exclude": {"base_drive_controller", "gripper_l_joint1"},
        },
        "ffw_sg2": {
            "file": "ai_worker_ffw_sg2_controllers.yaml",
            "include": {"base_steer_controller", "base_drive_controller", "left_wheel_drive_joint", "gripper_l_joint1"},
            "exclude": {"finger_l_joint1"},
        },
        "ffw_sh5": {
            "file": "ai_worker_ffw_sh5_controllers.yaml",
            "include": {"base_steer_controller", "base_drive_controller", "left_wheel_drive_joint", "finger_l_joint20"},
            "exclude": {"gripper_l_joint1"},
        },
    }

    for spec in configs.values():
        path = description / "config" / "ros2_control" / spec["file"]
        assert path.is_file()
        text = path.read_text()
        for token in spec["include"]:
            assert token in text
        for token in spec["exclude"]:
            assert token not in text


def test_ai_worker_mujoco_models_are_stable_without_commands():
    mujoco = __import__("mujoco")
    np = __import__("numpy")

    mjcf_dir = ROOT / "ai_worker_mujoco_ros2" / "ai_worker_mujoco_description" / "mjcf"
    initial_positions = {
        "lift_joint": 0.0,
        "head_joint1": 0.0,
        "head_joint2": 0.0,
        "arm_l_joint1": 0.0,
        "arm_l_joint2": 0.0,
        "arm_l_joint3": 0.0,
        "arm_l_joint4": -0.7,
        "arm_l_joint5": 0.0,
        "arm_l_joint6": 0.0,
        "arm_l_joint7": 0.0,
        "arm_r_joint1": 0.0,
        "arm_r_joint2": 0.0,
        "arm_r_joint3": 0.0,
        "arm_r_joint4": -0.7,
        "arm_r_joint5": 0.0,
        "arm_r_joint6": 0.0,
        "arm_r_joint7": 0.0,
    }

    for filename in ("scene_ffw_bg2.xml", "scene_ffw_bh5.xml", "scene_ffw_sg2.xml", "scene_ffw_sh5.xml"):
        model = mujoco.MjModel.from_xml_path(str(mjcf_dir / filename))
        data = mujoco.MjData(model)
        for actuator_id in range(model.nu):
            joint_id = model.actuator_trnid[actuator_id][0]
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            data.ctrl[actuator_id] = initial_positions.get(joint_name, 0.0)

        previous_time = data.time
        for _ in range(int(1.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
            assert data.time > previous_time
            previous_time = data.time
            assert np.isfinite(data.qpos).all()
            assert np.isfinite(data.qvel).all()
            assert np.isfinite(data.qacc).all()


def test_humanoid_mujoco_ros2_control_is_mujoco_only():
    forbidden = ("hardware_type", "mock_components", "GenericSystem", "dexgraft")
    checked_suffixes = (".py", ".xacro", ".xml", ".md", ".yaml", ".yml")
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix not in checked_suffixes:
            continue
        if ".git" in path.parts or "build" in path.parts or "install" in path.parts or "test" in path.parts:
            continue
        text = path.read_text(errors="ignore")
        for token in forbidden:
            assert token not in text, f"{path.relative_to(ROOT)} still contains {token}"


def test_rby1_subset_contains_only_mujoco_ros2_control_entrypoints():
    rby1 = ROOT / "rby1_mujoco_ros2"
    description = rby1 / "rby1_mujoco_description"
    bringup = rby1 / "rby1_mujoco_bringup"

    assert (description / "urdf" / "rby1.urdf.xacro").is_file()
    assert (description / "urdf" / "rby1.ros2_control.xacro").is_file()
    assert (description / "mjcf" / "rby1a" / "model_act.xml").is_file()
    assert (description / "meshes" / "rby1a").is_dir()
    assert (description / "config" / "ros2_control" / "rby1a_controllers.yaml").is_file()
    assert (description / "config" / "ros2_control" / "rby1m_controllers.yaml").is_file()
    assert (description / "config" / "ros2_control" / "rby1ub_controllers.yaml").is_file()
    assert (bringup / "launch" / "robot.launch.py").is_file()
    assert not (bringup / "config" / "ros2_control").exists()
    assert (rby1 / "rby1_mujoco_ros2" / "package.xml").is_file()

    launch_text = (bringup / "launch" / "robot.launch.py").read_text()
    urdf_text = (description / "urdf" / "rby1.urdf.xacro").read_text()
    assert 'FindPackageShare("rby1_mujoco_description")' in launch_text
    assert 'FindPackageShare("rby1_mujoco_bringup")' not in launch_text
    assert "hardware_type" not in launch_text
    assert "robot_ip" not in launch_text
    assert "hardware_type" not in urdf_text
    assert "rby1_hardware" not in urdf_text
    assert "camera.launch.py" not in {path.name for path in (bringup / "launch").iterdir()}
    assert not (bringup / "scripts").exists()


def _mjcf_joint_names(path):
    root = ET.parse(path).getroot()
    return {
        joint.attrib["name"]
        for joint in root.iter("joint")
        if "name" in joint.attrib
    }


def _mjcf_actuator_targets(path):
    root = ET.parse(path).getroot()
    actuator = root.find("actuator")
    assert actuator is not None, f"{path} has no actuator block"
    return {
        child.attrib["joint"]
        for child in actuator
        if "joint" in child.attrib
    }


def _mjcf_version_include(path):
    root = ET.parse(path).getroot()
    includes = [
        include.attrib["file"]
        for include in root.findall("include")
        if include.attrib.get("file", "").startswith("./rby1_")
    ]
    assert len(includes) == 1, f"{path} should include one versioned RBY1 body"
    return includes[0]


def test_rby1_mujoco_subset_supports_all_versioned_models():
    description = ROOT / "rby1_mujoco_ros2" / "rby1_mujoco_description"
    control_xacro = (description / "urdf" / "rby1.ros2_control.xacro").read_text()
    urdf_xacro = (description / "urdf" / "rby1.urdf.xacro").read_text()

    assert "robot_version" in control_xacro
    assert "model_act_${robot_version}.xml" in control_xacro
    assert 'robot_version="$(arg robot_version)"' in urdf_xacro

    for robot_model, versions in {
        "a": ("v1.0", "v1.1", "v1.2"),
        "m": ("v1.0", "v1.1", "v1.2", "v1.3"),
    }.items():
        mjcf_dir = description / "mjcf" / f"rby1{robot_model}"
        for version in versions:
            wrapper = mjcf_dir / f"model_act_{version}.xml"
            body = mjcf_dir / f"rby1_{version}.xml"
            assert wrapper.is_file()
            assert _mjcf_version_include(wrapper) == f"./rby1_{version}.xml"
            missing_targets = _mjcf_actuator_targets(wrapper) - _mjcf_joint_names(body)
            assert not missing_targets, (
                f"{wrapper.relative_to(ROOT)} targets missing joints: "
                f"{sorted(missing_targets)}"
            )

    ub_wrapper = description / "mjcf" / "rby1ub" / "model_act.xml"
    ub_body = description / "mjcf" / "rby1ub" / "rby1.xml"
    assert not _mjcf_actuator_targets(ub_wrapper) - _mjcf_joint_names(ub_body)
