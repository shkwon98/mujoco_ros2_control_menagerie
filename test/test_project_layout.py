from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ROBOT_ROOT = ROOT.parent


def test_humanoid_project_contains_g1_and_rby1_subprojects():
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
    assert "<exec_depend>g1_mujoco_ros2</exec_depend>" in package_xml
    assert "<exec_depend>rby1_mujoco_ros2</exec_depend>" in package_xml
    assert "project(humanoid_mujoco_ros2_control)" in cmake


def test_g1_and_rby1_mujoco_ros2_control_entrypoints_are_documented():
    readme = (ROOT / "README.md").read_text()
    assert "ros2 launch g1_mujoco_bringup robot.launch.py hardware_type:=mujoco" in readme
    assert "ros2 launch rby1_mujoco_bringup robot.launch.py hardware_type:=mujoco" in readme

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
    assert "rby1_hardware" not in rby1_control


def test_rby1_subset_contains_only_mujoco_ros2_control_entrypoints():
    rby1 = ROOT / "rby1_mujoco_ros2"
    description = rby1 / "rby1_mujoco_description"
    bringup = rby1 / "rby1_mujoco_bringup"

    assert (description / "urdf" / "rby1.urdf.xacro").is_file()
    assert (description / "urdf" / "rby1.ros2_control.xacro").is_file()
    assert (description / "mjcf" / "rby1a" / "model_act.xml").is_file()
    assert (description / "meshes" / "rby1a").is_dir()
    assert (bringup / "launch" / "robot.launch.py").is_file()
    assert (bringup / "config" / "ros2_control" / "rby1a_controllers.yaml").is_file()
    assert (rby1 / "rby1_mujoco_ros2" / "package.xml").is_file()

    launch_text = (bringup / "launch" / "robot.launch.py").read_text()
    urdf_text = (description / "urdf" / "rby1.urdf.xacro").read_text()
    assert 'choices=["mujoco", "mock"]' in launch_text
    assert "robot_ip" not in launch_text
    assert 'hardware_type" default="mujoco"' in urdf_text
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
