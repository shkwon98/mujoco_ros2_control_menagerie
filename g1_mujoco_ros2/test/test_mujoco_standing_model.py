from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "g1_mujoco_description"


def test_fixed_base_mujoco_model_keeps_pelvis_supported_for_upper_body_control():
    mujoco = pytest.importorskip("mujoco")

    model_path = DESCRIPTION / "mjcf" / "g1_29dof_fixed.xml"
    text = model_path.read_text()

    assert 'joint name="floating_base_joint"' not in text

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    pelvis_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
    )

    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[pelvis_id, 2])
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    assert model.nq == model.nu
    assert float(data.xpos[pelvis_id, 2]) == pytest.approx(initial_z, abs=1e-6)


def test_fixed_base_mujoco_model_does_not_oscillate_without_commands():
    mujoco = pytest.importorskip("mujoco")
    np = pytest.importorskip("numpy")

    model_path = DESCRIPTION / "mjcf" / "g1_29dof_fixed.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    mujoco.mj_forward(model, data)
    initial_qpos = data.qpos.copy()
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    joint_motion = np.max(np.abs(data.qpos - initial_qpos))
    joint_speed = np.max(np.abs(data.qvel))

    assert joint_motion < 0.1
    assert joint_speed < 0.5


def test_default_xacro_uses_menagerie_scene_and_keeps_fixed_base_override():
    wrapper = DESCRIPTION / "urdf" / "g1_mujoco.urdf.xacro"
    ros2_control = DESCRIPTION / "urdf" / "g1.ros2_control.xacro"
    wrapper_text = wrapper.read_text()

    assert 'name="robot_model" default="g1"' in wrapper_text
    assert 'name="mujoco_model_file" default="auto"' in wrapper_text
    assert "scene_with_hands_fixed.xml" in ros2_control.read_text()
    assert "mujoco_model_file" in ros2_control.read_text()
    assert (DESCRIPTION / "mjcf" / "g1_29dof_fixed.xml").is_file()
    assert (DESCRIPTION / "mjcf" / "scene_with_hands_fixed.xml").is_file()


def test_fixed_hand_mujoco_scene_keeps_pelvis_supported_for_upper_body_control():
    mujoco = pytest.importorskip("mujoco")

    model_path = DESCRIPTION / "mjcf" / "scene_with_hands_fixed.xml"
    text = model_path.read_text()

    assert 'include file="g1_with_hands.xml"' in text
    assert 'weld body1="world" body2="pelvis"' in text

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    pelvis_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
    )

    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[pelvis_id, 2])
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    assert float(data.xpos[pelvis_id, 2]) == pytest.approx(initial_z, abs=1e-4)


@pytest.mark.parametrize("model_name", ["g1_29dof_fixed.xml", "g1_29dof.xml"])
def test_g1_mujoco_models_include_passive_joint_stabilization(model_name):
    text = (DESCRIPTION / "mjcf" / model_name).read_text()

    assert 'joint damping="0.05" armature="0.01" frictionloss="0.2"' in text
