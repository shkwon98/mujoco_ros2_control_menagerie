# Unitree G1 MuJoCo ROS 2 Control

This folder provides Unitree G1 support in the same shape as the other
MuJoCo ROS 2 Control Menagerie integrations:

- `g1_mujoco_description`: URDF, MJCF, meshes, and `ros2_control`
  xacro.
- `g1_mujoco_bringup`: `ros2_control_node` and controller spawners.
- `g1_mujoco_ros2`: metapackage.

The primary MuJoCo models follow Google DeepMind MuJoCo Menagerie's
`unitree_g1` split:

- `robot_model:=g1`: 29-DoF body model, `mjcf/scene.xml`
- `robot_model:=g1_with_hands`: 29-DoF body plus 14 hand joints, `mjcf/scene_with_hands_fixed.xml`

The older local `g1_29dof.xml` and `g1_29dof_fixed.xml` files are kept for
compatibility and static upper-body bringup. The default hand scene welds the
pelvis to the world for upper-body and hand control without a balance controller.

Build:

```bash
colcon build --merge-install --symlink-install --base-paths src/robot/mujoco_ros2_control_menagerie/g1_mujoco_ros2
```

Launch MuJoCo-backed control:

```bash
source install/setup.bash
ros2 launch g1_mujoco_bringup robot.launch.py
```

Launch the hand model:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
```

Use the original floating-base hand scene explicitly when testing balance:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands mujoco_model_file:=scene_with_hands.xml
```

For fixed-base upper-body testing with the body-only model:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py mujoco_model_file:=g1_29dof_fixed.xml
```

Main trajectory topics:

```text
/control/body/arm_left_controller/joint_trajectory
/control/body/arm_right_controller/joint_trajectory
/control/body/torso_controller/joint_trajectory
/control/body/leg_controller/joint_trajectory
/control/hand_left/hand_left_controller/joint_trajectory
/control/hand_right/hand_right_controller/joint_trajectory
```

Main proprioception topics for `robot_model:=g1_with_hands`:

```text
/sensors/proprio/body/joint_states
/sensors/proprio/body/dynamic_joint_states
/sensors/proprio/hand_left/joint_states
/sensors/proprio/hand_left/dynamic_joint_states
/sensors/proprio/hand_right/joint_states
/sensors/proprio/hand_right/dynamic_joint_states
```

Before using this for real task tuning, verify whether your physical or target
sim robot is the older 29-DoF layout, lock-waist layout, or a newer revision.
The URDF, MJCF, and controller joint list must all stay on the same G1 variant.
