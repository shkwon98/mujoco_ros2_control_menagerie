# Unitree G1 MuJoCo ROS 2 Control

This folder bootstraps Unitree G1 support in the same shape as the current
RBY1 setup:

- `g1_mujoco_description`: URDF, MJCF, meshes, and `ros2_control`
  xacro.
- `g1_mujoco_bringup`: `ros2_control_node`, controller spawners, and
  a DexGRAFT body config.
- `g1_mujoco_ros2`: metapackage.

The initial model uses Unitree's `g1_29dof.urdf` and matching `g1_29dof.xml`
from `unitreerobotics/unitree_ros`. The MJCF actuator block has been converted
from torque motors to position actuators so that
`joint_trajectory_controller/JointTrajectoryController` can command position
trajectories through `mujoco_ros2_control`.

The default launch uses `g1_29dof_fixed.xml`, where the pelvis is fixed to the
world. This is intentional for DexGRAFT and upper-body control bringup: a
floating humanoid will not stand from joint trajectory position controllers
alone. Use `mujoco_model_file:=g1_29dof.xml` only when you also provide a
whole-body balance controller.

Build:

```bash
colcon build --merge-install --symlink-install --base-paths src/robot/humanoid_mujoco_ros2_control/g1_mujoco_ros2
```

Launch MuJoCo-backed control:

```bash
source install/setup.bash
ros2 launch g1_mujoco_bringup robot.launch.py hardware_type:=mujoco
```

Launch the mock backend for ROS-side testing:

```bash
source install/setup.bash
ros2 launch g1_mujoco_bringup robot.launch.py hardware_type:=mock
```

The DexGRAFT body config is:

```bash
src/robot/humanoid_mujoco_ros2_control/g1_mujoco_ros2/g1_mujoco_bringup/config/dexgraft_g1.yaml
```

Before using this for real task tuning, verify whether your physical or target
sim robot is the older 29-DoF layout, lock-waist layout, or a newer revision.
The URDF, MJCF, controller joint list, and DexGRAFT frame names must all stay
on the same G1 variant.
