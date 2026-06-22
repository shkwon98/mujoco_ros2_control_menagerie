# Humanoid MuJoCo ROS 2 Control

This umbrella project groups humanoid robot integrations that expose MuJoCo
simulation through `ros2_control`.

Robots:

- `ai_worker_mujoco_ros2`: AI Worker FFW MuJoCo descriptions, controllers, and bringup.
- `g1_mujoco_ros2`: Unitree G1 MuJoCo description, controllers, and bringup.
- `rby1_mujoco_ros2`: RBY1 MuJoCo description, controllers, and bringup.

Build the core MuJoCo ros2_control packages:

```bash
colcon build --merge-install --symlink-install \
  --base-paths src/robot/humanoid_mujoco_ros2_control \
  --packages-select \
    ai_worker_mujoco_description ai_worker_mujoco_bringup ai_worker_mujoco_ros2 \
    g1_mujoco_description g1_mujoco_bringup g1_mujoco_ros2 \
    rby1_mujoco_description rby1_mujoco_bringup rby1_mujoco_ros2 \
    humanoid_mujoco_ros2_control
```

Launch AI Worker:

```bash
source install/setup.bash
ros2 launch ai_worker_mujoco_bringup robot.launch.py
```

Supported AI Worker MuJoCo models are `ffw_bg2`, `ffw_bh5`, `ffw_sg2`, and
`ffw_sh5`. The default is `ffw_bg2`.

```bash
ros2 launch ai_worker_mujoco_bringup robot.launch.py robot_model:=ffw_sg2
```

Launch G1:

```bash
source install/setup.bash
ros2 launch g1_mujoco_bringup robot.launch.py
```

Launch RBY1:

```bash
source install/setup.bash
ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=a
```

Common body control topics:

```text
/control/body/robot_description
/control/body/arm_left_controller/joint_trajectory
/control/body/arm_right_controller/joint_trajectory
/control/body/torso_controller/joint_trajectory
/control/body/head_controller/joint_trajectory
/control/body/arm_left_controller/follow_joint_trajectory
/control/body/arm_right_controller/follow_joint_trajectory
/control/body/torso_controller/follow_joint_trajectory
/control/body/head_controller/follow_joint_trajectory
```

`head_controller` is available only on robot models that expose controllable head joints.

G1 defaults to a fixed-base MuJoCo model for upper-body control bringup.
Use `mujoco_model_file:=g1_29dof.xml` only when a whole-body balance controller
is also running.
