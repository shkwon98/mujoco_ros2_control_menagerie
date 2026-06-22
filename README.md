# MuJoCo ROS 2 Control Menagerie

This umbrella project groups robot integrations that expose MuJoCo
simulation through `ros2_control`.

Robots:

- `ai_worker_mujoco_ros2`: AI Worker FFW MuJoCo descriptions, controllers, and bringup.
- `g1_mujoco_ros2`: Unitree G1 MuJoCo description, controllers, and bringup.
- `rby1_mujoco_ros2`: RBY1 MuJoCo description, controllers, and bringup.

Build the core MuJoCo ros2_control packages:

```bash
colcon build --merge-install --symlink-install \
  --base-paths src/robot/mujoco_ros2_control_menagerie \
  --packages-select \
    ai_worker_mujoco_description ai_worker_mujoco_bringup ai_worker_mujoco_ros2 \
    g1_mujoco_description g1_mujoco_bringup g1_mujoco_ros2 \
    rby1_mujoco_description rby1_mujoco_bringup rby1_mujoco_ros2 \
    mujoco_ros2_control_menagerie
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

G1 supports `robot_model:=g1` and `robot_model:=g1_with_hands`.

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
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
/control/hand_left/hand_left_controller/joint_trajectory
/control/hand_right/hand_right_controller/joint_trajectory
/control/body/arm_left_controller/follow_joint_trajectory
/control/body/arm_right_controller/follow_joint_trajectory
/control/body/torso_controller/follow_joint_trajectory
/control/body/head_controller/follow_joint_trajectory
/control/hand_left/hand_left_controller/follow_joint_trajectory
/control/hand_right/hand_right_controller/follow_joint_trajectory
```

`head_controller` is available only on robot models that expose controllable head joints.
AI Worker `ffw_sg2` and `ffw_sh5` also expose `/control/body/base_steer_controller/joint_trajectory`
and `/control/body/base_drive_controller/commands`. The drive controller uses
`std_msgs/msg/Float64MultiArray` velocity commands in left, right, rear wheel order.

G1 defaults to the Google DeepMind MuJoCo Menagerie scene for the selected
`robot_model`. For fixed-base upper-body control bringup, override the model
with `mujoco_model_file:=g1_29dof_fixed.xml`.
