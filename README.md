# Humanoid MuJoCo ROS 2 Control

This umbrella project groups humanoid robot integrations that expose MuJoCo
simulation through `ros2_control`.

Robots:

- `g1_mujoco_ros2`: Unitree G1 MuJoCo description, controllers, and bringup.
- `rby1_mujoco_ros2`: RBY1 MuJoCo description, controllers, and bringup.

Build the core MuJoCo ros2_control packages:

```bash
colcon build --merge-install --symlink-install \
  --base-paths src/robot/humanoid_mujoco_ros2_control \
  --packages-select \
    g1_mujoco_description g1_mujoco_bringup g1_mujoco_ros2 \
    rby1_mujoco_description rby1_mujoco_bringup rby1_mujoco_ros2 \
    humanoid_mujoco_ros2_control
```

Launch G1:

```bash
source install/setup.bash
ros2 launch g1_mujoco_bringup robot.launch.py hardware_type:=mujoco
```

Launch RBY1:

```bash
source install/setup.bash
ros2 launch rby1_mujoco_bringup robot.launch.py hardware_type:=mujoco robot_model:=a
```

G1 defaults to a fixed-base MuJoCo model for upper-body and DexGRAFT bringup.
Use `mujoco_model_file:=g1_29dof.xml` only when a whole-body balance controller
is also running.
