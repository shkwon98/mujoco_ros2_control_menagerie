# MuJoCo ROS 2 Control Menagerie

ROS 2 packages for running robot MuJoCo models through `ros2_control`.

This repository is intentionally scoped to simulation bringup:

- MuJoCo model files, meshes, and URDF/xacro wrappers
- `ros2_control` hardware descriptions
- Controller YAML files
- Launch files that start `ros2_control_node`, `robot_state_publisher`, and controller spawners

It does not contain physical robot drivers, DexGraft retargeting configs, or task policies.
Those packages should depend on this repository, not the other way around.

## Supported Robots

| Robot | Launch package | `robot_model` values | Notes |
| --- | --- | --- | --- |
| AI Worker FFW | `ai_worker_mujoco_bringup` | `ffw_bg2`, `ffw_bh5`, `ffw_sg2`, `ffw_sh5` | `g2` models use grippers, `h5` models use 20-joint hands, `s` models include mobile-base wheel control |
| Unitree G1 | `g1_mujoco_bringup` | `g1`, `g1_with_hands`, `g1_with_inspire_hands` | Hand variants default to fixed-base scenes for upper-body work |
| RBY1 | `rby1_mujoco_bringup` | `a`, `m`, `a_wuji`, `m_wuji` | Wuji variants are v1.2 body models with separate hand controllers |

Each robot follows the same package split:

```text
<robot>_mujoco_ros2/
  <robot>_mujoco_description/  # URDF, MJCF, meshes, ros2_control config
  <robot>_mujoco_bringup/      # launch files
  <robot>_mujoco_ros2/         # metapackage
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --merge-install --symlink-install \
  --base-paths src/robot/mujoco_ros2_control_menagerie \
  --packages-select \
    ai_worker_mujoco_description ai_worker_mujoco_bringup ai_worker_mujoco_ros2 \
    g1_mujoco_description g1_mujoco_bringup g1_mujoco_ros2 \
    rby1_mujoco_description rby1_mujoco_bringup rby1_mujoco_ros2 \
    mujoco_ros2_control_menagerie

source install/setup.bash
```

## Launch

Run one bringup at a time unless you intentionally isolate them with different
ROS domains or namespaces.

### AI Worker

```bash
ros2 launch ai_worker_mujoco_bringup robot.launch.py
```

Examples:

```bash
ros2 launch ai_worker_mujoco_bringup robot.launch.py robot_model:=ffw_bh5
ros2 launch ai_worker_mujoco_bringup robot.launch.py robot_model:=ffw_sg2
```

### Unitree G1

```bash
ros2 launch g1_mujoco_bringup robot.launch.py
```

Model variants:

| `robot_model` | Default MuJoCo file | Controllers | Base behavior |
| --- | --- | --- | --- |
| `g1` | `scene.xml` | body, arms, torso, legs | floating base |
| `g1_with_hands` | `scene_with_hands_fixed.xml` | body, arms, torso, legs, hands | pelvis welded to world |
| `g1_with_inspire_hands` | `scene_inspire_hand_fixed.xml` | body, arms, torso, legs, Inspire hands | pelvis welded to world |

Examples:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_inspire_hands
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands mujoco_model_file:=scene_with_hands.xml
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_inspire_hands mujoco_model_file:=scene_inspire_hand.xml
ros2 launch g1_mujoco_bringup robot.launch.py mujoco_model_file:=g1_29dof_fixed.xml
```

G1 hand variants default to fixed-base scenes so arm and hand control can be
tested without a balance controller. Override `mujoco_model_file` with
`scene_with_hands.xml` or `scene_inspire_hand.xml` when you explicitly want the
original floating-base hand scene.

Useful G1 launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `robot_model` | `g1` | `g1`, `g1_with_hands`, or `g1_with_inspire_hands` |
| `mujoco_model_file` | `auto` | MJCF file under `g1_mujoco_description/mjcf` |
| `controllers_yaml` | `auto` | Controller YAML selected by `robot_model` |
| `initial_positions_file` | package default | Initial joint positions YAML |
| `log_level` | `info` | ROS log level |

### RBY1

```bash
ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=a
```

Examples:

```bash
ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=m robot_version:=v1.2
ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=a_wuji robot_version:=v1.2
ros2 launch rby1_mujoco_bringup robot.launch.py robot_model:=m_wuji robot_version:=v1.2
```

`robot_model:=a_wuji` and `robot_model:=m_wuji` are v1.2-only variants. They
use the RBY1A/RBY1M bodies and replace the stock grippers with separate Wuji
hand controllers.

Useful RBY1 launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `robot_model` | `a` | `a`, `m`, `a_wuji`, or `m_wuji` |
| `robot_version` | `v1.2` | RBY1 model version for non-Wuji variants |
| `controllers_yaml` | `auto` | Controller YAML selected by `robot_model` |
| `log_level` | `info` | ROS log level |

## Common ROS Interface

All bringup files use the same top-level control and sensor namespaces.

### Description and controller manager

```text
/control/body/robot_description
/control/body/controller_manager
```

### Body trajectory controllers

Available controllers depend on the robot model, but the naming convention is
kept stable:

```text
/control/body/arm_left_controller/joint_trajectory
/control/body/arm_right_controller/joint_trajectory
/control/body/torso_controller/joint_trajectory
/control/body/head_controller/joint_trajectory
/control/body/leg_controller/joint_trajectory
```

The corresponding `FollowJointTrajectory` actions live beside those topics:

```text
/control/body/<controller_name>/follow_joint_trajectory
```

### Hand trajectory controllers

Robots with separate hand controllers use:

```text
/control/hand_left/hand_left_controller/joint_trajectory
/control/hand_right/hand_right_controller/joint_trajectory
/control/hand_left/hand_left_controller/follow_joint_trajectory
/control/hand_right/hand_right_controller/follow_joint_trajectory
```

### Proprioception

```text
/sensors/proprio/body/joint_states
/sensors/proprio/body/dynamic_joint_states
/sensors/proprio/hand_left/joint_states
/sensors/proprio/hand_left/dynamic_joint_states
/sensors/proprio/hand_right/joint_states
/sensors/proprio/hand_right/dynamic_joint_states
```

RBY1 `a` and `m` publish all joint states through body proprioception. AI Worker,
G1 hand variants, and RBY1 Wuji variants split hand proprioception into
`hand_left` and `hand_right` when hand controllers are present.

### Mobile base

AI Worker `ffw_sg2` and `ffw_sh5` expose:

```text
/control/body/base_steer_controller/joint_trajectory
/control/body/base_drive_controller/commands
```

The drive command is `std_msgs/msg/Float64MultiArray` in left, right, rear wheel
order.

RBY1 `a` and `m` expose:

```text
/cmd_vel
```

## rqt Joint Trajectory Controller

For body controllers:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller \
  --clear-config --force-discover \
  --ros-args \
  -r robot_description:=/control/body/robot_description
```

In the GUI, select:

```text
Controller manager: /control/body/controller_manager
```

For hand controllers, some models remap the controller runtime namespace to
`/control/hand_left` and `/control/hand_right`. For G1 hand variants, run:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller \
  --clear-config --force-discover \
  --ros-args \
  -r robot_description:=/control/body/robot_description \
  -r /control/body/hand_left_controller/controller_state:=/control/hand_left/hand_left_controller/controller_state \
  -r /control/body/hand_left_controller/joint_trajectory:=/control/hand_left/hand_left_controller/joint_trajectory \
  -r /control/body/hand_right_controller/controller_state:=/control/hand_right/hand_right_controller/controller_state \
  -r /control/body/hand_right_controller/joint_trajectory:=/control/hand_right/hand_right_controller/joint_trajectory
```

In the GUI, select `/control/body/controller_manager`, then choose
`hand_left_controller` or `hand_right_controller`.

## Notes

- Launch files are MuJoCo-only.
- If topic discovery looks stale, stop the old launch process before starting a new robot.
- `source /opt/ros/jazzy/setup.bash` before building or launching from a clean shell.
- Keep each robot variant's MuJoCo file, URDF/xacro wrapper, and controller YAML
  on the same joint set. Hand joints must exist in both the URDF and the MJCF.
