# Unitree G1 MuJoCo ROS 2 Control

Unitree G1 MuJoCo bringup for `ros2_control`.

The package is useful for upper-body and hand-control work without a physical
robot. It provides the MuJoCo model, URDF/xacro wrapper, controller YAML, and
launch file needed by `ros2_control_node`.

## Packages

```text
g1_mujoco_description/  # URDF, MJCF, meshes, ros2_control config
g1_mujoco_bringup/      # robot.launch.py
g1_mujoco_ros2/         # metapackage
```

## Models

| `robot_model` | Default MuJoCo file | Controllers | Base behavior |
| --- | --- | --- | --- |
| `g1` | `scene.xml` | body, arms, torso, legs | floating base |
| `g1_with_hands` | `scene_with_hands_fixed.xml` | body, arms, torso, legs, hands | pelvis welded to world |

The hand model defaults to a fixed-base scene so arm and hand control can be
tested without a balance controller. The original floating-base hand scene is
still available as `scene_with_hands.xml`.

Additional compatibility files are kept under `mjcf/`:

```text
g1_29dof.xml
g1_29dof_fixed.xml
g1.xml
g1_with_hands.xml
scene.xml
scene_with_hands.xml
scene_with_hands_fixed.xml
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --merge-install --symlink-install \
  --base-paths src/robot/mujoco_ros2_control_menagerie \
  --packages-select g1_mujoco_description g1_mujoco_bringup g1_mujoco_ros2

source install/setup.bash
```

## Launch

Body-only floating-base model:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py
```

Hand model fixed to the world:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
```

Hand model with the original floating base:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py \
  robot_model:=g1_with_hands \
  mujoco_model_file:=scene_with_hands.xml
```

Legacy fixed-base 29-DoF model:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py mujoco_model_file:=g1_29dof_fixed.xml
```

Useful launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `robot_model` | `g1` | `g1` or `g1_with_hands` |
| `mujoco_model_file` | `auto` | MJCF file under `g1_mujoco_description/mjcf` |
| `controllers_yaml` | `auto` | Controller YAML selected by `robot_model` |
| `initial_positions_file` | package default | Initial joint positions YAML |
| `log_level` | `info` | ROS log level |

## Control Topics

Body controllers:

```text
/control/body/arm_left_controller/joint_trajectory
/control/body/arm_right_controller/joint_trajectory
/control/body/torso_controller/joint_trajectory
/control/body/leg_controller/joint_trajectory
```

Hand controllers for `robot_model:=g1_with_hands`:

```text
/control/hand_left/hand_left_controller/joint_trajectory
/control/hand_right/hand_right_controller/joint_trajectory
/control/hand_left/hand_left_controller/follow_joint_trajectory
/control/hand_right/hand_right_controller/follow_joint_trajectory
```

Proprioception:

```text
/sensors/proprio/body/joint_states
/sensors/proprio/body/dynamic_joint_states
/sensors/proprio/hand_left/joint_states
/sensors/proprio/hand_left/dynamic_joint_states
/sensors/proprio/hand_right/joint_states
/sensors/proprio/hand_right/dynamic_joint_states
```

## rqt Joint Trajectory Controller

Start the robot first:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
```

Then start rqt in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller \
  --clear-config --force-discover \
  --ros-args \
  -r robot_description:=/control/body/robot_description \
  -r /control/body/hand_left_controller/controller_state:=/control/hand_left/hand_left_controller/controller_state \
  -r /control/body/hand_left_controller/joint_trajectory:=/control/hand_left/hand_left_controller/joint_trajectory \
  -r /control/body/hand_right_controller/controller_state:=/control/hand_right/hand_right_controller/controller_state \
  -r /control/body/hand_right_controller/joint_trajectory:=/control/hand_right/hand_right_controller/joint_trajectory
```

In the GUI:

```text
Controller manager: /control/body/controller_manager
Controller: hand_left_controller or hand_right_controller
```

Wait until the selected controller has published `controller_state` before
enabling command mode. If command mode is enabled too early, the rqt plugin can
try to command before it has a current joint position.

## Model Consistency

Keep these three files on the same G1 variant:

- MuJoCo model file
- URDF/xacro wrapper
- Controller YAML joint list

For `robot_model:=g1_with_hands`, the hand joints must exist in both the URDF
and the MJCF. For `robot_model:=g1`, use the body-only controller YAML.
