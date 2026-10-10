# MuJoCo ROS 2 Control Menagerie

ROS 2 Jazzy packages for running robot models in MuJoCo through `ros2_control`.

## Supported robots

| Robot | Launch package | `robot_model` values | Default |
| --- | --- | --- | --- |
| AgiBot G2 | `agibot_g2_mujoco_bringup` | — | — |
| AI Worker FFW | `ai_worker_mujoco_bringup` | `ffw_bg2`, `ffw_bh5`, `ffw_sg2`, `ffw_sh5` | `ffw_bg2` |
| Mobile ALOHA | `mobile_aloha_mujoco_bringup` | `vx300s`, `piper` | `vx300s` |
| Unitree G1 | `g1_mujoco_bringup` | `g1`, `g1_with_hands`, `g1_with_inspire_hands` | `g1` |
| Rainbow Robotics RB-Series | `rb_mujoco_bringup` | See below | Required |
| RBY1 | `rby1_mujoco_bringup` | `a`, `m`, `a_wuji`, `m_wuji` | `a` |

RB-Series models:

| Family | `robot_model` values |
| --- | --- |
| RB1 | `rb1_500es_u` |
| RB3 | `rb3_730es_u`, `rb3_1200e`, `rb3_1200e_u` |
| RB5 | `rb5_850e`, `rb5_850e_u` |
| RB6 | `rb6_1700e_u` |
| RB10 | `rb10_1300e`, `rb10_1300e_u` |
| RB16 | `rb16_900e`, `rb16_900e_u` |
| RB20 | `rb20_1800e_u`, `rb20_1900es_u` |
| RB30 | `rb30_1400es_u` |

## Build

Run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src/robot/mujoco_ros2_control_menagerie \
  --ignore-src --rosdistro jazzy -r -y

colcon build --merge-install --symlink-install \
  --base-paths src/robot/mujoco_ros2_control_menagerie

source install/setup.bash
```

RB and RBY1 include Wuji Hand1 and Hand2 URDF, MJCF and mesh assets
in their description packages; no external Wuji description packages are required.
The public hand model names are `wuji_hand` and `wuji_hand2`; `wuji_hand2`
always uses the bundled `wuji_hand2_beta2` assets. Beta names are not accepted
as `hand_model` arguments.
RB Hand1 assets come from [wuji-description](https://github.com/wuji-technology/wuji-description)
revision `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`; RB Hand2 assets use the same
bundled models as RBY1. Each description package includes `LICENSE.wuji_description`.

## Launch

Choose one command; run one robot per ROS domain:

```bash
ros2 launch agibot_g2_mujoco_bringup robot.launch.py
ros2 launch ai_worker_mujoco_bringup robot.launch.py
ros2 launch mobile_aloha_mujoco_bringup robot.launch.py
ros2 launch g1_mujoco_bringup robot.launch.py
ros2 launch rb_mujoco_bringup robot.launch.py robot_model:=rb5_850e
ros2 launch rby1_mujoco_bringup robot.launch.py
```

| Argument | Default | Purpose |
| --- | --- | --- |
| `headless` | `false` | Run without the MuJoCo viewer |
| `use_rqt` | `false` | Open the joint trajectory controller GUI in monitor mode |
| `log_level` | `info` | ROS log level; wheeled base loggers stay at `ERROR` |
| `use_navigation` | `true` | Start Nav2 on mobile models; disable for direct base control |

G1 and RB-Series have no Nav2 pipeline. To list a package's arguments:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py --show-args
```

Model options:

- FFW: `g2` variants use grippers, `h5` variants use dexterous hands;
  `ffw_sg2` and `ffw_sh5` have mobile bases.
- Mobile ALOHA: `piper` uses four arms, with front followers and rear leaders.
- G1: `g1` has a floating base; hand variants default to fixed bases.
  Select `mujoco_model_file:=scene_with_hands.xml` or `scene_inspire_hand.xml`
  for the corresponding floating-base hand model.
- RB-Series: `hand_model` defaults to `none`; options are `wuji_hand`,
  `wuji_hand2`. `hand_side` selects `right` (default)
  or `left`, using a simulated mount attached directly to `flange`.
  Hand1 retains its docking adapter; Hand2 attaches its native mount link.
  Hand descriptions use `flange` as their shared root with the body description;
  no `left_hand_base` or `right_hand_base` intermediate frame is created.
- RBY1: `a_wuji` and `m_wuji` use Wuji hands. `hand_model` defaults to
  `wuji_hand`; select `wuji_hand2` for Hand2.
  `robot_version` defaults to `v1.2`; A models support `v1.0`–`v1.2`,
  M models support `v1.0`–`v1.3`.
  Wuji hands attach directly to `left_flange` and `right_flange`, which are also
  the shared roots for body/hand descriptions. Hand1 keeps its docking adapter,
  oriented so fingers follow flange +X and the thumb flange +Z; M v1.3 retains its 66.384 mm mounting offset. No intermediate
  `left_hand_base` or `right_hand_base` frame is created.

For example:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
ros2 launch rb_mujoco_bringup robot.launch.py \
  robot_model:=rb5_850e hand_model:=wuji_hand2 hand_side:=right
ros2 launch rby1_mujoco_bringup robot.launch.py \
  robot_model:=a_wuji hand_model:=wuji_hand2
```

### Multiple RB instances

Only RB bringup currently supports instance-specific interfaces. Omit `instance_id`
to retain the existing single-robot paths and simulation-time behavior.

```bash
# Separate terminals; each simulation contains its arm and attached hand.
ros2 launch rb_mujoco_bringup robot.launch.py \
  robot_model:=rb5_850e instance_id:=rb_01 \
  hand_model:=wuji_hand2 hand_instance_id:=wuji_hand_01 hand_side:=left \
  base_xyz:="0 0.5 0"
ros2 launch rb_mujoco_bringup robot.launch.py \
  robot_model:=rb5_850e instance_id:=rb_02 \
  hand_model:=wuji_hand2 hand_instance_id:=wuji_hand_01 hand_side:=right \
  base_xyz:="0 -0.5 0"
```

Body interfaces use `/<instance_id>`; hand feedback and descriptions use
`/<hand_instance_id>/<side>_eef`, with commands on
`/<hand_instance_id>/<side>_eef_controller`. Joint names stay unchanged; TF prefixes
are `<instance_id>_` and `<hand_instance_id>_<side>_`. RViz uses the separate
`visualization/robot_description` topic under each body/hand namespace, with
matching link names and an empty TF Prefix.

`base_xyz` and `base_rpy` place each base in `map` (metres/radians). Named instances
use system time by default; shared teleop should use `use_sim_time:=false`.
Explicit `use_sim_time:=true` uses that instance's private `/<instance_id>/clock`.
Each arm and hand share one controller manager and MuJoCo world. Separate RB
instances have no physical interaction or collision checks against one another.

## Common ROS interface

| Endpoint | Purpose |
| --- | --- |
| `/controller_manager` | One controller manager for the robot |
| `/robot_description` | Full control URDF, including `ros2_control` metadata |
| `/control/<segment>/robot_description` | Body or hand URDF without control metadata |
| `/sensors/proprio/<segment>/joint_states` | `sensor_msgs/msg/JointState` |
| `/sensors/proprio/<segment>/dynamic_joint_states` | Joint interface states |
| `/control/body/<controller>/joint_trajectory` | Body `trajectory_msgs/msg/JointTrajectory` commands |
| `/control/hand_<side>/hand_<side>_controller/joint_trajectory` | Hand or follower-gripper trajectory commands |

`<segment>` is `body`, `hand_left` or `hand_right`; `<side>` is `left` or `right`.
Hand topics exist for models with separate moving hand joints. Description topics
publish `std_msgs/msg/String` with reliable, transient-local QoS. Each trajectory
controller also provides `controller_state` and a `follow_joint_trajectory` action
under the same prefix.

RB-Series uses `arm_controller`; dual-arm robots use `arm_left_controller` and
`arm_right_controller`, with torso, head or leg controllers where available.
G2 grippers provide state only. RBY1 stock grippers remain in the body interface;
PiPER rear leaders have additional `leader_*_controller` body interfaces.

Legacy launches use `use_sim_time=true`; message timestamps follow `/clock`. Named RB instances use the time policy described above.
Mobile robots use `map -> odom -> base_link` TF; fixed FFW models use
`map -> base_link`, RB-Series uses `map -> link0`, and G1 uses `map -> pelvis`
(static for fixed scenes, dynamic for floating scenes). Floating G1 also
publishes `/sensors/proprio/body/base_pose`. RB-Series retains the native `tcp` frame
and exposes a mounting-face-centred `flange` with +X pointing outward toward the tool.

Mobile ALOHA, RB and RBY1 add fixed ROS-Industrial flange frames in the enclosing
xacro descriptions. RB keeps one flange: at joint zero X points right, Y forward,
and Z up. RBY1 has left/right flanges: at joint zero X points down, Y left and Z
forward; with the elbow bent forward the reference is X forward, Y left, Z up.
VX300S and PiPER have forward/left/up flanges at model zero. PiPER joint5 is
re-zeroed by `0.0872495900012399` rad (approximately 5 degrees) on all four arms;
URDF and MuJoCo limits and initial positions are shifted together. Native hardware
angles satisfy `q_native = q_model + offset`; driver state/commands and published
TF must use a consistent convention. Existing startup physical postures are preserved.

Hand1/Hand2 fingers follow flange +X and thumbs follow flange +Z on both sides.
Hand1 docking geometry and translations, and the RBY1 M v1.3 axial correction,
are retained. RBY1 physical hand orientations stay unchanged: thumbs forward,
dorsa outward. RB thumbs now point up. Native gripper geometry is unchanged.
URDF/MuJoCo composition tests cover both hands and the supported robot variants.

### Mobile bases

G2, FFW `ffw_sg2`/`ffw_sh5`, Mobile ALOHA and RBY1 expose:

| Topic/action | Type |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/follow_path` | Nav2 path-following action, when navigation is enabled |

Stamp velocity commands with `/clock` and express them in `base_link`
(an empty `header.frame_id` also denotes this frame). Linear velocity uses m/s;
angular velocity uses rad/s. Differential bases support forward motion and yaw;
swerve/mecanum bases also support lateral motion. Commands that expire trigger
braking. `use_navigation:=false` leaves direct `/cmd_vel` control available.
