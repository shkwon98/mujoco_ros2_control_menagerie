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
  oriented so the thumb follows flange +X; M v1.3 retains its 66.384 mm mounting offset. No intermediate
  `left_hand_base` or `right_hand_base` frame is created.

For example:

```bash
ros2 launch g1_mujoco_bringup robot.launch.py robot_model:=g1_with_hands
ros2 launch rb_mujoco_bringup robot.launch.py \
  robot_model:=rb5_850e hand_model:=wuji_hand2 hand_side:=right
ros2 launch rby1_mujoco_bringup robot.launch.py \
  robot_model:=a_wuji hand_model:=wuji_hand2
```

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

All nodes use `use_sim_time=true`; message timestamps follow `/clock`.
Mobile robots use `map -> odom -> base_link` TF; fixed FFW models use
`map -> base_link`, RB-Series uses `map -> link0`, and G1 uses `map -> pelvis`
(static for fixed scenes, dynamic for floating scenes). Floating G1 also
publishes `/sensors/proprio/body/base_pose`. RB-Series retains the native `tcp` frame
and exposes a mounting-face-centred `flange` with +Z pointing outward toward the tool.

Mobile ALOHA, RB and RBY1 add these fixed flange frames in their enclosing
xacro descriptions while preserving the original robot URDF/model xacro files.
`robot_state_publisher` publishes their transforms from the expanded description.
Mounting directions are defined with all arm joints at zero. RB flange +X follows
TCP +X, +Y points upward, and +Z follows TCP -Y. Hand1 and Hand2 on either side
use the same single `flange` frame: thumbs follow +X and fingers follow +Z.
The right dorsum follows +Y (up) and the left dorsum follows -Y (down).
RBY1 flanges use native
`ee_left`/`ee_right` +X as +X and native -Z as +Z: left -Y and right +Y represent
the outward-facing dorsum. Hand1 and Hand2 thumbs follow flange +X, and fingers
extend along flange +Z. Hand1's adapter rotation is aligned to these axes while
retaining its docking-to-palm geometry and mounting offsets in URDF and MuJoCo.
The direct right Hand2 mount on `rb20_1900es_u` intersects the wrist and thumb
geometry at joint zero; its `rb_wuji` clearance check fails.

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
