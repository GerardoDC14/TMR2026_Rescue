# DLS Servo — Singularity-Robust Arm Control

Drop-in replacement for MoveIt Servo that eliminates singularity hard-stops using **Damped Least Squares (DLS)**.

---

## Problem

MoveIt Servo uses the Jacobian pseudoinverse (`dq = J+ * dx`) for real-time Cartesian control. When the Jacobian's condition number exceeds a threshold (300 in the original config), Servo **halts the arm completely**. This happens frequently with the Jaguar arm's wrist singularity — when Joint5 approaches 0, Joint4 and Joint6 axes become collinear, the condition number spikes, and the arm stops mid-motion.

Plan&Execute works fine because OMPL plans in joint space and can route around singularities. Servo works in Cartesian space and hits them head-on.

## Solution

The DLS servo node (`damped_servo.py`) replaces the Jacobian pseudoinverse with:

```
dq = J^T (J*J^T + lambda^2 * I)^-1 * dx
```

Where `lambda` ramps up adaptively as the condition number grows:

| Condition number | Lambda | Behavior |
|---|---|---|
| < 40 | 0 | Perfect Cartesian tracking (identical to pseudoinverse) |
| 40 – 200 | 0 → 0.1 | Gradually sacrifices accuracy for stability |
| > 200 | 0.1 (max) | Maximum damping — arm still moves, just approximately |

**The arm never halts.** Near singularities, it becomes "stiff" in certain directions (which is physically correct) rather than stopping dead.

Verified numerically: at the worst-case wrist singularity (q=0), DLS produces max joint velocity of **0.24 rad/s** vs pseudoinverse's **2.13 rad/s** — a 9x reduction that keeps motion safe and bounded.

---

## Files created/modified

### New files

| File | Description |
|------|-------------|
| `src/jaguar_teleop/jaguar_teleop/damped_servo.py` | The DLS servo ROS2 node |
| `src/jaguar_full/config/damped_servo_params.yaml` | Tunable parameters |
| `src/jaguar_full/launch/damped_servo.launch.py` | Launch file (replaces `servo.launch.py`) |

### Modified files

| File | Change |
|------|--------|
| `src/jaguar_teleop/setup.py` | Added `damped_servo` entry point |

### Original files preserved

`servo.launch.py` and `servo_params.yaml` are **untouched** — you can always revert to MoveIt Servo by launching `servo.launch.py` instead.

---

## Usage

The only change to the launch sequence is **step 2** — replace `servo.launch.py` with `damped_servo.launch.py`:

```bash
# 1. MoveIt  (move_group, robot_state_publisher, RViz)
ros2 launch jaguar_full demo.launch.py

# 2. DLS Servo  (replaces: ros2 launch jaguar_full servo.launch.py)
ros2 launch jaguar_full damped_servo.launch.py

# 3. Brushless motors  (ODrive via Ginkgo USB-CAN)
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py verbose:=true verbose_period_s:=0.5

# 4. Joystick control  (joy driver + joystick_servo + serial bridge)
ros2 launch jaguar_teleop joystick.launch.py
```

Everything else (joystick mapping, serial bridge, ODrive bridge) works **exactly the same** — the DLS servo exposes identical topics and services as MoveIt Servo.

### Control modes (keyboard and joystick)

Both `keyboard_servo` and `joystick_servo` have three modes, cycled with `m` (keyboard) or `Start` (joystick):

| Mode | Frame | Description |
|------|-------|-------------|
| **CART-GLOBAL** | base_link | Cartesian control in world frame (default) |
| **CART-LOCAL** | Link6 | Cartesian control in end-effector frame — "forward" means wherever the EE is pointing |
| **JOINT** | — | Direct joint-by-joint control |

In **CART-LOCAL** mode, all linear/angular commands are relative to the end effector's current orientation. For example, if the EE is pointing straight down, pressing "forward" (+X) moves the arm downward (-Z global).

### Collision checking

Self-collision checking uses the URDF collision meshes and the SRDF's allowed collision matrix. Specifically:

- **Link1 vs base_link**: disabled (adjacent, always touching)
- **Link2–6 vs base_link**: **checked** — prevents arm from crashing into its own base
- **Adjacent links**: disabled (they share a joint)
- **Non-adjacent arm links** (e.g., Link6 vs Link2): **checked**

The `collision_lookahead` parameter (default 1.5) checks 50% further ahead than the actual motion, creating a safety buffer so the arm stops *before* contact rather than exactly at it.

### Rebuilding after changes

```bash
cd ~/TMR2026_Rescue/jaguar/arm
source /opt/ros/humble/setup.bash
colcon build --packages-select jaguar_teleop jaguar_full --symlink-install
source install/setup.bash
```

---

## Topic / service interface

Identical to MoveIt Servo — the node is named `servo_node` and uses the `~/` namespace convention:

| Interface | Name | Type |
|-----------|------|------|
| Subscribe | `/joint_states` | `sensor_msgs/JointState` |
| Subscribe | `/servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` |
| Subscribe | `/servo_node/delta_joint_cmds` | `control_msgs/JointJog` |
| Publish | `/jaguar_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| Publish | `/servo_node/status` | `std_msgs/Int8` |
| Service | `/servo_node/start_servo` | `std_srvs/Trigger` |
| Service | `/servo_node/pause_servo` | `std_srvs/Trigger` |

Status codes (same as MoveIt Servo, compatible with `joystick_servo.py`):

| Code | Meaning |
|------|---------|
| 0 | OK |
| 1 | Near singularity — degrading accuracy (never halts) |
| 2 | Singularity halt — **never emitted by DLS servo** |
| 3 | Leaving singularity |
| 4 | Collision detected — motion rejected |
| 5 | Near joint limit |

---

## Parameters

All tunable via `src/jaguar_full/config/damped_servo_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | 30.0 | Control loop Hz |
| `max_linear_speed` | 0.2 | Max Cartesian linear speed (m/s) |
| `max_rotational_speed` | 0.35 | Max Cartesian angular speed (rad/s) |
| `max_joint_speed` | 0.35 | Max joint velocity cap (rad/s) |
| `condition_lower` | 40.0 | Condition number where damping starts |
| `condition_upper` | 200.0 | Condition number where damping maxes out |
| `damping_max` | 0.1 | Maximum damping factor (lambda) |
| `joint_limit_margin` | 0.1 | Hard stop zone near joint limits (rad) |
| `joint_limit_scale_zone` | 0.25 | Soft deceleration ramp near limits (rad) |
| `command_timeout` | 0.2 | Zero commands if no input within this window (s) |
| `smoothing_alpha` | 0.4 | EMA filter on twist input (lower = smoother, laggier) |
| `check_collisions` | true | Enable self-collision checking via moveit_py |
| `collision_check_rate` | 10.0 | How often to run collision checks (Hz) |
| `collision_lookahead` | 1.5 | Check 50% further than actual motion (safety margin) |
| `self_collision_proximity_threshold` | 0.01 | Proximity threshold (m) |
| `ee_frame_name` | Link6 | End-effector frame name (for local Cartesian mode) |

### Tuning guide

- **Arm feels too sluggish near singularities**: Raise `condition_lower` (damping kicks in later) or lower `damping_max`
- **Arm oscillates near singularities**: Lower `condition_lower` (damping kicks in earlier) or raise `damping_max`
- **Joystick feels laggy**: Raise `smoothing_alpha` toward 1.0
- **Motion is jerky**: Lower `smoothing_alpha` toward 0.2
- **Arm stops too early at joint limits**: Reduce `joint_limit_margin` and `joint_limit_scale_zone`
- **Collision stops too early/late**: Adjust `collision_lookahead` (1.0 = exact, 2.0 = large margin)
- **Disable collision checking** (if not needed): Set `check_collisions: false` in the YAML

---

## Architecture

The DLS servo uses **pure numpy kinematics** for the Jacobian and **moveit_py** for collision checking. It parses the URDF at startup to build the kinematic chain and computes:

1. **Forward kinematics**: chain of 4x4 homogeneous transforms
2. **Geometric Jacobian**: standard formula using joint axes and positions
3. **DLS pseudoinverse**: `J^T (J*J^T + lambda^2*I)^-1` with adaptive lambda based on SVD condition number

This runs at 30 Hz with negligible CPU overhead (6x6 matrix operations).

### Data flow

```
joystick_servo
    │
    ├─ /servo_node/delta_twist_cmds  (Cartesian mode)
    └─ /servo_node/delta_joint_cmds  (Joint mode)
           │
      damped_servo (DLS)
           │
           ├─ EMA smoothing on twist input
           ├─ Jacobian computation from URDF chain
           ├─ SVD condition number → adaptive lambda
           ├─ DLS solve → joint velocities
           ├─ Joint velocity cap
           ├─ Joint limit soft deceleration
           ├─ Self-collision check (moveit_py, 10 Hz)
           │
           └─→ /jaguar_arm_controller/joint_trajectory
                    │
               JointTrajectoryController (open_loop_control: true)
                    │
               /joint_states → ODrive bridge + Serial bridge
```

---

## Dependencies

Standard ROS2 Humble packages plus MoveIt (compiled from source in this workspace):

- `numpy` (comes with Python)
- `urdf_parser_py` (comes with ROS2)
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `control_msgs`, `trajectory_msgs`, `std_msgs`, `std_srvs` (standard ROS2)
- `moveit_configs_utils` (used by launch file, part of MoveIt)
- `moveit_py` (collision checking — compiled from source in `arm/src/`)
