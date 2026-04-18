#!/usr/bin/env python3
"""
Damped Least-Squares Servo Node
================================
Drop-in replacement for MoveIt Servo that uses *Damped Least Squares* (DLS)
for Cartesian-to-joint velocity mapping.  Near singularities the damping
factor rises smoothly so the arm **never halts** — it trades Cartesian
tracking accuracy for joint-space stability instead.

Topic / service interface is identical to moveit_servo so the existing
joystick_servo node works unchanged.

Theory
------
Standard pseudoinverse:  dq = J+ dx        (blows up near singularities)
DLS:                     dq = J^T (J J^T + lambda^2 I)^-1 dx

lambda is zero far from singularities (exact tracking) and ramps up as the
Jacobian condition number grows, keeping joint velocities bounded.
"""

import numpy as np
import threading
import tempfile
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration

from urdf_parser_py.urdf import URDF
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
N_JOINTS = 6

# Status codes — compatible with MoveIt Servo / joystick_servo.py
STATUS_OK        = 0
STATUS_DECEL     = 1   # near singularity, degrading accuracy
STATUS_HALT      = 2   # we never emit this
STATUS_LEAVING   = 3   # leaving singularity region
STATUS_COLLISION = 4
STATUS_JLIMIT    = 5

# ---------------------------------------------------------------------------
# Pure-numpy kinematics  (no KDL / external dependency)
# ---------------------------------------------------------------------------

def _rpy_to_rot(r, p, y):
    """Roll-Pitch-Yaw (XYZ extrinsic) to 3x3 rotation matrix."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,             cp*cr  ],
    ])


def _origin_to_T(xyz, rpy):
    """URDF origin (xyz, rpy) to 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


def _Rz(angle):
    """4x4 rotation about local Z."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [ c, -s, 0, 0],
        [ s,  c, 0, 0],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1.],
    ])


class ArmKinematics:
    """FK + geometric Jacobian for a serial revolute chain parsed from URDF."""

    def __init__(self, urdf_string, base='base_link', tip='Link6'):
        robot = URDF.from_xml_string(urdf_string)
        by_child = {j.child: j for j in robot.joints}

        # Trace kinematic path from tip back to base
        path, current = [], tip
        while current != base:
            joint = by_child[current]
            path.append(joint)
            current = joint.parent
        path.reverse()

        self.origins = []        # fixed 4x4 transforms
        self.is_actuated = []    # bool per joint in path
        self.lower = []
        self.upper = []

        for j in path:
            xyz = list(j.origin.xyz) if j.origin and j.origin.xyz else [0, 0, 0]
            rpy = list(j.origin.rpy) if j.origin and j.origin.rpy else [0, 0, 0]
            self.origins.append(_origin_to_T(xyz, rpy))

            act = j.type in ('revolute', 'continuous')
            self.is_actuated.append(act)
            if act:
                self.lower.append(j.limit.lower if j.limit else -np.pi)
                self.upper.append(j.limit.upper if j.limit else  np.pi)

        self.lower = np.array(self.lower)
        self.upper = np.array(self.upper)
        self.n_act = int(sum(self.is_actuated))

    def fk(self, q):
        """Forward kinematics.

        Returns
        -------
        T_ee : (4,4) end-effector transform in base frame
        z_axes : list of (3,) joint Z-axes in base frame (before rotation)
        p_joints : list of (3,) joint origins in base frame (before rotation)
        """
        T = np.eye(4)
        z_axes, p_joints = [], []
        qi = 0

        for k, origin in enumerate(self.origins):
            T = T @ origin
            if self.is_actuated[k]:
                z_axes.append(T[:3, 2].copy())
                p_joints.append(T[:3, 3].copy())
                T = T @ _Rz(q[qi])
                qi += 1

        return T, z_axes, p_joints

    def jacobian(self, q):
        """6 x n_act geometric Jacobian in base frame.

        Rows 0-2 : linear velocity
        Rows 3-5 : angular velocity
        """
        T_ee, z_axes, p_joints = self.fk(q)
        p_ee = T_ee[:3, 3]

        J = np.zeros((6, self.n_act))
        for i in range(self.n_act):
            J[:3, i] = np.cross(z_axes[i], p_ee - p_joints[i])
            J[3:, i] = z_axes[i]
        return J


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class DampedServo(Node):

    def __init__(self):
        super().__init__('servo_node')   # same name → same ~/topic namespace

        # ── declare parameters ────────────────────────────────────────
        self.declare_parameter('robot_description', '')
        self.declare_parameter('robot_description_semantic', '')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_rotational_speed', 0.35)
        self.declare_parameter('max_joint_speed', 0.35)
        self.declare_parameter('condition_lower', 40.0)
        self.declare_parameter('condition_upper', 200.0)
        self.declare_parameter('damping_max', 0.1)
        self.declare_parameter('joint_limit_margin', 0.1)
        self.declare_parameter('joint_limit_scale_zone', 0.25)
        self.declare_parameter('command_timeout', 0.2)
        self.declare_parameter('smoothing_alpha', 0.4)
        self.declare_parameter('check_collisions', True)
        self.declare_parameter('collision_check_rate', 10.0)
        self.declare_parameter('collision_lookahead', 1.5)
        self.declare_parameter('self_collision_proximity_threshold', 0.01)
        self.declare_parameter('ee_frame_name', 'Link6')

        urdf_string = self.get_parameter('robot_description').value
        if not urdf_string:
            self.get_logger().fatal('robot_description parameter is empty!')
            raise RuntimeError('robot_description not set')

        self.publish_rate   = self.get_parameter('publish_rate').value
        self.max_lin        = self.get_parameter('max_linear_speed').value
        self.max_rot        = self.get_parameter('max_rotational_speed').value
        self.max_jvel       = self.get_parameter('max_joint_speed').value
        self.cond_lower     = self.get_parameter('condition_lower').value
        self.cond_upper     = self.get_parameter('condition_upper').value
        self.damp_max       = self.get_parameter('damping_max').value
        self.jl_margin      = self.get_parameter('joint_limit_margin').value
        self.jl_scale_zone  = self.get_parameter('joint_limit_scale_zone').value
        self.cmd_timeout    = self.get_parameter('command_timeout').value
        self.alpha          = self.get_parameter('smoothing_alpha').value
        self.check_collisions = self.get_parameter('check_collisions').value
        self.collision_rate = self.get_parameter('collision_check_rate').value
        self.coll_lookahead = self.get_parameter('collision_lookahead').value
        self.ee_frame       = self.get_parameter('ee_frame_name').value

        # ── kinematics ────────────────────────────────────────────────
        self.kin = ArmKinematics(urdf_string)
        assert self.kin.n_act == N_JOINTS, \
            f'Expected {N_JOINTS} actuated joints, got {self.kin.n_act}'

        # ── collision checking (moveit_py) ────────────────────────────
        self._collision_ok = True   # last collision check result
        self._coll_counter = 0      # cycle counter for rate-limiting
        srdf_string = self.get_parameter('robot_description_semantic').value

        if self.check_collisions and srdf_string:
            self._coll_skip = max(1, int(round(
                self.publish_rate / self.collision_rate)))
            try:
                urdf_tmp = self._write_tmp(urdf_string, '.urdf')
                srdf_tmp = self._write_tmp(srdf_string, '.srdf')
                self._robot_model = RobotModel(urdf_tmp, srdf_tmp)
                self._planning_scene = PlanningScene(self._robot_model)
                self._robot_state = RobotState(self._robot_model)
                self.get_logger().info(
                    f'Collision checking enabled @ {self.collision_rate:.0f} Hz '
                    f'(every {self._coll_skip} cycles)')
            except Exception as e:
                self.get_logger().warn(
                    f'Collision checking init failed: {e} — running without')
                self.check_collisions = False
        elif self.check_collisions:
            self.get_logger().warn(
                'robot_description_semantic not provided — '
                'collision checking disabled')
            self.check_collisions = False

        # ── mutable state (guarded by _lock) ──────────────────────────
        self._lock          = threading.Lock()
        self._q             = np.zeros(N_JOINTS)
        self._q_valid       = False
        self._twist_cmd     = np.zeros(6)
        self._twist_frame   = 'base_link'  # or ee_frame for local mode
        self._filtered_tw   = np.zeros(6)
        self._joint_cmd     = None        # (index, velocity) or None
        self._last_cmd_time = self.get_clock().now()
        self._running       = False       # starts paused, like MoveIt Servo
        self._last_status   = STATUS_OK

        # ── publishers ────────────────────────────────────────────────
        self.traj_pub   = self.create_publisher(
            JointTrajectory, '/jaguar_arm_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(Int8, '~/status', 10)

        # ── subscribers ───────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            TwistStamped, '~/delta_twist_cmds', self._twist_cb, 10)
        self.create_subscription(
            JointJog, '~/delta_joint_cmds', self._joint_jog_cb, 10)

        # ── solver mode switching (GUI topic) ─────────────────────────
        # Publish: "servo" when DLS servo is active, "planner" when paused
        # Subscribe: GUI sends "servo" or "planner" to switch
        # transient_local so a late-joining GUI (or a GUI restart) always
        # sees the current solver state without needing the user to toggle.
        solver_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self._solver_mode_pub = self.create_publisher(
            String, '/arm/solver_mode', solver_qos)
        self.create_subscription(
            String, '/arm/solver_mode', self._solver_mode_cb, solver_qos)
        # Emit the initial state (paused) so subscribers don't have to wait
        # for the first start/pause transition to learn the current mode.
        self._publish_solver_mode()

        # ── services (MoveIt Servo compatible) ────────────────────────
        cb_group = ReentrantCallbackGroup()
        self.create_service(
            Trigger, '~/start_servo', self._srv_start, callback_group=cb_group)
        self.create_service(
            Trigger, '~/pause_servo', self._srv_pause, callback_group=cb_group)

        # ── main control timer ────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self.publish_rate, self._control_loop)

        self.get_logger().info(
            f'DLS Servo ready — cond=[{self.cond_lower}, {self.cond_upper}]  '
            f'lambda_max={self.damp_max}  {self.publish_rate:.0f} Hz')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _write_tmp(content, suffix):
        """Write string to a temp file and return the path."""
        fd, path = tempfile.mkstemp(suffix=suffix)
        with os.fdopen(fd, 'w') as f:
            f.write(content)
        return path

    def _is_colliding(self, q_new, q_current):
        """Check if a joint configuration is in self-collision.

        Uses a lookahead: checks at q_current + (q_new - q_current) * lookahead
        so the arm stops *before* actually colliding (safety margin).

        Rate-limited: only runs every N cycles (set by collision_check_rate).
        Returns the cached result on skipped cycles.
        """
        if not self.check_collisions:
            return False

        self._coll_counter += 1
        if self._coll_counter < self._coll_skip:
            return not self._collision_ok
        self._coll_counter = 0

        # Check at lookahead position for safety margin
        dq = q_new - q_current
        q_check = q_current + dq * self.coll_lookahead
        q_check = np.clip(q_check, self.kin.lower, self.kin.upper)

        self._robot_state.set_joint_group_positions('jaguar_arm', q_check)
        self._robot_state.update()
        colliding = self._planning_scene.is_state_colliding(
            self._robot_state, 'jaguar_arm', False)
        self._collision_ok = not colliding
        return colliding

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    self._q[i] = msg.position[msg.name.index(name)]
            self._q_valid = True

    def _twist_cb(self, msg: TwistStamped):
        t = msg.twist
        with self._lock:
            self._twist_cmd = np.array([
                t.linear.x,  t.linear.y,  t.linear.z,
                t.angular.x, t.angular.y, t.angular.z,
            ])
            self._twist_frame = msg.header.frame_id
            self._joint_cmd = None
            self._last_cmd_time = self.get_clock().now()

    def _joint_jog_cb(self, msg: JointJog):
        if not msg.joint_names or not msg.velocities:
            return
        name = msg.joint_names[0]
        if name in JOINT_NAMES:
            with self._lock:
                self._joint_cmd = (JOINT_NAMES.index(name), msg.velocities[0])
                self._twist_cmd[:] = 0.0
                self._last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _srv_start(self, _req, resp):
        self._running = True
        self._publish_solver_mode()
        resp.success = True
        resp.message = 'DLS servo started'
        self.get_logger().info('Servo STARTED')
        return resp

    def _srv_pause(self, _req, resp):
        self._running = False
        self._publish_solver_mode()
        resp.success = True
        resp.message = 'DLS servo paused'
        self.get_logger().info('Servo PAUSED')
        return resp

    def _solver_mode_cb(self, msg: String):
        """Handle GUI-driven solver switch.

        "servo"   → DLS servo active (real-time jogging).
        "planner" → DLS paused, MoveIt plan-and-execute / RViz markers work.

        Ignores an echo of our own published state so the pub/sub on the
        same topic doesn't bounce.
        """
        requested = msg.data.strip().lower()
        if requested == 'servo' and not self._running:
            self._running = True
            self.get_logger().info('Solver mode → SERVO (real-time jogging)')
            self._publish_solver_mode()
        elif requested == 'planner' and self._running:
            self._running = False
            self.get_logger().info('Solver mode → PLANNER (MoveIt plan-and-execute)')
            self._publish_solver_mode()

    def _publish_solver_mode(self):
        msg = String()
        msg.data = 'servo' if self._running else 'planner'
        self._solver_mode_pub.publish(msg)

    # ------------------------------------------------------------------
    # Main control loop  (~30 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self):
        if not self._running:
            return

        # snapshot state
        with self._lock:
            if not self._q_valid:
                return
            q          = self._q.copy()
            twist_raw  = self._twist_cmd.copy()
            twist_frame = self._twist_frame
            joint_cmd  = self._joint_cmd
            cmd_time   = self._last_cmd_time

        dt = 1.0 / self.publish_rate
        age = (self.get_clock().now() - cmd_time).nanoseconds * 1e-9

        # zero commands on timeout
        if age > self.cmd_timeout:
            twist_raw[:] = 0.0
            joint_cmd = None

        # compute joint displacement
        if joint_cmd is not None:
            dq = self._compute_joint_jog(q, joint_cmd, dt)
        else:
            dq = self._compute_cartesian(q, twist_raw, twist_frame, dt)

        # enforce joint limits
        q_new = np.clip(
            q + dq,
            self.kin.lower + self.jl_margin * 0.5,
            self.kin.upper - self.jl_margin * 0.5,
        )
        dq = q_new - q

        # collision check — reject motion if lookahead position collides
        if self._is_colliding(q_new, q):
            self._publish_status(STATUS_COLLISION)
            q_new = q
            dq = np.zeros(N_JOINTS)

        self._publish_trajectory(q_new, dq, dt)

    # ------------------------------------------------------------------
    # Joint-space jog  (no Jacobian, no singularity concern)
    # ------------------------------------------------------------------

    def _compute_joint_jog(self, q, cmd, dt):
        idx, vel = cmd
        dq = np.zeros(N_JOINTS)
        dq[idx] = vel * self.max_jvel * dt

        status = STATUS_OK
        for i in range(N_JOINTS):
            q_next = q[i] + dq[i]
            if q_next < self.kin.lower[i] + self.jl_margin or \
               q_next > self.kin.upper[i] - self.jl_margin:
                status = STATUS_JLIMIT

        self._publish_status(status)
        return dq

    # ------------------------------------------------------------------
    # Cartesian control via Damped Least Squares
    # ------------------------------------------------------------------

    def _compute_cartesian(self, q, twist_raw, twist_frame, dt):
        # EMA smoothing on the input twist
        self._filtered_tw = (self.alpha * twist_raw
                             + (1.0 - self.alpha) * self._filtered_tw)

        # scale unitless [-1, 1] commands to physical velocities
        dx = np.empty(6)
        dx[:3] = self._filtered_tw[:3] * self.max_lin     # m/s
        dx[3:] = self._filtered_tw[3:] * self.max_rot     # rad/s

        # --- if twist is in EE frame, rotate to base frame ---
        if twist_frame == self.ee_frame:
            T_ee, _, _ = self.kin.fk(q)
            R = T_ee[:3, :3]
            dx[:3] = R @ dx[:3]
            dx[3:] = R @ dx[3:]

        # --- Jacobian & condition number ---
        J = self.kin.jacobian(q)
        sv = np.linalg.svd(J, compute_uv=False)
        cond = sv[0] / sv[-1] if sv[-1] > 1e-12 else 1e12

        # --- adaptive damping ---
        if cond > self.cond_lower:
            ratio = min((cond - self.cond_lower)
                        / (self.cond_upper - self.cond_lower), 1.0)
            lam = self.damp_max * ratio
        else:
            lam = 0.0

        status = STATUS_DECEL if lam > 0.01 * self.damp_max else STATUS_OK

        # --- DLS solve:  dq = J^T (J J^T + lambda^2 I)^-1  dx ---
        JJT = J @ J.T
        try:
            dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), dx)
        except np.linalg.LinAlgError:
            dq = np.linalg.lstsq(J, dx, rcond=None)[0]

        # clamp max joint velocity
        peak = np.max(np.abs(dq))
        if peak > self.max_jvel:
            dq *= self.max_jvel / peak

        # smooth deceleration near joint limits
        for i in range(N_JOINTS):
            dist_lo = q[i] - self.kin.lower[i]
            dist_hi = self.kin.upper[i] - q[i]

            if dq[i] < 0 and dist_lo < self.jl_scale_zone:
                dq[i] *= max(dist_lo / self.jl_scale_zone, 0.0)
                if dist_lo < self.jl_margin:
                    status = STATUS_JLIMIT
            elif dq[i] > 0 and dist_hi < self.jl_scale_zone:
                dq[i] *= max(dist_hi / self.jl_scale_zone, 0.0)
                if dist_hi < self.jl_margin:
                    status = STATUS_JLIMIT

        self._publish_status(status)
        return dq * dt

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_trajectory(self, q_new, dq, dt):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = q_new.tolist()
        pt.velocities = (dq / dt if dt > 0 else np.zeros(N_JOINTS)).tolist()
        ns = int(dt * 1e9)
        pt.time_from_start = Duration(
            sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

        msg.points = [pt]
        self.traj_pub.publish(msg)

    def _publish_status(self, code):
        self._last_status = code
        msg = Int8()
        msg.data = code
        self.status_pub.publish(msg)


# -----------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------

def main():
    rclpy.init()
    node = DampedServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
