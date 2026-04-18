#!/usr/bin/env python3
import math
import time
import threading
import tkinter as tk
from array import array
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory, RobotTrajectory
from tf2_ros import TransformException, Buffer, TransformListener
from control_msgs.msg import JointTrajectoryControllerState

class ArmDiagnostics(Node):
    def __init__(self):
        super().__init__("arm_diagnostics")
        
        self.TEST_POSITIONS = [
            array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),      # Home position
            array('d', [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]),     # Small movement
            array('d', [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]),     # Medium movement
            array('d', [0.0, -1.57, 1.57, 0.0, 0.0, 0.0])    # Max movement
        ]
        
        self.JOINT_NAMES = [
            'joint1',
            'joint2',
            'joint3', 
            'joint4',
            'joint5',
            'joint6'
        ]
        
        self._setup_diagnostics()
        
        # Start UI
        threading.Thread(target=self._run_diagnostic_ui, daemon=True).start()

    def _setup_diagnostics(self):
        """Initialize all diagnostic components."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_state = JointState()
        self.controller_state = None
        self.create_subscription(
            JointState, "/joint_states", 
            self._joint_state_callback, 10)
        self.create_subscription(
            JointTrajectoryControllerState, "/jaguar_arm_controller/controller_state",
            self._controller_state_callback, 10)
        
        # Service clients
        self.cartesian_cli = self.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self.ik_cli = self.create_client(
            GetPositionIK, "/compute_ik")
        
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/jaguar_arm_controller/joint_trajectory", 10)
        self.traj_viz_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10)
        
        self.get_logger().info("Connecting to services...")
        while not self.cartesian_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Cartesian path service not available, waiting...")
        while not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("IK service not available, waiting...")

    def _joint_state_callback(self, msg):
        """Store current joint state."""
        self.joint_state = msg

    def _controller_state_callback(self, msg):
        """Store controller state."""
        self.controller_state = msg

    def _run_diagnostic_ui(self):
        """Create diagnostic UI."""
        self.root = tk.Tk()
        self.root.title("Arm Diagnostic Tool v4")
        
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        
        test_frame = tk.LabelFrame(self.root, text="Diagnostic Tests", padx=5, pady=5)
        test_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        
        basic_tests = [
            ("1. Verify Controller", self._test_controller),
            ("2. Check TF Frames", self._check_tf_frames),
            ("3. Test Joint Limits", self._test_joint_limits),
            ("4. Test Cartesian", self._test_cartesian)
        ]
        
        for i, (text, cmd) in enumerate(basic_tests):
            tk.Button(test_frame, text=text, command=cmd).grid(
                row=0, column=i, padx=5, pady=5, sticky="ew")
        
        adv_frame = tk.LabelFrame(self.root, text="Advanced Tests", padx=5, pady=5)
        adv_frame.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        
        adv_tests = [
            ("5. Test Joint 2", lambda: self._test_single_joint(1)),
            ("6. Test Joint 3", lambda: self._test_single_joint(2)),
            ("7. Full Diagnostic", self._full_diagnostic)
        ]
        
        for i, (text, cmd) in enumerate(adv_tests):
            tk.Button(adv_frame, text=text, command=cmd).grid(
                row=0, column=i, padx=5, pady=5, sticky="ew")
        
        config_frame = tk.LabelFrame(self.root, text="Configuration", padx=5, pady=5)
        config_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        
        labels = ["Controller Topic:", "Planning Group:", "EE Link:", "Base Frame:"]
        defaults = [
            "/jaguar_arm_controller/joint_trajectory",
            "arm_jaguar",
            "Link6",
            "base_link"
        ]
        
        for i, (label, default) in enumerate(zip(labels, defaults)):
            tk.Label(config_frame, text=label).grid(row=i, column=0, sticky="w")
            entry = tk.Entry(config_frame)
            entry.insert(0, default)
            entry.grid(row=i, column=1, sticky="ew")
            setattr(self, f"{label[:-1].lower().replace(' ', '_')}_entry", entry)
        
        self.status_var = tk.StringVar()
        self.status_var.set("Diagnostics ready\n")
        status = tk.Label(self.root, 
                         textvariable=self.status_var,
                         width=120,
                         height=25,
                         font=('Courier', 9),
                         justify='left',
                         relief='sunken',
                         bg='black',
                         fg='white')
        status.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        
        tk.Button(self.root, text="Clear Log", command=lambda: self.status_var.set("")).grid(
            row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        
        self._log("Diagnostic tool initialized")
        self.root.mainloop()

    def _log(self, message):
        """Log message with timestamp."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        current_text = self.status_var.get()
        if len(current_text) > 10000:  # Prevent memory issues
            current_text = current_text[:5000]
        self.status_var.set(f"[{timestamp}] {message}\n{current_text}")
        self.get_logger().info(message)

    def _send_joint_command(self, positions, duration=2.0):
        """Send joint position command."""
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(point)
        self.traj_pub.publish(traj)

    def _test_controller(self):
        """Test controller communication."""
        self._log("\n=== Testing Controller Communication ===")
        
        if not self.controller_state:
            self._log("ERROR: No controller state received!")
            self._log("Check if controller is running")
            return
            
        self._log(f"Controller state:\n"
                 f"- Running: {self.controller_state.actual.positions != []}\n"
                 f"- Error: {self._format_array(self.controller_state.error.positions)}\n"
                 f"- Desired: {self._format_array(self.controller_state.desired.positions)}\n"
                 f"- Actual: {self._format_array(self.controller_state.actual.positions)}")
        
        # Test small movement
        test_pos = array('d', [0.0, -0.1, 0.1, 0.0, 0.0, 0.0])
        self._log(f"\nSending test command: {self._format_array(test_pos)}")
        
        self._send_joint_command(test_pos)
        
        # Check if command was received
        start_time = time.time()
        while time.time() - start_time < 3.0:  # 3 second timeout
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.controller_state.desired.positions and 
                any(abs(d) > 0.01 for d in self.controller_state.desired.positions)):
                break
        
        if self.controller_state.desired.positions:
            self._log("SUCCESS: Controller received command")
            self._log(f"Desired: {self._format_array(self.controller_state.desired.positions)}")
            self._log(f"Actual: {self._format_array(self.controller_state.actual.positions)}")
        else:
            self._log("ERROR: Controller not responding to commands")

    def _check_tf_frames(self):
        """Verify TF frame connectivity."""
        self._log("\n=== Checking TF Frames ===")
        base_frame = self.base_frame_entry.get()
        ee_link = self.ee_link_entry.get()
        
        try:
            # Get latest available transform
            trans = self.tf_buffer.lookup_transform(
                base_frame, ee_link, rclpy.time.Time())
            
            self._log(f"TF SUCCESS: {base_frame} -> {ee_link}")
            self._log(f"Position: [x: {trans.transform.translation.x:.3f}, "
                     f"y: {trans.transform.translation.y:.3f}, "
                     f"z: {trans.transform.translation.z:.3f}]")
            self._log(f"Rotation: [x: {trans.transform.rotation.x:.3f}, "
                     f"y: {trans.transform.rotation.y:.3f}, "
                     f"z: {trans.transform.rotation.z:.3f}, "
                     f"w: {trans.transform.rotation.w:.3f}]")
                     
        except TransformException as ex:
            self._log(f"TF ERROR: {str(ex)}")
            self._log("Possible issues:")
            self._log("- Robot state publisher not running")
            self._log("- URDF frame names incorrect")
            self._log("- TF tree not properly connected")

    def _test_joint_limits(self):
        """Test joint movement through different positions."""
        self._log("\n=== Testing Joint Limits ===")
        
        for i, test_pos in enumerate(self.TEST_POSITIONS):
            self._log(f"\nTesting position {i}: {self._format_array(test_pos)}")
            
            self._send_joint_command(test_pos, duration=3.0)
            
            # Wait for movement
            start_time = time.time()
            while time.time() - start_time < 5.0:  # 5 second timeout
                rclpy.spin_once(self, timeout_sec=0.1)
                if hasattr(self.joint_state, 'position'):
                    current_pos = array('d', self.joint_state.position)
                    error = sum(abs(c - t) for c, t in zip(current_pos, test_pos))
                    if error < 0.05:  # 0.05 rad tolerance
                        break
            
            # Check result
            if not hasattr(self.joint_state, 'position'):
                self._log("ERROR: No joint state received!")
                continue
                
            current_pos = array('d', self.joint_state.position)
            error = sum(abs(c - t) for c, t in zip(current_pos, test_pos))
            max_error = max(abs(c - t) for c, t in zip(current_pos, test_pos))
            
            if error < 0.2:
                self._log(f"SUCCESS: Reached position {i}")
                self._log(f"Current: {self._format_array(current_pos)}")
                self._log(f"Error: {error:.3f} rad (max: {max_error:.3f} rad)")
            else:
                self._log(f"FAILED: Position {i}")
                self._log(f"Current: {self._format_array(current_pos)}")
                self._log(f"Error: {error:.3f} rad (max: {max_error:.3f} rad)")
                self._log("Possible issues:")
                self._log("- Joint limits exceeded")
                self._log("- Controller not running")
                self._log("- Hardware not responding")

    def _test_single_joint(self, joint_index):
        """Test individual joint movement."""
        self._log(f"\n=== Testing Joint {joint_index+1} ===")
        test_positions = [0.1, 0.3, 0.5, 0.8]  # Incremental positions
        
        for pos in test_positions:
            cmd = array('d', [0.0]*6)
            cmd[joint_index] = pos
            
            self._log(f"\nCommanding Joint {joint_index+1} to {pos:.2f} rad")
            self._send_joint_command(cmd, duration=2.0)
            
            # Wait and verify
            start_time = time.time()
            while time.time() - start_time < 5.0:  # 5 second timeout
                rclpy.spin_once(self, timeout_sec=0.1)
                if (hasattr(self.joint_state, 'position') and 
                   len(self.joint_state.position) > joint_index):
                    current = self.joint_state.position[joint_index]
                    if abs(current - pos) < 0.05:
                        break
                    
            # Log results
            if not hasattr(self.joint_state, 'position'):
                self._log("ERROR: No joint state received!")
                continue
                
            error = abs(self.joint_state.position[joint_index] - pos)
            status = "SUCCESS" if error < 0.1 else "FAILED"
            color = "green" if status == "SUCCESS" else "red"
            
            self._log(f"Joint {joint_index+1} result: {status}", color)
            self._log(f"  Commanded: {pos:.2f} rad")
            self._log(f"  Actual: {self.joint_state.position[joint_index]:.2f} rad")
            self._log(f"  Error: {error:.3f} rad")

    def _test_cartesian(self):
        """Test Cartesian planning."""
        self._log("\n=== Testing Cartesian Planning ===")
        
        # First verify we can get current EE pose
        try:
            base_frame = self.base_frame_entry.get()
            ee_link = self.ee_link_entry.get()
            
            trans = self.tf_buffer.lookup_transform(
                base_frame, ee_link, rclpy.time.Time())
            
            # Create planning request
            req = GetCartesianPath.Request()
            req.header.frame_id = base_frame
            req.link_name = ee_link
            req.group_name = self.planning_group_entry.get()
            
            # Set start state
            req.start_state.joint_state = self.joint_state
            req.max_step = 0.01
            req.jump_threshold = 0.0
            
            # Small movement in X direction
            pose = Pose()
            pose.position.x = trans.transform.translation.x + 0.02  # 2cm movement
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            req.waypoints.append(pose)
            
            self._log(f"Planning Cartesian path to:")
            self._log(f"  Position: [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
            self._log(f"  Orientation: [{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
                     f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f}]")
            
            future = self.cartesian_cli.call_async(req)
            future.add_done_callback(self._handle_cartesian_result)
            
        except Exception as e:
            self._log(f"Cartesian setup failed: {str(e)}")

    def _handle_cartesian_result(self, future):
        """Process Cartesian planning result."""
        try:
            res = future.result()
            
            if res.fraction > 0.9:
                self._log("Cartesian planning SUCCESS", "green")
                self._log(f"Planned path: {res.fraction:.0%}")
                
                # Visualize trajectory
                display_msg = DisplayTrajectory()
                display_msg.trajectory = [res.solution]
                self.traj_viz_pub.publish(display_msg)
                
                # Execute trajectory
                traj = res.solution.joint_trajectory
                traj.joint_names = self.JOINT_NAMES
                self.traj_pub.publish(traj)
                
            else:
                self._log("Cartesian planning FAILED", "red")
                self._log(f"Fraction planned: {res.fraction:.0%}")
                self._log(f"Error code: {res.error_code.val} ({self._get_moveit_error_name(res.error_code.val)})")
                self._log("Possible issues:")
                self._log("- Invalid start state")
                self._log("- Collision constraints")
                self._log("- Planning group misconfiguration")
                
                # Log the planned trajectory if any
                if hasattr(res.solution, 'joint_trajectory') and res.solution.joint_trajectory.points:
                    self._log("Planned trajectory (first point):")
                    first_point = res.solution.joint_trajectory.points[0]
                    self._log(f"  Positions: {self._format_array(first_point.positions)}")
                         
        except Exception as e:
            self._log(f"Cartesian service failed: {str(e)}", "red")

    def _full_diagnostic(self):
        """Run complete diagnostic sequence."""
        self._log("\n=== STARTING FULL DIAGNOSTIC ===")
        self._test_controller()
        time.sleep(1)
        self._check_tf_frames()
        time.sleep(1)
        self._test_joint_limits()
        time.sleep(1)
        self._test_cartesian()
        self._log("\n=== FULL DIAGNOSTIC COMPLETE ===")

    def _format_array(self, arr):
        """Format array for display."""
        if not arr:
            return "[]"
        return "[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(*arr)

    def _get_moveit_error_name(self, code):
        """Get MoveIt error code name."""
        error_names = {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -8: "PREEMPTED",
            -9: "START_STATE_IN_COLLISION",
            -10: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -11: "GOAL_IN_COLLISION",
            -12: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -13: "GOAL_CONSTRAINTS_VIOLATED",
            -14: "INVALID_GROUP_NAME",
            -15: "INVALID_GOAL_CONSTRAINTS",
            -16: "INVALID_ROBOT_STATE",
            -17: "INVALID_LINK_NAME",
            -18: "INVALID_OBJECT_NAME",
            -19: "FRAME_TRANSFORM_FAILURE",
            -20: "COLLISION_CHECKING_UNAVAILABLE",
            -21: "ROBOT_STATE_STALE",
            -22: "SENSOR_INFO_STALE",
            -23: "NO_IK_SOLUTION"
        }
        return error_names.get(code, f"UNKNOWN_ERROR_CODE_{code}")

    def _log(self, message, color=None):
        """Log message with optional color."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}"
        
        if color:
            # For terminal output
            color_codes = {
                "red": "\033[91m",
                "green": "\033[92m",
                "yellow": "\033[93m",
                "blue": "\033[94m",
                "reset": "\033[0m"
            }
            log_entry = f"{color_codes.get(color, '')}{log_entry}{color_codes['reset']}"
        
        # Update GUI
        current_text = self.status_var.get()
        if len(current_text) > 10000:  # Prevent memory issues
            current_text = current_text[:5000]
        self.status_var.set(f"{log_entry}\n{current_text}")
        
        # Also log to ROS
        if color == "red":
            self.get_logger().error(message)
        elif color == "yellow":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

def main():
    rclpy.init()
    node = ArmDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()