import threading
import cv2
import numpy as np

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel
)

from .camera_widget import CameraWidget
from .armcontrol_widget import ArmControlWidget
from .telemetry_widget import TelemetryWidget
from .operator_panel_widget import OperatorPanelWidget


class MainWindow(QMainWindow):

    def __init__(self, demo_mode=True):

        super().__init__()

        self.demo_mode = demo_mode
        self.gui_ready = False

        self.setWindowTitle("RoboCup Rescue GUI")
        self.resize(1500, 900)

        main_widget = QWidget()
        self.main_layout = QVBoxLayout()

        # -----------------------
        # TOP PANEL
        # -----------------------

        top_layout = QHBoxLayout()

        self.arm_cam_big = CameraWidget("Arm Camera")

        self.arm_control = ArmControlWidget()

        self.operator_panel = OperatorPanelWidget()

        top_layout.addWidget(self.arm_cam_big, 3)
        top_layout.addWidget(self.arm_control, 1)
        top_layout.addWidget(self.operator_panel, 1)

        # -----------------------
        # RVIZ
        # -----------------------

        self.rviz_placeholder = QLabel("RViz View")
        self.rviz_placeholder.setStyleSheet("background-color:black")

        # -----------------------
        # CAMERAS
        # -----------------------

        self.camera_layout = QHBoxLayout()

        self.front_cam = CameraWidget("Front Camera")
        self.arm_cam = CameraWidget("Arm Camera")
        self.rear_cam = CameraWidget("Rear Camera")

        self.camera_layout.addWidget(self.front_cam)
        self.camera_layout.addWidget(self.arm_cam)
        self.camera_layout.addWidget(self.rear_cam)

        # -----------------------
        # TELEMETRY
        # -----------------------

        self.telemetry = TelemetryWidget()

        # -----------------------
        # BUILD LAYOUT
        # -----------------------

        self.main_layout.addLayout(top_layout)
        self.main_layout.addWidget(self.rviz_placeholder, 3)
        self.main_layout.addLayout(self.camera_layout)
        self.main_layout.addWidget(self.telemetry)

        main_widget.setLayout(self.main_layout)
        self.setCentralWidget(main_widget)

        QTimer.singleShot(200, self.enable_gui)

        if self.demo_mode:
            self.start_demo_mode()
        else:
            self.start_ros_mode()

    def enable_gui(self):
        self.gui_ready = True

    # -----------------------
    # DEMO MODE
    # -----------------------

    def start_demo_mode(self):

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_fake_data)
        self.timer.start(50)

    def update_fake_data(self):

        frame = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(frame,"Front Camera",(180,240),
                    cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        self.front_cam.update_frame(frame)

        frame2 = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(frame2,"Arm Camera",(180,240),
                    cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
        self.arm_cam.update_frame(frame2)
        self.arm_cam_big.update_frame(frame2)

        frame3 = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(frame3,"Rear Camera",(180,240),
                    cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        self.rear_cam.update_frame(frame3)

        joints = np.random.uniform(-3.14,3.14,6)
        self.telemetry.update_joints(joints)

    # -----------------------
    # ROS MODE
    # -----------------------

    def start_ros_mode(self):

        import rclpy
        from ros.camera_subscriber import CameraSubscriber
        from ros.joint_state_subscriber import JointStateSubscriber

        rclpy.init()

        self.front_cam_node = CameraSubscriber(
            "/camera_front/image_raw",
            self.front_cam.update_frame
        )

        self.arm_cam_node = CameraSubscriber(
            "/camera_arm/image_raw",
            self.arm_cam.update_frame
        )

        self.rear_cam_node = CameraSubscriber(
            "/camera_rear/image_raw",
            self.rear_cam.update_frame
        )

        self.joint_node = JointStateSubscriber(
            self.telemetry.update_joints
        )

        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()

    def spin_ros(self):

        import rclpy

        while rclpy.ok():

            rclpy.spin_once(self.front_cam_node, timeout_sec=0.01)
            rclpy.spin_once(self.arm_cam_node, timeout_sec=0.01)
            rclpy.spin_once(self.rear_cam_node, timeout_sec=0.01)
            rclpy.spin_once(self.joint_node, timeout_sec=0.01)