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


class MainWindow(QMainWindow):

    def __init__(self, demo_mode=True):

        super().__init__()

        self.demo_mode = demo_mode

        self.setWindowTitle("RoboCup Rescue GUI")
        self.resize(1400, 900)

        # estado de cámaras
        self.expanded_camera = None
        self.gui_ready = False

        main_widget = QWidget()
        main_layout = QVBoxLayout()

        # -----------------------
        # TOP SECTION
        # -----------------------

        top_layout = QHBoxLayout()

        self.rviz_placeholder = QLabel("RViz View")
        self.rviz_placeholder.setStyleSheet(
            "background-color: black; color:white"
        )

        self.arm_control = ArmControlWidget()
        self.sensors_panel = QLabel("Sensors Panel")

        top_layout.addWidget(self.rviz_placeholder, 4)
        top_layout.addWidget(self.arm_control, 2)
        top_layout.addWidget(self.sensors_panel, 1)

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

        # conectar click
        self.front_cam.clicked.connect(self.expand_camera)
        self.arm_cam.clicked.connect(self.expand_camera)
        self.rear_cam.clicked.connect(self.expand_camera)

        # -----------------------
        # TELEMETRY
        # -----------------------

        self.telemetry = TelemetryWidget()

        main_layout.addLayout(top_layout)
        main_layout.addLayout(self.camera_layout)
        main_layout.addWidget(self.telemetry)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # activar GUI después de iniciar
        QTimer.singleShot(100, self.enable_gui)

        # -----------------------
        # DEMO OR ROS MODE
        # -----------------------

        if self.demo_mode:
            self.start_demo_mode()
        else:
            self.start_ros_mode()

    # ------------------------------------------------
    # GUI READY
    # ------------------------------------------------

    def enable_gui(self):
        self.gui_ready = True

    # ------------------------------------------------
    # DEMO MODE
    # ------------------------------------------------

    def start_demo_mode(self):

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_fake_data)
        self.timer.start(50)

    def update_fake_data(self):

        frame = np.zeros((480,640,3), dtype=np.uint8)

        cv2.putText(
            frame,"Front Camera",(180,240),
            cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2
        )

        self.front_cam.update_frame(frame)

        frame2 = np.zeros((480,640,3), dtype=np.uint8)

        cv2.putText(
            frame2,"Arm Camera",(180,240),
            cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2
        )

        self.arm_cam.update_frame(frame2)

        frame3 = np.zeros((480,640,3), dtype=np.uint8)

        cv2.putText(
            frame3,"Rear Camera",(180,240),
            cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2
        )

        self.rear_cam.update_frame(frame3)

        joints = np.random.uniform(-3.14,3.14,6)

        self.telemetry.update_joints(joints)

    # ------------------------------------------------
    # CAMERA EXPANSION
    # ------------------------------------------------

    def expand_camera(self, camera):

        if not self.gui_ready:
            return

        if self.expanded_camera is not None:
            return

        self.expanded_camera = camera

        for cam in [self.front_cam, self.arm_cam, self.rear_cam]:

            if cam != camera:
                cam.hide()

        camera.setMinimumHeight(600)

    def restore_cameras(self):

        if self.expanded_camera is None:
            return

        for cam in [self.front_cam, self.arm_cam, self.rear_cam]:
            cam.show()

        self.expanded_camera.setMinimumHeight(200)

        self.expanded_camera = None

    def keyPressEvent(self, event):

        if event.key() == Qt.Key.Key_Escape:
            self.restore_cameras()

    # ------------------------------------------------
    # ROS MODE
    # ------------------------------------------------

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