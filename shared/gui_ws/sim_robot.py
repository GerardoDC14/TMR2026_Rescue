#!/usr/bin/env python3
"""Simulates the robot's on-board ROS2 node for GUI testing.

Publishes:
  /config              (std_msgs/String, latched)  — available topics
  /sensor/magnetometer (geometry_msgs/Vector3)
  /sensor/gas          (std_msgs/Int32)
  /joint_states        (sensor_msgs/JointState)
  /camera/front        (sensor_msgs/Image, bgr8)   — from VideoCapture(0)
  /thermal/front       (std_msgs/Float64MultiArray) — fake 24×32 thermal
  /audio               (std_msgs/Int16MultiArray)   — mic at 16 kHz, when enabled

Subscribes:
  /audio_enable        (std_msgs/Bool)
  /estop               (std_msgs/Bool)
"""

import json, math, random, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, Empty, Int32, String, Float64MultiArray, Int16MultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState, Image

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False

try:
    import pyaudio
    _PYAUDIO = True
except ImportError:
    _PYAUDIO = False

# ── Config ───────────────────────────────────────────────────────────────────

CAMERA_TOPIC  = "/camera/front"
THERMAL_TOPIC = "/thermal/front"
CAMERA_ID     = 0
JOINT_NAMES   = ["base_joint", "shoulder", "elbow", "wrist"]
THERMAL_ROWS  = 24
THERMAL_COLS  = 32
AUDIO_RATE    = 16000
AUDIO_CHUNK   = 4000   # 250 ms

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class SimRobot(Node):
    def __init__(self):
        super().__init__("sim_robot")

        self._t = 0.0
        self._audio_active = False
        self._audio_stream = None
        self._audio_lock   = threading.Lock()

        # ── Publishers ───────────────────────────────────────────────────────

        self._pub_config     = self.create_publisher(String,             "/config",              LATCHED)
        self._pub_mag        = self.create_publisher(Vector3,            "/sensor/magnetometer", 10)
        self._pub_gas        = self.create_publisher(Int32,              "/sensor/gas",          10)
        self._pub_joints     = self.create_publisher(JointState,         "/joint_states",        10)
        self._pub_camera     = self.create_publisher(Image,              CAMERA_TOPIC,           10)
        self._pub_thermal    = self.create_publisher(Float64MultiArray,  THERMAL_TOPIC,          10)
        self._pub_audio      = self.create_publisher(Int16MultiArray,    "/audio",               10)
        self._pub_heartbeat  = self.create_publisher(Empty,              "/heartbeat",           10)

        # ── Subscribers ──────────────────────────────────────────────────────

        self.create_subscription(Bool, "/audio_enable", self._on_audio_enable, 10)
        self.create_subscription(Bool, "/estop",        self._on_estop,        10)

        # ── Camera ───────────────────────────────────────────────────────────

        self._cap = None
        if _CV2:
            self._cap = cv2.VideoCapture(CAMERA_ID)
            if not self._cap.isOpened():
                self.get_logger().warn(f"Camera {CAMERA_ID} unavailable — using colour bars")

        # ── PyAudio ──────────────────────────────────────────────────────────

        self._pa = None
        if _PYAUDIO:
            try:
                self._pa = pyaudio.PyAudio()
                self.get_logger().info("PyAudio ready — mic will stream when /audio_enable=true")
            except Exception as e:
                self.get_logger().warn(f"PyAudio init failed: {e}")

        # ── Timers ───────────────────────────────────────────────────────────

        #self.create_timer(1.0 / 30.0, self._fast_tick)   # 30 Hz — camera, joints
        #self.create_timer(0.1,        self._slow_tick)   # 10 Hz — mag, thermal
        #self.create_timer(0.5,        self._gas_tick)    #  2 Hz — gas
        #self.create_timer(1.0,        self._heartbeat_tick)  #  1 Hz — heartbeat

        self._publish_config()
        self.get_logger().info("sim_robot node started")

    # ── Subscriptions ────────────────────────────────────────────────────────

    def _on_audio_enable(self, msg: Bool):
        self._audio_active = msg.data
        self.get_logger().info(f"Audio {'ENABLED' if msg.data else 'disabled'}")
        self._update_audio_stream()

    def _on_estop(self, msg: Bool):
        self.get_logger().warn(f"E-STOP {'ACTIVE' if msg.data else 'cleared'}")

    # ── Config (latched, published once) ─────────────────────────────────────

    def _publish_config(self):
        msg = String()
        msg.data = json.dumps({
            "camera_topics":  [CAMERA_TOPIC],
            "thermal_topics": [THERMAL_TOPIC],
        })
        self._pub_config.publish(msg)

    # ── Timers ───────────────────────────────────────────────────────────────

    def _fast_tick(self):
        self._t += 1.0 / 30.0
        self._publish_camera()
        self._publish_joints()

    def _slow_tick(self):
        self._publish_magnetometer()
        self._publish_thermal()

    def _gas_tick(self):
        msg = Int32(data=int(400 + random.gauss(0, 20)))
        self._pub_gas.publish(msg)

    def _heartbeat_tick(self):
        self._pub_heartbeat.publish(Empty())

    # ── Sensor publishers ─────────────────────────────────────────────────────

    def _publish_magnetometer(self):
        msg = Vector3(
            x=25.0 + random.gauss(0, 0.3),
            y=-10.0 + random.gauss(0, 0.3),
            z=42.0 + math.sin(self._t * 0.1) * 2.0,
        )
        self._pub_mag.publish(msg)

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = [math.sin(self._t * 0.4 + i * 1.1) * 0.7
                        for i in range(len(JOINT_NAMES))]
        msg.velocity = [0.0] * len(JOINT_NAMES)
        self._pub_joints.publish(msg)

    def _publish_camera(self):
        if self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if not ret:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
        else:
            # Colour-bar fallback (no OpenCV needed for basic testing)
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            for i, c in enumerate([(255,0,0),(0,255,0),(0,0,255),
                                    (255,255,0),(0,255,255),(255,0,255)]):
                frame[:, i*107:(i+1)*107] = c

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height   = frame.shape[0]
        msg.width    = frame.shape[1]
        msg.encoding = "bgr8"
        msg.step     = msg.width * 3
        msg.data     = frame.tobytes()
        self._pub_camera.publish(msg)

    def _publish_thermal(self):
        # Simulate a warm centre blob drifting around
        cx = THERMAL_COLS / 2 + math.sin(self._t * 0.3) * 6
        cy = THERMAL_ROWS / 2 + math.cos(self._t * 0.2) * 4
        data = []
        for r in range(THERMAL_ROWS):
            for c in range(THERMAL_COLS):
                d2 = (r - cy) ** 2 + (c - cx) ** 2
                data.append(28.0 + 12.0 * math.exp(-d2 / 30.0) + random.gauss(0, 0.2))

        dim_r = MultiArrayDimension(label="height", size=THERMAL_ROWS, stride=THERMAL_ROWS * THERMAL_COLS)
        dim_c = MultiArrayDimension(label="width",  size=THERMAL_COLS, stride=THERMAL_COLS)
        msg = Float64MultiArray(layout=MultiArrayLayout(dim=[dim_r, dim_c]), data=data)
        self._pub_thermal.publish(msg)

    # ── Audio ─────────────────────────────────────────────────────────────────

    def _update_audio_stream(self):
        if not self._pa:
            return
        with self._audio_lock:
            if self._audio_active and self._audio_stream is None:
                self._audio_stream = self._pa.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=AUDIO_RATE,
                    input=True,
                    frames_per_buffer=AUDIO_CHUNK,
                    stream_callback=self._audio_callback,
                )
                self._audio_stream.start_stream()
            elif not self._audio_active and self._audio_stream is not None:
                self._audio_stream.stop_stream()
                self._audio_stream.close()
                self._audio_stream = None

    def _audio_callback(self, in_data, frame_count, time_info, status):
        samples = np.frombuffer(in_data, dtype=np.int16)
        msg = Int16MultiArray(data=samples.tolist())
        self._pub_audio.publish(msg)
        return (None, pyaudio.paContinue)

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        with self._audio_lock:
            if self._audio_stream:
                self._audio_stream.stop_stream()
                self._audio_stream.close()
        if self._pa:
            self._pa.terminate()
        if self._cap:
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
