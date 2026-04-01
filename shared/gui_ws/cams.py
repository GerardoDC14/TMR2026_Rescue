#!/usr/bin/env python3
"""
gst_to_ros2.py — Run on the LAPTOP

Receives H.264/RTP video streams from the robot via GStreamer,
decodes them, and publishes as ROS2 sensor_msgs/Image topics.

All ROS2 publishing is local (loopback) so WiFi quality doesn't matter here.

Requirements:
    sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-libav
    pip install opencv-python numpy

    OpenCV MUST be built with GStreamer support. Check with:
        python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer

    If GStreamer says NO, you need to either:
      - Install opencv from apt (sudo apt install python3-opencv) which usually has it, or
      - Build from source with -D WITH_GSTREAMER=ON

Usage:
    ros2 run your_package gst_to_ros2  (if installed as a ROS2 package)
    OR
    python3 gst_to_ros2.py             (standalone)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import json
import threading
import numpy as np


# ─── Configuration ─────────────────────────────────────────────
# Each entry: (udp_port, ros2_topic_name)
# Must match the ports in start_robot.sh
CAMERA_STREAMS = [
    (5000, "/camera/front"),
    # (5001, "/camera/rear"),
]

FEC_ENABLED = True  # Must match start_robot.sh setting
# ───────────────────────────────────────────────────────────────


def build_gst_pipeline(port: int) -> str:
    """Build the GStreamer receive/decode pipeline string for OpenCV."""
    fec_element = "rtpulpfecdec pt=122 ! rtpstorage size-time=220000000 !" if FEC_ENABLED else ""

    # rtpjitterbuffer adds a small buffer to smooth out packet reordering
    # latency=20 means 20ms buffer — tune this (lower = less latency, more glitches)
    pipeline = (
        f"udpsrc port={port} "
        f"caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
        f"rtpjitterbuffer latency=20 ! "
        f"{fec_element} "
        f"rtph264depay ! "
        f"h264parse ! "
        f"avdec_h264 ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )
    return pipeline


class GstToRos2Node(Node):
    def __init__(self):
        super().__init__("gst_to_ros2_bridge")
        self.bridge = CvBridge()

        # Use best-effort QoS — matches what most image subscribers expect
        # and avoids reliability overhead on local transport
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.threads = []

        for port, topic in CAMERA_STREAMS:
            publisher = self.create_publisher(Image, topic, qos)
            self.get_logger().info(f"Listening on UDP port {port} -> {topic}")

            t = threading.Thread(
                target=self._stream_loop,
                args=(port, topic, publisher),
                daemon=True,
            )
            t.start()
            self.threads.append(t)

        self.get_logger().info(
            f"Bridge running with {len(CAMERA_STREAMS)} stream(s). "
            "Waiting for data from robot..."
        )

        # Publish /config so the GUI knows which camera topics exist
        # Matches sim_robot's latched config message
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_config = self.create_publisher(String, "/config", latched_qos)

        config_msg = String()
        config_msg.data = json.dumps({
            "camera_topics":  [topic for _, topic in CAMERA_STREAMS],
            "thermal_topics": ["/sensors/thermal"],
        })
        self._pub_config.publish(config_msg)
        self.get_logger().info(f"Published /config: {config_msg.data}")

    def _stream_loop(self, port: int, topic: str, publisher):
        """Capture loop for a single GStreamer stream."""
        pipeline = build_gst_pipeline(port)
        self.get_logger().debug(f"GStreamer pipeline [{topic}]: {pipeline}")

        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.get_logger().error(
                f"Failed to open GStreamer pipeline for {topic} (port {port}). "
                "Check that OpenCV was built with GStreamer support: "
                "python3 -c \"import cv2; print(cv2.getBuildInformation())\" | grep GStreamer"
            )
            return

        self.get_logger().info(f"Stream active: {topic}")

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = topic.split("/")[2] if topic.count("/") >= 2 else "camera"
            publisher.publish(msg)

        cap.release()


def main(args=None):
    # Quick check for GStreamer support
    build_info = cv2.getBuildInformation()
    if "GStreamer" in build_info:
        gst_lines = [l for l in build_info.split("\n") if "GStreamer" in l]
        for l in gst_lines:
            if "NO" in l.upper():
                print("=" * 60)
                print("WARNING: OpenCV does NOT have GStreamer support!")
                print("Install with: sudo apt install python3-opencv")
                print("Or rebuild OpenCV with -D WITH_GSTREAMER=ON")
                print("=" * 60)
    
    rclpy.init(args=args)
    node = GstToRos2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()