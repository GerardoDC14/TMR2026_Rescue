#!/usr/bin/env python3
"""
gst_sender — esp32_bridge package, runs on the robot (Jetson Orin Nano).

Discovers /dev/video* capture devices (or uses an explicit list), spawns one
GStreamer H.264/RTP UDP sender per camera, and publishes a latched /config
so the laptop's gst_bridge knows which topics to create.

Prefers the Jetson hardware encoder (nvv4l2h264enc); falls back to x264enc
when running on a dev machine without it.
"""

import glob
import json
import shutil
import signal
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

BASE_PORT = 5000


def _discover_cameras() -> list[str]:
    found = []
    for dev in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        ok, _ = cap.read()
        cap.release()
        if ok:
            found.append(dev)
    return found


def _have_element(name: str) -> bool:
    return subprocess.run(
        ["gst-inspect-1.0", name],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def _pick_encoder(bitrate_kbps: int, keyframe_period: int) -> str:
    if _have_element("nvv4l2h264enc"):
        return (
            f"nvv4l2h264enc bitrate={bitrate_kbps * 1000} "
            f"control-rate=1 profile=0 iframeinterval={keyframe_period} "
            f"insert-sps-pps=true maxperf-enable=true"
        )
    return (
        f"x264enc tune=zerolatency speed-preset=ultrafast "
        f"bitrate={bitrate_kbps} key-int-max={keyframe_period}"
    )


def _build_pipeline(device: str, port: int, laptop_ip: str,
                    width: int, height: int, framerate: int,
                    encoder: str, fec_percentage: int) -> list[str]:
    fec = (f"rtpulpfecenc percentage={fec_percentage} pt=122 !"
           if fec_percentage > 0 else "")
    pipeline = (
        f"v4l2src device={device} ! "
        f"video/x-raw,width={width},height={height},framerate={framerate}/1 ! "
        f"videoconvert ! {encoder} ! h264parse ! "
        f"rtph264pay config-interval=1 pt=96 ! {fec} "
        f"udpsink host={laptop_ip} port={port} sync=false async=false"
    )
    return ["gst-launch-1.0", "-e"] + pipeline.split()


class GstSenderNode(Node):
    def __init__(self):
        super().__init__("gst_sender")

        self.declare_parameter("laptop_ip", "")
        self.declare_parameter("cameras", ["auto"])
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("bitrate_kbps", 2000)
        self.declare_parameter("keyframe_period", 30)
        self.declare_parameter("fec_percentage", 25)

        laptop_ip = self.get_parameter("laptop_ip").value
        if not laptop_ip:
            self.get_logger().error("laptop_ip parameter is required")
            raise SystemExit(1)

        if not shutil.which("gst-launch-1.0"):
            self.get_logger().error(
                "gst-launch-1.0 not found. Install gstreamer1.0-tools.")
            raise SystemExit(1)

        cameras_param = list(self.get_parameter("cameras").value)
        if cameras_param == ["auto"] or not cameras_param:
            devices = _discover_cameras()
            if not devices:
                self.get_logger().error("No working /dev/video* devices found")
                raise SystemExit(1)
            self.get_logger().info(f"Auto-discovered cameras: {devices}")
        else:
            devices = cameras_param

        encoder = _pick_encoder(
            self.get_parameter("bitrate_kbps").value,
            self.get_parameter("keyframe_period").value,
        )
        self.get_logger().info(f"Encoder: {encoder.split()[0]}")

        self._procs: list[subprocess.Popen] = []
        camera_topics: list[str] = []

        for i, device in enumerate(devices):
            port = BASE_PORT + i
            topic = f"/camera/{device.rsplit('/', 1)[-1]}"
            cmd = _build_pipeline(
                device, port, laptop_ip,
                self.get_parameter("width").value,
                self.get_parameter("height").value,
                self.get_parameter("framerate").value,
                encoder,
                self.get_parameter("fec_percentage").value,
            )
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            )
            self._procs.append(proc)
            camera_topics.append(topic)
            self.get_logger().info(
                f"  {device} → {laptop_ip}:{port} → {topic} (pid {proc.pid})")

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_config = self.create_publisher(String, "/config", latched_qos)
        msg = String()
        msg.data = json.dumps({"camera_topics": camera_topics})
        self._pub_config.publish(msg)
        self.get_logger().info(f"Published /config: {msg.data}")

    def destroy_node(self):
        for proc in self._procs:
            if proc.poll() is None:
                proc.terminate()
        deadline = time.time() + 2.0
        for proc in self._procs:
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GstSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
