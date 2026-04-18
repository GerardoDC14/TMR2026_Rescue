#!/usr/bin/env python3
"""
gst_sender — esp32_bridge package, runs on the robot (Jetson Orin Nano).

Discovers /dev/video* capture devices (or uses an explicit list), spawns one
GStreamer H.264/RTP UDP sender per camera, and publishes a latched /config
so the laptop's gst_bridge knows which topics to create.

Prefers the Jetson hardware encoder (nvv4l2h264enc); falls back to x264enc
when running on a dev machine without it.

Default capture format is MJPG (image/jpeg) because raw YUY2 over USB 2.0
exhausts bandwidth with more than ~2 concurrent 640x480@30 cameras. Pass
`pixel_format:=yuyv` to force uncompressed.
"""

import glob
import json
import shutil
import signal
import subprocess
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

BASE_PORT = 5000


def _discover_cameras(logger) -> list[str]:
    """Return /dev/video* nodes that actually deliver frames.

    Logitech cameras expose multiple /dev/videoN nodes per physical device
    (raw + metadata). We keep only ones where V4L2 returns a frame.
    """
    found = []
    rejected = []
    for dev in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            rejected.append((dev, "could not open"))
            continue
        ok, _ = cap.read()
        cap.release()
        if ok:
            found.append(dev)
        else:
            rejected.append((dev, "opened but no frame"))

    logger.info(f"Camera discovery: kept {found}")
    for dev, why in rejected:
        logger.info(f"  skipped {dev}: {why}")
    return found


def _have_element(name: str) -> bool:
    return subprocess.run(
        ["gst-inspect-1.0", name],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def _list_v4l2_formats(device: str) -> str:
    """Return a human-readable summary of the device's supported formats."""
    if not shutil.which("v4l2-ctl"):
        return "(v4l2-ctl not installed)"
    try:
        out = subprocess.run(
            ["v4l2-ctl", f"--device={device}", "--list-formats-ext"],
            capture_output=True, text=True, timeout=2,
        )
        return out.stdout.strip() or out.stderr.strip() or "(empty)"
    except Exception as e:
        return f"(query failed: {e})"


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
                    encoder: str, fec_percentage: int,
                    pixel_format: str) -> list[str]:
    fec = (f"rtpulpfecenc percentage={fec_percentage} pt=122 !"
           if fec_percentage > 0 else "")

    pf = pixel_format.lower()
    if pf == "mjpg":
        # MJPG → JPEG decode → raw → encoder. ~10× less USB bandwidth.
        capture_stage = (
            f"image/jpeg,width={width},height={height},framerate={framerate}/1 ! "
            f"jpegdec ! videoconvert ! "
        )
    elif pf == "yuyv":
        capture_stage = (
            f"video/x-raw,format=YUY2,width={width},height={height},"
            f"framerate={framerate}/1 ! videoconvert ! "
        )
    else:  # "auto" — let GStreamer negotiate
        capture_stage = (
            f"video/x-raw,width={width},height={height},"
            f"framerate={framerate}/1 ! videoconvert ! "
        )

    pipeline = (
        f"v4l2src device={device} ! "
        f"{capture_stage}{encoder} ! h264parse ! "
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
        self.declare_parameter("pixel_format", "mjpg")  # mjpg | yuyv | auto
        self.declare_parameter("log_gst_stderr", True)

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
            devices = _discover_cameras(self.get_logger())
            if not devices:
                self.get_logger().error("No working /dev/video* devices found")
                raise SystemExit(1)
        else:
            devices = cameras_param
            self.get_logger().info(f"Using cameras from parameter: {devices}")

        pixel_format = self.get_parameter("pixel_format").value
        self.get_logger().info(f"Pixel format: {pixel_format}")

        # Log each device's actual supported formats — invaluable when MJPG
        # negotiation fails (e.g. a webcam that only offers YUY2).
        for dev in devices:
            self.get_logger().info(
                f"── {dev} supported formats ──\n{_list_v4l2_formats(dev)}")

        encoder = _pick_encoder(
            self.get_parameter("bitrate_kbps").value,
            self.get_parameter("keyframe_period").value,
        )
        self.get_logger().info(f"Encoder: {encoder.split()[0]}")

        self._procs: list[subprocess.Popen] = []
        self._stderr_threads: list[threading.Thread] = []
        self._stop_readers = threading.Event()
        camera_topics: list[str] = []

        log_stderr = self.get_parameter("log_gst_stderr").value

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
                pixel_format,
            )
            self.get_logger().info(f"Launching: {' '.join(cmd)}")
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE if log_stderr else subprocess.DEVNULL,
                text=True,
                bufsize=1,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            )
            self._procs.append(proc)
            camera_topics.append(topic)
            self.get_logger().info(
                f"  {device} → {laptop_ip}:{port} → {topic} (pid {proc.pid})")

            if log_stderr and proc.stderr is not None:
                t = threading.Thread(
                    target=self._drain_stderr,
                    args=(proc, device),
                    daemon=True,
                )
                t.start()
                self._stderr_threads.append(t)

        # Watchdog: log when any pipeline exits unexpectedly
        self._watchdog = threading.Thread(
            target=self._watch_procs,
            args=(list(zip(devices, self._procs)),),
            daemon=True,
        )
        self._watchdog.start()

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

    def _drain_stderr(self, proc: subprocess.Popen, device: str):
        """Forward every gst-launch stderr line to the ROS logger."""
        try:
            for line in proc.stderr:  # blocks until EOF
                if self._stop_readers.is_set():
                    break
                line = line.rstrip()
                if not line:
                    continue
                low = line.lower()
                if ("error" in low or "warn" in low or "could not" in low
                        or "failed" in low):
                    self.get_logger().warn(f"[gst {device}] {line}")
                else:
                    self.get_logger().info(f"[gst {device}] {line}")
        except Exception as e:
            self.get_logger().warn(
                f"[gst {device}] stderr reader crashed: {e}")

    def _watch_procs(self, pairs):
        """Log any pipeline that exits on its own."""
        reported = set()
        while not self._stop_readers.is_set():
            for device, proc in pairs:
                rc = proc.poll()
                if rc is not None and device not in reported:
                    reported.add(device)
                    self.get_logger().error(
                        f"[gst {device}] pipeline EXITED with code {rc}. "
                        f"Check stderr above for the reason "
                        f"(common: USB bandwidth, device busy, format negotiation).")
            time.sleep(0.5)

    def destroy_node(self):
        self._stop_readers.set()
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
