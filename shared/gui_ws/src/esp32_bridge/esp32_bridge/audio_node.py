"""
audio_node.py
ROS2 node that captures audio from USB microphone and publishes it to /audio topic.
Subscribes to /audio_enable to control recording state.

Auto-detects the Logitech C920 Pro webcam microphone by default. Pass
`device_index` to force a specific PortAudio input index.
"""

import math
import struct
import threading
import time

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int16MultiArray


# Substrings that identify the Logitech C920 on Linux (order = preference)
C920_NAME_HINTS = (
    'C920',            # "HD Pro Webcam C920"
    'HD Pro Webcam',
    'Logitech Webcam',
    'USB Audio',       # generic fallback — many USB webcams appear as this
    'Webcam',
)


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('device_index', -1)  # -1 = auto-detect C920
        self.declare_parameter('device_name_hint', '')  # optional override

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.device_index = self.get_parameter('device_index').get_parameter_value().integer_value
        self.name_hint = self.get_parameter('device_name_hint').get_parameter_value().string_value

        self.pa = pyaudio.PyAudio()
        self.stream = None
        self.recording = False
        self.recording_lock = threading.Lock()

        # Diagnostic counters
        self._chunks_published = 0
        self._last_log_time = time.monotonic()
        self._last_peak = 0
        self._last_rms_sq_sum = 0.0
        self._last_sample_count = 0
        self._recording_started_at = None
        self._warned_no_samples = False

        # QoS for best-effort audio streaming
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.audio_pub = self.create_publisher(Int16MultiArray, '/audio', qos)
        self.audio_enable_sub = self.create_subscription(
            Bool, '/audio_enable', self._on_audio_enable, 10)

        # Enumerate and pick device
        self._log_all_input_devices()
        chosen_index, chosen_info = self._resolve_input_device()

        if chosen_index is None:
            self.get_logger().error(
                'No usable input device found. Audio capture disabled. '
                'Plug in the C920 and restart, or set `device_index` explicitly.')
            return

        self.device_index = chosen_index
        self.get_logger().info(
            f'Selected input device [{chosen_index}] "{chosen_info["name"]}" '
            f'(host_api={chosen_info["hostApi"]}, '
            f'max_in_ch={int(chosen_info["maxInputChannels"])}, '
            f'default_rate={int(chosen_info["defaultSampleRate"])} Hz)')

        # Verify the requested sample rate is supported, warn otherwise
        try:
            supported = self.pa.is_format_supported(
                rate=self.sample_rate,
                input_device=chosen_index,
                input_channels=1,
                input_format=pyaudio.paInt16,
            )
            self.get_logger().info(
                f'Device supports {self.sample_rate}Hz mono int16: {supported}')
        except ValueError as e:
            self.get_logger().warn(
                f'Device does NOT support {self.sample_rate}Hz mono int16: {e}. '
                f'Falling back to device default '
                f'{int(chosen_info["defaultSampleRate"])} Hz.')
            self.sample_rate = int(chosen_info['defaultSampleRate'])

        # Open stream
        try:
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=chosen_index,
                frames_per_buffer=self.chunk_size,
            )
            self.get_logger().info(
                f'Audio stream OPEN: rate={self.sample_rate}Hz, '
                f'chunk={self.chunk_size} ({self.chunk_size / self.sample_rate * 1000:.1f}ms), '
                f'device_index={chosen_index}')
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            return

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(
            'Audio capture node ready. Publish True on /audio_enable to start streaming.')

    # ── Device discovery ──────────────────────────────────────────────────────
    def _log_all_input_devices(self):
        count = self.pa.get_device_count()
        self.get_logger().info(f'PortAudio reports {count} device(s):')
        for i in range(count):
            try:
                info = self.pa.get_device_info_by_index(i)
            except Exception as e:
                self.get_logger().warn(f'  [{i}] <info error: {e}>')
                continue
            max_in = int(info.get('maxInputChannels', 0))
            if max_in <= 0:
                continue  # skip output-only devices in the listing
            self.get_logger().info(
                f'  [{i}] name="{info["name"]}" '
                f'host_api={info["hostApi"]} '
                f'in_ch={max_in} '
                f'default_rate={int(info["defaultSampleRate"])} Hz')

    def _resolve_input_device(self):
        """Return (index, info_dict) of the chosen input device, or (None, None)."""
        # 1. Explicit index wins
        if self.device_index >= 0:
            try:
                info = self.pa.get_device_info_by_index(self.device_index)
                if int(info.get('maxInputChannels', 0)) > 0:
                    self.get_logger().info(
                        f'Using explicit device_index={self.device_index}')
                    return self.device_index, info
                self.get_logger().error(
                    f'device_index={self.device_index} has no input channels '
                    f'(name="{info["name"]}")')
                return None, None
            except Exception as e:
                self.get_logger().error(
                    f'device_index={self.device_index} invalid: {e}')
                return None, None

        # 2. Custom name hint
        hints = [self.name_hint] if self.name_hint else list(C920_NAME_HINTS)

        count = self.pa.get_device_count()
        for hint in hints:
            for i in range(count):
                try:
                    info = self.pa.get_device_info_by_index(i)
                except Exception:
                    continue
                if int(info.get('maxInputChannels', 0)) <= 0:
                    continue
                if hint.lower() in info['name'].lower():
                    self.get_logger().info(
                        f'Matched input device by hint "{hint}": '
                        f'[{i}] "{info["name"]}"')
                    return i, info

        # 3. Fall back to default input
        try:
            info = self.pa.get_default_input_device_info()
            self.get_logger().warn(
                f'No C920 found. Falling back to default input: '
                f'[{info["index"]}] "{info["name"]}"')
            return int(info['index']), info
        except Exception as e:
            self.get_logger().error(f'No default input device: {e}')
            return None, None

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _on_audio_enable(self, msg: Bool):
        with self.recording_lock:
            was = self.recording
            self.recording = msg.data
            if msg.data and not was:
                self._recording_started_at = time.monotonic()
                self._warned_no_samples = False
                self._chunks_published = 0
            elif not msg.data and was:
                self._recording_started_at = None
        state = 'ON' if msg.data else 'OFF'
        self.get_logger().info(f'/audio_enable → {state}')

    def _capture_loop(self):
        while rclpy.ok():
            with self.recording_lock:
                is_recording = self.recording

            if not is_recording or not self.stream:
                time.sleep(0.01)
                continue

            try:
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                samples = struct.unpack(f'<{self.chunk_size}h', data)

                msg = Int16MultiArray()
                msg.data = list(samples)
                self.audio_pub.publish(msg)

                self._chunks_published += 1

                # accumulate stats for periodic log
                peak = max(abs(s) for s in samples)
                if peak > self._last_peak:
                    self._last_peak = peak
                self._last_rms_sq_sum += sum(s * s for s in samples)
                self._last_sample_count += len(samples)

                self._maybe_log_stats()

            except Exception as e:
                self.get_logger().warn(f'Audio capture error: {e}')
                time.sleep(0.01)

    def _maybe_log_stats(self):
        now = time.monotonic()
        dt = now - self._last_log_time
        if dt < 1.0:
            # Warn if we've been recording for a while with zero samples
            if (self._recording_started_at
                and now - self._recording_started_at > 3.0
                and self._chunks_published == 0
                and not self._warned_no_samples):
                self.get_logger().warn(
                    '/audio_enable has been True for 3s but no samples captured.')
                self._warned_no_samples = True
            return

        if self._last_sample_count > 0:
            rms = math.sqrt(self._last_rms_sq_sum / self._last_sample_count)
            # dBFS relative to full-scale int16 (32767)
            dbfs = 20 * math.log10(rms / 32767.0) if rms > 0 else -120.0
            rate = self._chunks_published / dt if dt > 0 else 0.0
            self.get_logger().info(
                f'[mic] chunks/s={rate:.1f} peak={self._last_peak} '
                f'rms={rms:.0f} ({dbfs:+.1f} dBFS)'
                + (' — SILENT (check mic mute / gain)' if dbfs < -60 else ''))
        self._last_log_time = now
        self._last_peak = 0
        self._last_rms_sq_sum = 0.0
        self._last_sample_count = 0
        self._chunks_published = 0

    def __del__(self):
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception:
                pass
        if self.pa:
            try:
                self.pa.terminate()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
