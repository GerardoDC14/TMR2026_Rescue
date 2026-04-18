"""
audio_node.py
ROS2 node that captures audio from USB microphone, encodes it with Opus
(narrowband voice-optimised), and publishes compressed frames on /audio.
Subscribes to /audio_enable to control recording state.

Auto-detects the Logitech C920 Pro webcam microphone by default. Pass
`device_index` to force a specific PortAudio input index.

Transport: UInt8MultiArray, one Opus packet per message (20 ms at 16 kHz by
default). Typical payload is ~40–60 bytes at 16 kbps — two orders of
magnitude smaller than the previous raw PCM stream.
"""

import ctypes
import ctypes.util
import math
import struct
import threading
import time

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, UInt8MultiArray


# Substrings that identify the Logitech C920 on Linux (order = preference)
C920_NAME_HINTS = (
    'C920',
    'HD Pro Webcam',
    'Logitech Webcam',
    'USB Audio',
    'Webcam',
)

# ── libopus minimal ctypes binding ──────────────────────────────────────────
# Avoids the pip `opuslib` dependency; libopus0 is a standard Ubuntu package.

OPUS_APPLICATION_VOIP = 2048
OPUS_APPLICATION_AUDIO = 2049
OPUS_SET_BITRATE_REQUEST = 4002
OPUS_SET_COMPLEXITY_REQUEST = 4010
OPUS_SET_INBAND_FEC_REQUEST = 4012
OPUS_SET_PACKET_LOSS_PERC_REQUEST = 4014
OPUS_SET_SIGNAL_REQUEST = 4024
OPUS_SIGNAL_VOICE = 3001


def _load_libopus():
    name = ctypes.util.find_library('opus') or 'libopus.so.0'
    lib = ctypes.CDLL(name)

    lib.opus_strerror.restype = ctypes.c_char_p
    lib.opus_strerror.argtypes = [ctypes.c_int]

    lib.opus_encoder_get_size.restype = ctypes.c_int
    lib.opus_encoder_get_size.argtypes = [ctypes.c_int]

    lib.opus_encoder_create.restype = ctypes.c_void_p
    lib.opus_encoder_create.argtypes = [
        ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.opus_encoder_destroy.restype = None
    lib.opus_encoder_destroy.argtypes = [ctypes.c_void_p]

    # opus_encode(st, pcm, frame_size, data, max_data_bytes)
    lib.opus_encode.restype = ctypes.c_int
    lib.opus_encode.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int16),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.c_int32,
    ]

    # opus_encoder_ctl is variadic — we only ever pass one int arg, which
    # shares the C va_list ABI with a fixed int parameter on Linux/amd64+arm64.
    lib.opus_encoder_ctl.restype = ctypes.c_int
    lib.opus_encoder_ctl.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
    ]
    return lib


class OpusEncoder:
    def __init__(self, sample_rate: int, bitrate_bps: int,
                 complexity: int, packet_loss_pct: int, use_fec: bool):
        self._lib = _load_libopus()
        err = ctypes.c_int(0)
        self._enc = self._lib.opus_encoder_create(
            sample_rate, 1, OPUS_APPLICATION_VOIP, ctypes.byref(err))
        if err.value != 0 or not self._enc:
            raise RuntimeError(
                f'opus_encoder_create failed: '
                f'{self._lib.opus_strerror(err.value).decode()}')

        def ctl(req, val, name):
            r = self._lib.opus_encoder_ctl(self._enc, req, val)
            if r < 0:
                raise RuntimeError(
                    f'opus CTL {name}({val}) failed: '
                    f'{self._lib.opus_strerror(r).decode()}')

        ctl(OPUS_SET_BITRATE_REQUEST, bitrate_bps, 'SET_BITRATE')
        ctl(OPUS_SET_COMPLEXITY_REQUEST, complexity, 'SET_COMPLEXITY')
        ctl(OPUS_SET_SIGNAL_REQUEST, OPUS_SIGNAL_VOICE, 'SET_SIGNAL')
        ctl(OPUS_SET_INBAND_FEC_REQUEST, 1 if use_fec else 0, 'SET_FEC')
        ctl(OPUS_SET_PACKET_LOSS_PERC_REQUEST,
            max(0, min(100, packet_loss_pct)), 'SET_PLC')

        self._max_packet = 4000  # RFC 6716 max
        self._buf = (ctypes.c_ubyte * self._max_packet)()

    def encode(self, pcm_int16: bytes, frame_size: int) -> bytes:
        pcm_arr = (ctypes.c_int16 * frame_size).from_buffer_copy(pcm_int16)
        n = self._lib.opus_encode(
            self._enc, pcm_arr, frame_size,
            self._buf, self._max_packet)
        if n < 0:
            raise RuntimeError(
                f'opus_encode failed: '
                f'{self._lib.opus_strerror(n).decode()}')
        return bytes(self._buf[:n])

    def close(self):
        if self._enc:
            self._lib.opus_encoder_destroy(self._enc)
            self._enc = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Parameters
        self.declare_parameter('sample_rate', 16000)        # Opus-valid: 8/12/16/24/48 kHz
        self.declare_parameter('frame_ms', 20)              # Opus-valid: 2.5/5/10/20/40/60
        self.declare_parameter('device_index', -1)          # -1 = auto-detect C920
        self.declare_parameter('device_name_hint', '')
        self.declare_parameter('opus_bitrate_kbps', 16)     # 6–64 is the VOIP sweet spot
        self.declare_parameter('opus_complexity', 5)        # 0–10, higher = better quality / more CPU
        self.declare_parameter('opus_fec', True)            # forward error correction
        self.declare_parameter('opus_packet_loss_pct', 20)  # expected loss % for FEC tuning

        self.sample_rate = self.get_parameter('sample_rate').value
        self.frame_ms = self.get_parameter('frame_ms').value
        self.device_index = self.get_parameter('device_index').value
        self.name_hint = self.get_parameter('device_name_hint').value
        bitrate_kbps = self.get_parameter('opus_bitrate_kbps').value
        complexity = self.get_parameter('opus_complexity').value
        use_fec = bool(self.get_parameter('opus_fec').value)
        plc_pct = self.get_parameter('opus_packet_loss_pct').value

        # Opus frame size in samples (= PyAudio chunk size)
        self.frame_size = int(self.sample_rate * self.frame_ms / 1000)
        if self.sample_rate not in (8000, 12000, 16000, 24000, 48000):
            self.get_logger().error(
                f'sample_rate={self.sample_rate} not valid for Opus '
                '(must be 8000, 12000, 16000, 24000, or 48000)')
            return
        if self.frame_ms not in (5, 10, 20, 40, 60):
            self.get_logger().error(
                f'frame_ms={self.frame_ms} not valid for Opus '
                '(must be 5, 10, 20, 40, or 60)')
            return

        # ── Opus encoder ────────────────────────────────────────────────────
        try:
            self.encoder = OpusEncoder(
                sample_rate=self.sample_rate,
                bitrate_bps=bitrate_kbps * 1000,
                complexity=complexity,
                packet_loss_pct=plc_pct,
                use_fec=use_fec,
            )
            self.get_logger().info(
                f'Opus encoder ready: {self.sample_rate}Hz mono, '
                f'{self.frame_ms}ms frames ({self.frame_size} samples), '
                f'{bitrate_kbps} kbps, complexity={complexity}, '
                f'FEC={"on" if use_fec else "off"} @ {plc_pct}% expected loss')
        except Exception as e:
            self.get_logger().error(f'Opus encoder init failed: {e}')
            return

        self.pa = pyaudio.PyAudio()
        self.stream = None
        self.recording = False
        self.recording_lock = threading.Lock()

        # Diagnostic counters
        self._chunks_published = 0
        self._bytes_out = 0
        self._last_log_time = time.monotonic()
        self._last_peak = 0
        self._last_rms_sq_sum = 0.0
        self._last_sample_count = 0
        self._recording_started_at = None
        self._warned_no_samples = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.audio_pub = self.create_publisher(UInt8MultiArray, '/audio', qos)
        self.audio_enable_sub = self.create_subscription(
            Bool, '/audio_enable', self._on_audio_enable, 10)

        # ── Device selection ───────────────────────────────────────────────
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
            self.get_logger().error(
                f'Device does NOT support {self.sample_rate}Hz mono int16: {e}. '
                f'Change sample_rate or pick a different device.')
            return

        try:
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=chosen_index,
                frames_per_buffer=self.frame_size,
            )
            self.get_logger().info(
                f'Audio stream OPEN: rate={self.sample_rate}Hz, '
                f'frame={self.frame_size} ({self.frame_ms}ms), '
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
                continue
            self.get_logger().info(
                f'  [{i}] name="{info["name"]}" '
                f'host_api={info["hostApi"]} '
                f'in_ch={max_in} '
                f'default_rate={int(info["defaultSampleRate"])} Hz')

    def _resolve_input_device(self):
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
                self._bytes_out = 0
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
                data = self.stream.read(self.frame_size, exception_on_overflow=False)

                # Encode to Opus
                packet = self.encoder.encode(data, self.frame_size)

                msg = UInt8MultiArray()
                msg.data = list(packet)
                self.audio_pub.publish(msg)

                self._chunks_published += 1
                self._bytes_out += len(packet)

                # Stats on raw PCM (for mic dBFS)
                samples = struct.unpack(f'<{self.frame_size}h', data)
                peak = max(abs(s) for s in samples)
                if peak > self._last_peak:
                    self._last_peak = peak
                self._last_rms_sq_sum += sum(s * s for s in samples)
                self._last_sample_count += len(samples)

                self._maybe_log_stats()

            except Exception as e:
                self.get_logger().warn(f'Audio capture/encode error: {e}')
                time.sleep(0.01)

    def _maybe_log_stats(self):
        now = time.monotonic()
        dt = now - self._last_log_time
        if dt < 1.0:
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
            dbfs = 20 * math.log10(rms / 32767.0) if rms > 0 else -120.0
            frames_per_sec = self._chunks_published / dt if dt > 0 else 0.0
            kbps = (self._bytes_out * 8) / dt / 1000.0
            avg_bytes = (self._bytes_out / self._chunks_published
                         if self._chunks_published else 0)
            self.get_logger().info(
                f'[mic] {frames_per_sec:.1f} fps peak={self._last_peak} '
                f'rms={rms:.0f} ({dbfs:+.1f} dBFS) '
                f'→ opus {kbps:.1f} kbps, avg {avg_bytes:.0f} B/frame'
                + (' — SILENT (check mic mute / gain)' if dbfs < -60 else ''))
        self._last_log_time = now
        self._last_peak = 0
        self._last_rms_sq_sum = 0.0
        self._last_sample_count = 0
        self._chunks_published = 0
        self._bytes_out = 0

    def __del__(self):
        if getattr(self, 'stream', None):
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception:
                pass
        if getattr(self, 'pa', None):
            try:
                self.pa.terminate()
            except Exception:
                pass
        if getattr(self, 'encoder', None):
            try:
                self.encoder.close()
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
