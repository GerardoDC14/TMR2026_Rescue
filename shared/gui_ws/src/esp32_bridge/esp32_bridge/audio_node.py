"""
audio_node.py
ROS2 node that captures audio from USB microphone and publishes it to /audio topic.
Subscribes to /audio_enable to control recording state.
"""

import pyaudio
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int16MultiArray, MultiArrayDimension, MultiArrayLayout


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('device_index', -1)  # -1 = default device

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.device_index = self.get_parameter('device_index').get_parameter_value().integer_value

        # Audio setup
        self.pa = pyaudio.PyAudio()
        self.stream = None
        self.recording = False
        self.recording_lock = threading.Lock()

        # QoS for best-effort audio streaming
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher
        self.audio_pub = self.create_publisher(Int16MultiArray, '/audio', qos)

        # Subscriber to control recording (keep reference so it doesn't get garbage collected)
        self.audio_enable_sub = self.create_subscription(Bool, '/audio_enable', self._on_audio_enable, 10)

        # Try to open audio stream
        try:
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index if self.device_index >= 0 else None,
                frames_per_buffer=self.chunk_size
            )
            self.get_logger().info(
                f'Audio stream opened: {self.sample_rate}Hz, '
                f'chunk_size={self.chunk_size}, device={self.device_index}')
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            return

        # Start audio capture thread
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('Audio capture node ready')

    def _on_audio_enable(self, msg: Bool):
        """Handle audio enable/disable requests."""
        with self.recording_lock:
            self.recording = msg.data
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f'Audio recording: {state}')

    def _capture_loop(self):
        """Continuously capture audio and publish to /audio topic."""
        import struct
        import time

        while rclpy.ok():
            # Check recording state (quick lock release)
            with self.recording_lock:
                is_recording = self.recording

            if not is_recording or not self.stream:
                # Sleep when not recording — do NOT call spin_once here,
                # the main thread's spin() owns the executor
                time.sleep(0.01)
                continue

            try:
                # Read audio chunk from microphone (outside lock to allow updates)
                data = self.stream.read(self.chunk_size)

                # Unpack int16 samples
                samples = struct.unpack(f'<{self.chunk_size}h', data)

                # Create and publish message
                msg = Int16MultiArray()
                msg.data = list(samples)

                self.audio_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Audio capture error: {e}')
                time.sleep(0.01)

    def __del__(self):
        """Cleanup audio resources."""
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception as e:
                self.get_logger().error(f'Error closing stream: {e}')

        if self.pa:
            try:
                self.pa.terminate()
            except Exception as e:
                self.get_logger().error(f'Error terminating PyAudio: {e}')


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
