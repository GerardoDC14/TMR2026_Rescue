"""
esp32_bridge_node.py
ROS2 Humble node that bridges the rescue robot ESP32 UART binary protocol
to ROS2 topics and vice-versa.

Binary frame format:
  [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
  CRC = XOR of TYPE ^ LEN_H ^ LEN_L ^ all PAYLOAD bytes

Published topics
────────────────
/robot/telemetry          std_msgs/Float32MultiArray   [speed_left_rpm, speed_right_rpm,
                                                          flipper_angle_deg, uptime_s]
/robot/mode               std_msgs/String              current RobotMode name
/robot/flags              std_msgs/UInt8               bitmask (ppm|sensors|can|estop)
/robot/ppm                std_msgs/Int16MultiArray     raw PPM µs values [ch1..ch6]
/robot/status             diagnostic_msgs/DiagnosticArray
/encoders/tracks          geometry_msgs/Vector3        x=left_rpm, y=right_rpm, z=0
/encoders/flipper         std_msgs/Float32MultiArray   [fl, fr, rl, rr] degrees
                                                        (only [0] valid on ROBOT_MAIN)
/sensors/imu              sensor_msgs/Imu              BNO055 euler orientation (converted to
                                                        quaternion in node) + accel + gyro
/sensors/mag              sensor_msgs/MagneticField    QMC5883L raw XYZ field
/sensors/thermal          sensor_msgs/Image            32×24 float32 image (°C)
/sensors/gas              std_msgs/Float32             Rs/Ro ratio
/motors/vesc_status       std_msgs/Float32MultiArray   [vesc_id, erpm, current_A, duty,
                                                         temp_fet_C, temp_motor_C, voltage_V]
/motors/main_status       std_msgs/Float32MultiArray   ROBOT_MAIN: [left_duty, right_duty,
                                                         flipper_duty] (normalised −1..+1)

Subscribed topics (PC → ESP32)
───────────────────────────────
/robot/estop              std_msgs/Bool                True=ESTOP, False=ESTOP_CLEAR
/arm/joint_command        std_msgs/Float32MultiArray   6 joint angles in degrees
/sensors/enable_mask      std_msgs/UInt8               bitmask: bit0=mag, 1=thermal, 2=gas, 3=imu
                                                        Starts at 0x00 (all off); GUI controls it.

Parameters
──────────
serial_port   (str)   /dev/ttyUSB0
baud_rate     (int)   921600
"""

import math
import struct
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial

from std_msgs.msg import (
    Bool, String, UInt8, Float32, Float32MultiArray, Int16MultiArray,
    UInt8MultiArray, MultiArrayDimension, MultiArrayLayout
)
from sensor_msgs.msg import Imu, MagneticField, Image
from geometry_msgs.msg import Vector3
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time


# ─── Protocol constants (must match config.h) ────────────────────────────────
SOF = b'\xAA\x55'
MSG_TELEMETRY   = 0x01
MSG_THERMAL     = 0x02
MSG_MAG         = 0x03
MSG_GAS         = 0x04
MSG_STATUS      = 0x05
MSG_IMU         = 0x06
MSG_ENCODER_EXT = 0x07
MSG_VESC_STATUS = 0x08

MSG_MOTOR_MAIN    = 0x09

MSG_ARM_JOINTS    = 0x10
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13
MSG_KEYBIND       = 0x14

MODE_NAMES = {0: 'INIT', 1: 'STANDBY', 2: 'NORMAL', 3: 'ARM', 4: 'ESTOP', 5: 'FLIPPER'}

PPM_CHANNELS = 6


# ─── Frame builder ────────────────────────────────────────────────────────────
def _build_frame(msg_type: int, payload: bytes) -> bytes:
    length = len(payload)
    len_h = (length >> 8) & 0xFF
    len_l = length & 0xFF
    crc = msg_type ^ len_h ^ len_l
    for b in payload:
        crc ^= b
    return bytes([0xAA, 0x55, msg_type, len_h, len_l]) + payload + bytes([crc])


# ─── Node ─────────────────────────────────────────────────────────────────────
class ESP32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        port      = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Open serial port
        self._ser = serial.Serial(port, baud_rate, timeout=0.1)
        self.get_logger().info(f'Opened serial port {port} @ {baud_rate} baud')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self._pub_telemetry = self.create_publisher(Float32MultiArray, '/robot/telemetry',     qos)
        self._pub_mode      = self.create_publisher(String,            '/robot/mode',          qos)
        self._pub_flags     = self.create_publisher(UInt8,             '/robot/flags',         qos)
        self._pub_ppm       = self.create_publisher(Int16MultiArray,   '/robot/ppm',           qos)
        self._pub_status    = self.create_publisher(DiagnosticArray,   '/robot/status',        qos)
        self._pub_tracks    = self.create_publisher(Vector3,           '/encoders/tracks',     qos)
        self._pub_flipper   = self.create_publisher(Float32MultiArray, '/encoders/flipper',    qos)
        self._pub_imu       = self.create_publisher(Imu,               '/sensors/imu',         qos)
        self._pub_mag       = self.create_publisher(MagneticField,     '/sensors/mag',         qos)
        self._pub_thermal   = self.create_publisher(Image,             '/sensors/thermal',     qos)
        self._pub_gas       = self.create_publisher(Float32,           '/sensors/gas',         qos)
        self._pub_vesc      = self.create_publisher(Float32MultiArray, '/motors/vesc_status',  qos)
        self._pub_motor_main = self.create_publisher(Float32MultiArray, '/motors/main_status', qos)

        # Subscribers
        self.create_subscription(Bool,              '/robot/estop',         self._on_estop,         10)
        self.create_subscription(Float32MultiArray, '/arm/joint_command',   self._on_arm_joints,    10)
        self.create_subscription(UInt8,             '/sensors/enable_mask', self._on_sensor_enable, 10)

        keybind_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(UInt8MultiArray,   '/robot/keybind',       self._on_keybind, keybind_qos)

        # Serial read thread
        self._rx_buf = bytearray()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Start with all sensors disabled (0x00); the GUI sends enable commands via /sensors/enable_mask
        self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([0x00])))
        self.get_logger().info('Sensor enable mask initialised: 0x00 (all sensors off)')

        self.get_logger().info('ESP32 bridge node ready')

    # ── RX loop ───────────────────────────────────────────────────────────────
    def _rx_loop(self):
        """Read bytes from serial and parse frames in a background thread."""
        state = 'SOF0'
        msg_type = 0
        length = 0
        payload = bytearray()
        running_crc = 0

        while rclpy.ok():
            try:
                raw = self._ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            if not raw:
                continue

            for byte in raw:
                if state == 'SOF0':
                    if byte == 0xAA:
                        state = 'SOF1'

                elif state == 'SOF1':
                    state = 'TYPE' if byte == 0x55 else 'SOF0'

                elif state == 'TYPE':
                    msg_type = byte
                    running_crc = byte
                    state = 'LEN_H'

                elif state == 'LEN_H':
                    length = byte << 8
                    running_crc ^= byte
                    state = 'LEN_L'

                elif state == 'LEN_L':
                    length |= byte
                    running_crc ^= byte
                    payload = bytearray()
                    state = 'CRC' if length == 0 else 'PAYLOAD'

                elif state == 'PAYLOAD':
                    payload.append(byte)
                    running_crc ^= byte
                    if len(payload) >= length:
                        state = 'CRC'

                elif state == 'CRC':
                    if byte == running_crc:
                        self._dispatch(msg_type, bytes(payload))
                    state = 'SOF0'

    def _dispatch(self, msg_type: int, payload: bytes):
        try:
            if msg_type == MSG_TELEMETRY:
                self._handle_telemetry(payload)
            elif msg_type == MSG_THERMAL:
                self._handle_thermal(payload)
            elif msg_type == MSG_MAG:
                self._handle_mag(payload)
            elif msg_type == MSG_GAS:
                self._handle_gas(payload)
            elif msg_type == MSG_STATUS:
                self._handle_status(payload)
            elif msg_type == MSG_IMU:
                self._handle_imu(payload)
            elif msg_type == MSG_ENCODER_EXT:
                self._handle_encoder_ext(payload)
            elif msg_type == MSG_VESC_STATUS:
                self._handle_vesc_status(payload)
            elif msg_type == MSG_MOTOR_MAIN:
                self._handle_motor_main(payload)
        except struct.error as e:
            self.get_logger().warn(f'Parse error type=0x{msg_type:02X}: {e}')

    # ── Message handlers ──────────────────────────────────────────────────────

    def _handle_telemetry(self, payload: bytes):
        # TelemetryPayload: u8 mode, u8 flags, u16[6] ppm, i16 spd_l, i16 spd_r,
        #                   i16 flipper_angle, u32 uptime_ms
        # Total: 2 + 12 + 6 + 4 = 24 bytes
        fmt = '<BB' + 'H' * PPM_CHANNELS + 'hhhl'
        size = struct.calcsize(fmt)
        if len(payload) < size:
            return
        fields = struct.unpack_from(fmt, payload)
        mode_val, flags = fields[0], fields[1]
        ppm = list(fields[2:2 + PPM_CHANNELS])
        spd_l_x10, spd_r_x10, flip_x10, uptime_ms = fields[2 + PPM_CHANNELS:]

        stamp = self.get_clock().now().to_msg()

        # /robot/mode
        mode_msg = String()
        mode_msg.data = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        self._pub_mode.publish(mode_msg)

        # /robot/flags
        flags_msg = UInt8()
        flags_msg.data = flags
        self._pub_flags.publish(flags_msg)

        # /robot/ppm
        ppm_msg = Int16MultiArray()
        ppm_msg.data = [int(v) for v in ppm]
        self._pub_ppm.publish(ppm_msg)

        # /robot/telemetry
        telem_msg = Float32MultiArray()
        telem_msg.data = [
            spd_l_x10 / 10.0,
            spd_r_x10 / 10.0,
            flip_x10  / 10.0,
            uptime_ms / 1000.0
        ]
        self._pub_telemetry.publish(telem_msg)

        # /encoders/tracks
        tracks_msg = Vector3()
        tracks_msg.x = spd_l_x10 / 10.0
        tracks_msg.y = spd_r_x10 / 10.0
        self._pub_tracks.publish(tracks_msg)

        # /encoders/flipper (ROBOT_MAIN single flipper → index 0)
        flip_msg = Float32MultiArray()
        flip_msg.data = [flip_x10 / 10.0, 0.0, 0.0, 0.0]
        self._pub_flipper.publish(flip_msg)

    def _handle_encoder_ext(self, payload: bytes):
        # EncoderExtPayload: i16[4] flipper angles × 10 deg (fl, fr, rl, rr)
        fmt = '<hhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        fl_x10, fr_x10, rl_x10, rr_x10 = struct.unpack_from(fmt, payload)

        flip_msg = Float32MultiArray()
        flip_msg.data = [fl_x10 / 10.0, fr_x10 / 10.0,
                         rl_x10 / 10.0, rr_x10 / 10.0]
        self._pub_flipper.publish(flip_msg)

    def _handle_imu(self, payload: bytes):
        # ImuPayload: i16 yaw, pitch, roll (×10 deg)
        #             i16 accel_x, y, z (×100 m/s²)
        #             i16 gyro_x, y, z (×1000 rad/s)
        #             u8 calib (bits[7:6]=sys, [5:4]=gyro, [3:2]=accel, [1:0]=mag)
        # = 9×2 + 1 = 19 bytes
        fmt = '<' + 'h' * 9 + 'B'
        if len(payload) < struct.calcsize(fmt):
            return
        (yaw_x10, pitch_x10, roll_x10,
         ax_100, ay_100, az_100,
         gx_1000, gy_1000, gz_1000,
         calib) = struct.unpack_from(fmt, payload)

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = 'imu_link'

        # Convert BNO055 ZYX Euler (degrees) → quaternion for sensor_msgs/Imu
        yaw   = math.radians(yaw_x10   / 10.0)
        pitch = math.radians(pitch_x10 / 10.0)
        roll  = math.radians(roll_x10  / 10.0)
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax_100 / 100.0
        imu_msg.linear_acceleration.y = ay_100 / 100.0
        imu_msg.linear_acceleration.z = az_100 / 100.0

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gx_1000 / 1000.0
        imu_msg.angular_velocity.y = gy_1000 / 1000.0
        imu_msg.angular_velocity.z = gz_1000 / 1000.0

        # Unknown covariance → -1 sentinel
        imu_msg.orientation_covariance[0]         = -1.0
        imu_msg.angular_velocity_covariance[0]    = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self._pub_imu.publish(imu_msg)

    def _handle_mag(self, payload: bytes):
        # MagPayload: i16 x_uT100, y_uT100, z_uT100
        fmt = '<hhh'
        if len(payload) < struct.calcsize(fmt):
            return
        x100, y100, z100 = struct.unpack_from(fmt, payload)

        mag_msg = MagneticField()
        mag_msg.header.stamp    = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'mag_link'
        # ROS convention: Tesla (sensor gives µT)
        mag_msg.magnetic_field.x = x100 / 100.0 * 1e-6
        mag_msg.magnetic_field.y = y100 / 100.0 * 1e-6
        mag_msg.magnetic_field.z = z100 / 100.0 * 1e-6
        self._pub_mag.publish(mag_msg)

    def _handle_thermal(self, payload: bytes):
        # ThermalPayload: i16[768] pixels (°C×10) — 1536 bytes
        n_pixels = 32 * 24
        expected = n_pixels * 2
        if len(payload) < expected:
            return
        pixels_raw = struct.unpack_from(f'<{n_pixels}h', payload, 0)

        # Use b''.join() to concatenate the generated bytes
        pixels_f32 = b''.join(
            struct.pack('<f', v / 10.0) for v in pixels_raw
        )

        img_msg = Image()
        img_msg.header.stamp    = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'thermal_link'
        img_msg.height   = 24
        img_msg.width    = 32
        img_msg.encoding = '32FC1'
        img_msg.step     = 32 * 4
        img_msg.data     = list(pixels_f32)
        self._pub_thermal.publish(img_msg)

    def _handle_gas(self, payload: bytes):
        # GasPayload: i16 rs_ro×100
        if len(payload) < 2:
            return
        rs_ro100, = struct.unpack_from('<h', payload)

        gas_msg = Float32()
        gas_msg.data = rs_ro100 / 100.0
        self._pub_gas.publish(gas_msg)

    def _handle_vesc_status(self, payload: bytes):
        # VescStatusPayload: u8 vesc_id, i32 erpm, i16 current×10, i16 duty×1000,
        #                    i16 temp_fet×10, i16 temp_motor×10, i16 v_in×10
        # Total: 1 + 4 + 2×5 = 15 bytes
        fmt = '<Bihhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        vesc_id, erpm, cur10, duty1000, tfet10, tmot10, vin10 = struct.unpack_from(fmt, payload)

        # Publish as [vesc_id, erpm, current_A, duty, temp_fet_C, temp_motor_C, voltage_V]
        msg = Float32MultiArray()
        msg.data = [
            float(vesc_id),
            float(erpm),
            cur10 / 10.0,
            duty1000 / 1000.0,
            tfet10 / 10.0,
            tmot10 / 10.0,
            vin10 / 10.0,
        ]
        self._pub_vesc.publish(msg)

    def _handle_motor_main(self, payload: bytes):
        # MainMotorPayload: i16 duty_left×1000, i16 duty_right×1000, i16 duty_flipper×1000
        fmt = '<hhh'
        if len(payload) < struct.calcsize(fmt):
            return
        duty_l, duty_r, duty_flip = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [duty_l / 1000.0, duty_r / 1000.0, duty_flip / 1000.0]
        self._pub_motor_main.publish(msg)

    def _handle_status(self, payload: bytes):
        if len(payload) < 4:
            return
        mode_val, flags, sensor_mask, _ = struct.unpack_from('<BBBB', payload)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name    = 'ESP32 Robot Status'
        status.level   = DiagnosticStatus.OK
        status.message = MODE_NAMES.get(mode_val, f'UNKNOWN_{mode_val}')
        status.values  = [
            KeyValue(key='mode',         value=str(mode_val)),
            KeyValue(key='ppm_ok',       value=str(bool(flags & 0x01))),
            KeyValue(key='sensors_on',   value=str(bool(flags & 0x02))),
            KeyValue(key='can_ok',       value=str(bool(flags & 0x04))),
            KeyValue(key='estop',        value=str(bool(flags & 0x08))),
            KeyValue(key='sensor_mask',  value=hex(sensor_mask)),
        ]
        if flags & 0x08:
            status.level   = DiagnosticStatus.ERROR
            status.message = 'ESTOP ACTIVE'
        diag.status = [status]
        self._pub_status.publish(diag)

    # ── TX helpers ────────────────────────────────────────────────────────────
    def _send(self, frame: bytes):
        try:
            self._ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _on_estop(self, msg: Bool):
        if msg.data:
            self._send(_build_frame(MSG_ESTOP, b''))
            self.get_logger().warn('Sent ESTOP')
        else:
            self._send(_build_frame(MSG_ESTOP_CLEAR, b''))
            self.get_logger().info('Sent ESTOP_CLEAR')

    def _on_arm_joints(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn('arm_joint_command must have 6 values')
            return
        payload = struct.pack('<' + 'h' * 6,
                              *[int(d * 100.0) for d in msg.data[:6]])
        self._send(_build_frame(MSG_ARM_JOINTS, payload))

    def _on_sensor_enable(self, msg: UInt8):
        self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([msg.data])))
        self.get_logger().info(f'Sensor enable mask set: 0x{msg.data:02X}')

    def _on_keybind(self, msg: UInt8MultiArray):
        if len(msg.data) < 15:
            self.get_logger().warn('Keybind message must have 15 bytes (3 modes x 5 channels)')
            return
        payload = bytes(msg.data[:15])
        self._send(_build_frame(MSG_KEYBIND, payload))
        self.get_logger().info('Keybind configuration sent to ESP32')


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
