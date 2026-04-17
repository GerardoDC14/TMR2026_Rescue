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
/motors/odrive_status     std_msgs/Float32MultiArray   [joint_idx, pos_turns, vel_turns_s,
                                                         iq_A, bus_voltage_V, bus_current_A]
/motors/lktech_status     std_msgs/Float32MultiArray   [joint_idx, motor_id, temp_C,
                                                         iq_A, speed_dps, angle_deg,
                                                         output_deg]
/motors/ze300_status      std_msgs/Float32MultiArray   [device_id, temp_C, iq_A,
                                                         speed_rpm, single_turn_counts,
                                                         position_counts, output_deg]
/motors/odrive_error      std_msgs/Float32MultiArray   [node_id, motor_error_bitfield]

Subscribed topics (PC → ESP32)
───────────────────────────────
/robot/estop              std_msgs/Bool                True=ESTOP, False=ESTOP_CLEAR
/joint_states             sensor_msgs/JointState       6 joint angles (Positions in radians)
/sensors/enable_mask      std_msgs/UInt8               bitmask: bit0=mag, 1=thermal, 2=gas, 3=imu
                                                        Starts at 0x00 (all off); GUI controls it.
/robot/ppm_calib          std_msgs/UInt16MultiArray    [min_us, neutral_us, max_us] RC calibration

Parameters
──────────
serial_port   (str)   /dev/ttyUSB0
baud_rate     (int)   921600
"""

import math
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial

from std_msgs.msg import (
    Bool, String, UInt8, Float32, Float32MultiArray, Int16MultiArray,
    UInt8MultiArray, UInt16MultiArray, MultiArrayDimension, MultiArrayLayout
)
from sensor_msgs.msg import Imu, MagneticField, Image, JointState
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
MSG_ODRIVE_STATUS = 0x0A
MSG_LKTECH_STATUS = 0x0B
MSG_ZE300_STATUS  = 0x0C
MSG_ODRIVE_ERROR  = 0x0D

MSG_ARM_JOINTS    = 0x10
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13
MSG_KEYBIND       = 0x14
MSG_PPM_CALIB     = 0x15
MSG_GRIPPER       = 0x16

MODE_NAMES = {0: 'INIT', 1: 'STANDBY', 2: 'NORMAL', 3: 'ARM', 4: 'ESTOP', 5: 'FLIPPER'}

PPM_CHANNELS = 6

# Largest legitimate payload is the thermal image (32*24*2 = 1536 bytes).
# Anything larger is a false SOF match inside another frame's payload.
MAX_PAYLOAD_LEN = 2048


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
        self.declare_parameter('reconnect_period', 3.0)
        self._serial_port     = self.get_parameter('serial_port').get_parameter_value().string_value
        self._baud_rate       = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self._reconnect_period = float(self.get_parameter('reconnect_period').value)
        self._ser: serial.Serial | None = None

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
        self._pub_odrive     = self.create_publisher(Float32MultiArray, '/motors/odrive_status', qos)
        self._pub_lktech     = self.create_publisher(Float32MultiArray, '/motors/lktech_status', qos)
        self._pub_ze300      = self.create_publisher(Float32MultiArray, '/motors/ze300_status',  qos)
        self._pub_odrive_err = self.create_publisher(Float32MultiArray, '/motors/odrive_error',  qos)
        self._pub_gripper    = self.create_publisher(Float32,           '/gripper',              qos)
        self._pub_deadband   = self.create_publisher(Float32,           '/robot/deadband',       qos)

        # Subscribers
        self.create_subscription(Bool,       '/robot/estop',         self._on_estop,         10)
        self.create_subscription(JointState, '/joint_states',        self._on_joint_states,  10)
        self.create_subscription(UInt8,      '/sensors/enable_mask', self._on_sensor_enable, 10)

        keybind_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(UInt8MultiArray,   '/robot/keybind',    self._on_keybind,    keybind_qos)
        self.create_subscription(UInt16MultiArray,  '/robot/ppm_calib',  self._on_ppm_calib,  keybind_qos)

        # Serial read thread — opens the port with retry, reconnects on error
        self._rx_buf = bytearray()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info('ESP32 bridge node ready (serial connection handled asynchronously)')

    # ── RX loop ───────────────────────────────────────────────────────────────
    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=0.1)
            # Discard any stale bytes the kernel buffered before we opened.
            # Without this, partial frames sitting in the OS buffer can trip
            # a false SOF match and stall the parser in PAYLOAD state.
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self.get_logger().info(
                f'Opened serial port {self._serial_port} @ {self._baud_rate} baud')
            # Start with all sensors disabled; the GUI sends enable commands via /sensors/enable_mask
            self._send(_build_frame(MSG_SENSOR_ENABLE, bytes([0x00])))
            return True
        except (serial.SerialException, OSError) as e:
            self._ser = None
            self.get_logger().warn(
                f'Serial port {self._serial_port} unavailable ({e}); '
                f'retrying in {self._reconnect_period:.0f}s')
            return False

    def _rx_loop(self):
        """Open serial (retry on failure), read bytes, and parse frames."""
        state = 'SOF0'
        msg_type = 0
        length = 0
        payload = bytearray()
        running_crc = 0

        while rclpy.ok():
            if self._ser is None:
                if not self._open_serial():
                    time.sleep(self._reconnect_period)
                    continue
                state, payload, running_crc = 'SOF0', bytearray(), 0

            try:
                raw = self._ser.read(256)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read error: {e}; will reconnect')
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                continue
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
                    if length > MAX_PAYLOAD_LEN:
                        # False SOF match — the claimed length is impossible.
                        # Bail out to resync instead of swallowing thousands of
                        # bytes (which would eat real frame boundaries).
                        state = 'SOF0'
                    else:
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
            elif msg_type == MSG_ODRIVE_STATUS:
                self._handle_odrive_status(payload)
            elif msg_type == MSG_LKTECH_STATUS:
                self._handle_lktech_status(payload)
            elif msg_type == MSG_ZE300_STATUS:
                self._handle_ze300_status(payload)
            elif msg_type == MSG_ODRIVE_ERROR:
                self._handle_odrive_error(payload)
            elif msg_type == MSG_GRIPPER:
                self._handle_gripper(payload)
        except struct.error as e:
            self.get_logger().warn(f'Parse error type=0x{msg_type:02X}: {e}')

    # ── Message handlers ──────────────────────────────────────────────────────

    def _handle_telemetry(self, payload: bytes):
        # TelemetryPayload: u8 mode, u8 flags, u16[6] ppm, i16 spd_l, i16 spd_r,
        #                   i16 flipper_angle, u32 uptime_ms
        # Total: 2 + 12 + 6 + 4 = 24 bytes
        fmt = '<BB' + 'H' * PPM_CHANNELS + 'hhhL'
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
        mag_msg.magnetic_field.x = float(x100)
        mag_msg.magnetic_field.y = float(y100)
        mag_msg.magnetic_field.z = float(z100)
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

    def _handle_odrive_status(self, payload: bytes):
        # OdriveStatusPayload: u8 joint_idx, i16 pos×100, i16 vel×100,
        #                      i16 iq×100, i16 bus_v×10, i16 bus_i×100
        # Total: 1 + 5×2 = 11 bytes
        fmt = '<Bhhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint_idx, pos100, vel100, iq100, bv10, bi100 = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [
            float(joint_idx),
            pos100 / 100.0,
            vel100 / 100.0,
            iq100 / 100.0,
            bv10 / 10.0,
            bi100 / 100.0,
        ]
        self._pub_odrive.publish(msg)

    def _handle_lktech_status(self, payload: bytes):
        # LktechStatusPayload: u8 joint_idx, u8 motor_id, i8 temp_c,
        #                      i16 iq×100, i16 speed_dps, i16 angle_deg, i16 output_deg×10
        # Total: 3 + 4×2 = 11 bytes
        fmt = '<BBbhhhh'
        if len(payload) < struct.calcsize(fmt):
            return
        joint_idx, motor_id, temp_c, iq100, speed_dps, angle_deg, out_deg10 = \
            struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [
            float(joint_idx),
            float(motor_id),
            float(temp_c),
            iq100 / 100.0,
            float(speed_dps),
            float(angle_deg),
            out_deg10 / 10.0,
        ]
        self._pub_lktech.publish(msg)

    def _handle_ze300_status(self, payload: bytes):
        # Ze300StatusPayload: u8 device_id, i8 temp_c, i16 iq×1000,
        #                     i16 speed_rpm×100, i16 single_turn_counts,
        #                     i32 position_counts, i16 output_deg×10
        # Total: 2 + 3×2 + 4 + 2 = 14 bytes
        fmt = '<Bbhhhih'
        if len(payload) < struct.calcsize(fmt):
            return
        dev_id, temp_c, iq1000, spd100, st_counts, pos_counts, out_deg10 = \
            struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [
            float(dev_id),
            float(temp_c),
            iq1000 / 1000.0,
            spd100 / 100.0,
            float(st_counts),
            float(pos_counts),
            out_deg10 / 10.0,
        ]
        self._pub_ze300.publish(msg)

    def _handle_odrive_error(self, payload: bytes):
        # OdriveErrorPayload: u8 node_id, u64 motor_error
        # Total: 1 + 8 = 9 bytes
        fmt = '<BQ'
        if len(payload) < struct.calcsize(fmt):
            return
        node_id, motor_error = struct.unpack_from(fmt, payload)
        msg = Float32MultiArray()
        msg.data = [float(node_id), float(motor_error)]
        self._pub_odrive_err.publish(msg)
        if motor_error != 0:
            self.get_logger().warn(
                f'ODrive node {node_id} error: 0x{motor_error:016X}')

    def _handle_gripper(self, payload: bytes):
        # GripperPayload: i16 normalised × 1000
        if len(payload) < 2:
            return
        val_1000, = struct.unpack_from('<h', payload)
        msg = Float32()
        msg.data = val_1000 / 1000.0
        self._pub_gripper.publish(msg)

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
        if self._ser is None:
            return
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write error: {e}; will reconnect')
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ── Subscribers ───────────────────────────────────────────────────────────
    def _on_estop(self, msg: Bool):
        if msg.data:
            self._send(_build_frame(MSG_ESTOP, b''))
            self.get_logger().warn('Sent ESTOP')
        else:
            self._send(_build_frame(MSG_ESTOP_CLEAR, b''))
            #self.get_logger().info('Sent ESTOP_CLEAR')

    def _on_joint_states(self, msg: JointState):
        """
        MoveIt publishes JointState arrays, but the order isn't guaranteed. 
        It's highly recommended to map positions by msg.name to guarantee the ESP32 
        gets them in the strict [J1, J2, J3, J4, J5, J6] order.
        """
        # TODO: Update these strings to match your actual URDF joint names!
        EXPECTED_JOINT_NAMES = [
            'Joint1', 'Joint2', 'Joint3', 
            'Joint4', 'Joint5', 'Joint6'
        ]

        if not msg.name or not msg.position:
            return

        # Create a dictionary to map joint names to their current position (in radians)
        joint_map = dict(zip(msg.name, msg.position))
        
        deg_positions = []
        for j_name in EXPECTED_JOINT_NAMES:
            # Fallback to 0.0 if the joint is missing from the message
            rad_pos = joint_map.get(j_name, 0.0) 
            # Convert radians to degrees for the ESP32 payload
            deg_positions.append(math.degrees(rad_pos))

        # Pack the 6 degrees values into 6 int16s (multiplying by 100)
        payload = struct.pack('<' + 'h' * 6,
                              *[int(d * 100.0) for d in deg_positions])
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

    def _on_ppm_calib(self, msg: UInt16MultiArray):
        # PpmCalibPayload: 6 channels × (min_us, neutral_us, max_us) uint16 = 36 bytes
        #                + uint16 deadband_1000 = 38 bytes total (19 uint16 values)
        # GUI sends 19 values: [ch0_min, ch0_neu, ch0_max, ..., ch5_max, deadband_1000]
        if len(msg.data) < 19:
            self.get_logger().warn(
                f'ppm_calib must have 19 values (6ch × 3 + deadband), got {len(msg.data)}')
            return
        payload = struct.pack('<' + 'HHH' * 6 + 'H',
                              *[int(v) for v in msg.data[:19]])
        self._send(_build_frame(MSG_PPM_CALIB, payload))

        # Publish deadband so rc_servo.py can use it
        deadband_val = msg.data[18] / 1000.0
        db_msg = Float32()
        db_msg.data = deadband_val
        self._pub_deadband.publish(db_msg)
        self.get_logger().info(
            f'PPM calibration sent to ESP32 (deadband={deadband_val:.3f})')


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