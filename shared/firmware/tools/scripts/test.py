"""
test_comms.py — ESP32 ↔ Mini PC protocol smoke test
Run on the NUC: python3 test_comms.py <robot_port> [emulator_port]

  robot_port    — UART to the robot ESP32       (e.g. /dev/ttyUSB0)
  emulator_port — UART to the PPM emulator ESP32 (e.g. /dev/ttyUSB1)
                  If omitted, PPM passthrough tests are skipped.

Wiring required for PPM tests:
  Emulator GPIO 2 ──► Robot GPIO 34
  Emulator GND    ──► Robot GND
"""

import serial
import struct
import time
import sys

# ─── Config ──────────────────────────────────────────────────────────────────
PORT          = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
EMULATOR_PORT = sys.argv[2] if len(sys.argv) > 2 else None
BAUD          = 921600
EMULATOR_BAUD = 115200

# ─── Protocol constants ───────────────────────────────────────────────────────
SOF               = bytes([0xAA, 0x55])
MSG_TELEMETRY     = 0x01
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13

MODE = {0: 'INIT', 1: 'STANDBY', 2: 'MAIN', 3: 'ARM', 4: 'ESTOP'}

# ─── Frame builder ────────────────────────────────────────────────────────────
def build_frame(msg_type: int, payload: bytes = b'') -> bytes:
    n = len(payload)
    crc = msg_type ^ (n >> 8) ^ (n & 0xFF)
    for b in payload:
        crc ^= b
    return bytes([0xAA, 0x55, msg_type, n >> 8, n & 0xFF]) + payload + bytes([crc & 0xFF])

# ─── Streaming frame parser ───────────────────────────────────────────────────
class Parser:
    (SOF0, SOF1, TYPE, LEN_H, LEN_L, PAYLOAD, CRC) = range(7)

    def __init__(self):
        self._s    = self.SOF0
        self._type = 0
        self._len  = 0
        self._buf  = bytearray()
        self._crc  = 0

    def feed(self, data: bytes):
        """Yield (msg_type, payload_bytes) for each valid frame received."""
        for b in data:
            if self._s == self.SOF0:
                if b == 0xAA: self._s = self.SOF1
            elif self._s == self.SOF1:
                self._s = self.TYPE if b == 0x55 else self.SOF0
            elif self._s == self.TYPE:
                self._type = b;  self._crc = b;  self._s = self.LEN_H
            elif self._s == self.LEN_H:
                self._len = b << 8;  self._crc ^= b;  self._s = self.LEN_L
            elif self._s == self.LEN_L:
                self._len |= b;  self._crc ^= b
                self._buf  = bytearray()
                self._s    = self.PAYLOAD if self._len > 0 else self.CRC
            elif self._s == self.PAYLOAD:
                self._buf.append(b);  self._crc ^= b
                if len(self._buf) >= self._len: self._s = self.CRC
            elif self._s == self.CRC:
                if b == (self._crc & 0xFF):
                    yield (self._type, bytes(self._buf))
                self._s = self.SOF0

# ─── Telemetry decoder ────────────────────────────────────────────────────────
# TelemetryPayload (packed, 24 bytes):
#   mode(u8) flags(u8) ppm[6](u16) speed_l(i16) speed_r(i16) flipper(i16) uptime(u32)
TELEM_FMT  = '<BB6HhhhI'
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # should be 24

def decode_telemetry(payload: bytes) -> dict | None:
    if len(payload) < TELEM_SIZE:
        return None
    mode, flags, *ppm, sl, sr, flip, uptime = struct.unpack_from(TELEM_FMT, payload)
    return {
        'mode':       MODE.get(mode, f'?({mode})'),
        'ppm_ok':     bool(flags & 0x01),
        'sensors_on': bool(flags & 0x02),
        'can_ok':     bool(flags & 0x04),
        'estop':      bool(flags & 0x08),
        'ppm':        ppm,
        'speed_l':    sl  / 10.0,
        'speed_r':    sr  / 10.0,
        'flipper':    flip / 10.0,
        'uptime_ms':  uptime,
    }

# ─── Test helpers ─────────────────────────────────────────────────────────────
def wait_for_telemetry(ser: serial.Serial, parser: Parser, timeout: float = 2.0) -> dict | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.read(ser.in_waiting or 1)
        for msg_type, payload in parser.feed(raw):
            if msg_type == MSG_TELEMETRY:
                return decode_telemetry(payload)
    return None

def drain_telemetry(ser: serial.Serial, parser: Parser, count: int = 3):
    """Discard `count` frames so stale values from before a command are gone."""
    seen = 0
    deadline = time.time() + 2.0
    while seen < count and time.time() < deadline:
        raw = ser.read(ser.in_waiting or 1)
        for msg_type, _ in parser.feed(raw):
            if msg_type == MSG_TELEMETRY:
                seen += 1

def send(ser: serial.Serial, msg_type: int, payload: bytes = b''):
    frame = build_frame(msg_type, payload)
    ser.write(frame)
    ser.flush()

def emulator_cmd(emu: serial.Serial, cmd: str):
    """Send a newline-terminated command to the PPM emulator."""
    emu.write((cmd + '\n').encode())
    emu.flush()
    time.sleep(0.05)  # let the emulator apply it before next PPM frame

# ─── Tests ────────────────────────────────────────────────────────────────────
def test_packet_rate(ser, parser, _=None):
    print("\n[1] Packet rate — collecting 100 frames ...")
    ser.reset_input_buffer()   # discard buffered frames so we measure steady-state rate
    frames, t0 = 0, time.time()
    while frames < 100:
        raw = ser.read(ser.in_waiting or 1)
        for msg_type, _ in parser.feed(raw):
            if msg_type == MSG_TELEMETRY:
                frames += 1
    elapsed = time.time() - t0
    hz = frames / elapsed
    print(f"    {frames} frames in {elapsed:.2f}s → {hz:.1f} Hz  (expect ~50 Hz)")
    assert 40 < hz < 60, f"FAIL: rate {hz:.1f} Hz out of range"
    print("    PASS")


def test_telemetry_content(ser, parser, _=None):
    print("\n[2] Telemetry content (PPM emulator connected) ...")
    t = wait_for_telemetry(ser, parser)
    assert t is not None, "FAIL: no telemetry received"
    print(f"    mode       = {t['mode']}        (expect MAIN or STANDBY — PPM valid)")
    print(f"    ppm_ok     = {t['ppm_ok']}      (expect True — emulator running)")
    print(f"    can_ok     = {t['can_ok']}       (expect False — no MCP2515)")
    print(f"    uptime_ms  = {t['uptime_ms']}")
    print(f"    ppm[0..5]  = {list(t['ppm'])}")
    # With a valid PPM signal the robot transitions STANDBY → MAIN; both are acceptable here
    assert t['mode'] in ('STANDBY', 'MAIN'), f"FAIL: unexpected mode {t['mode']}"
    assert t['ppm_ok'],                      "FAIL: ppm_ok is False — is emulator wired to GPIO 34?"
    print("    PASS")


def test_uptime_increases(ser, parser, _=None):
    print("\n[3] Uptime monotonically increases ...")
    prev = None
    for _ in range(10):
        t = wait_for_telemetry(ser, parser)
        assert t is not None, "FAIL: no telemetry"
        if prev is not None:
            delta = t['uptime_ms'] - prev
            assert 10 < delta < 50, f"FAIL: uptime delta {delta} ms (expect ~20)"
        prev = t['uptime_ms']
    print(f"    Last uptime: {prev} ms  PASS")


def test_estop(ser, parser, _=None):
    print("\n[4] ESTOP / ESTOP_CLEAR round-trip ...")
    send(ser, MSG_ESTOP)
    time.sleep(0.1)
    t = wait_for_telemetry(ser, parser)
    assert t is not None, "FAIL: no telemetry after ESTOP"
    print(f"    After ESTOP:       mode = {t['mode']}   (expect ESTOP)")
    assert t['mode'] == 'ESTOP', f"FAIL: mode is {t['mode']}"

    send(ser, MSG_ESTOP_CLEAR)
    time.sleep(0.1)
    t = wait_for_telemetry(ser, parser)
    assert t is not None, "FAIL: no telemetry after ESTOP_CLEAR"
    print(f"    After ESTOP_CLEAR: mode = {t['mode']}   (expect STANDBY)")
    assert t['mode'] == 'STANDBY', f"FAIL: mode is {t['mode']}"
    print("    PASS")


def test_sensor_enable(ser, parser, _=None):
    print("\n[5] Sensor enable (sensors not connected — just verifying no crash) ...")
    send(ser, MSG_SENSOR_ENABLE, bytes([0x07]))  # enable all
    time.sleep(0.2)
    t = wait_for_telemetry(ser, parser)
    assert t is not None, "FAIL: ESP32 stopped responding after sensor enable"
    print(f"    Still alive, mode = {t['mode']},  sensors_on = {t['sensors_on']}")
    send(ser, MSG_SENSOR_ENABLE, bytes([0x00]))
    print("    PASS")


# ─── PPM tests (require emulator) ─────────────────────────────────────────────
def test_ppm_default_values(ser, parser, emu=None):
    """All 6 channels should arrive at ~1500 µs (emulator power-on default)."""
    print("\n[6] PPM — default channel values ...")
    # Ensure emulator is reset to defaults if we can
    if emu:
        emulator_cmd(emu, "reset")
    time.sleep(0.15)   # let the robot decoder see several fresh frames
    drain_telemetry(ser, parser, count=3)

    t = wait_for_telemetry(ser, parser)
    assert t is not None, "FAIL: no telemetry"
    assert t['ppm_ok'], "FAIL: ppm_ok is False — check emulator wiring to GPIO 34"

    TOLERANCE = 100   # µs — accounts for delayMicroseconds jitter on emulator
    all_ok = True
    for i, v in enumerate(t['ppm']):
        in_range  = 800  <= v <= 2500
        near_mid  = abs(v - 1500) <= TOLERANCE
        status    = "OK" if (in_range and near_mid) else "FAIL"
        if status == "FAIL": all_ok = False
        print(f"    ch{i+1} = {v:4d} µs   range={'OK' if in_range else 'FAIL'}   "
              f"≈1500={'OK' if near_mid else f'FAIL (off by {v-1500:+d})'}")

    assert all_ok, "FAIL: one or more channels out of expected range"
    print("    PASS")


def test_ppm_channel_passthrough(ser, parser, emu):
    """
    Set specific channel values on the emulator, then verify the robot ESP32
    reports the same values in its telemetry PPM array.
    Tolerance: ±60 µs (delayMicroseconds jitter on emulator).
    """
    print("\n[7] PPM — channel passthrough accuracy ...")
    TOLERANCE = 60  # µs

    # (channel 1-indexed, target µs)
    cases = [(1, 1000), (2, 1200), (3, 2000), (4, 1800), (5, 1000), (6, 1700)]

    all_ok = True
    for ch, target_us in cases:
        emulator_cmd(emu, f"ch{ch} {target_us}")
        # PPM frame is 20 ms; wait ~200 ms so the decoder sees 10 fresh frames
        time.sleep(0.15)
        drain_telemetry(ser, parser, count=3)

        t = wait_for_telemetry(ser, parser)
        assert t is not None, f"FAIL: no telemetry while testing ch{ch}"
        actual = t['ppm'][ch - 1]
        diff   = actual - target_us
        ok     = abs(diff) <= TOLERANCE
        if not ok: all_ok = False
        print(f"    ch{ch}: sent={target_us:4d}  got={actual:4d}  "
              f"diff={diff:+d}  {'OK' if ok else f'FAIL (tol=±{TOLERANCE})'}")

    # Leave emulator in a safe neutral state
    emulator_cmd(emu, "reset")
    assert all_ok, "FAIL: one or more channels exceeded passthrough tolerance"
    print("    PASS")


def test_ppm_range_clamping(ser, parser, emu):
    """
    Push emulator to values near the edges of the RC.cpp clamp (800–2500 µs).
    Values should never leave that window in telemetry.
    """
    print("\n[8] PPM — clamping at edges (800 µs and 2500 µs) ...")

    for target_us in (800, 2500):
        emulator_cmd(emu, f"ch2 {target_us}")
        time.sleep(0.15)
        drain_telemetry(ser, parser, count=3)

        t = wait_for_telemetry(ser, parser)
        assert t is not None, "FAIL: no telemetry"
        actual = t['ppm'][1]   # ch2
        print(f"    ch2: sent={target_us}  got={actual}")
        assert 800 <= actual <= 2500, f"FAIL: {actual} is outside clamp window"

    emulator_cmd(emu, "reset")
    print("    PASS")


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    print(f"Opening robot port  {PORT} at {BAUD} baud ...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()

    emu = None
    if EMULATOR_PORT:
        print(f"Opening emulator port {EMULATOR_PORT} at {EMULATOR_BAUD} baud ...")
        emu = serial.Serial(EMULATOR_PORT, EMULATOR_BAUD, timeout=0.1)
        time.sleep(0.5)
        emu.reset_input_buffer()
    else:
        print("  (no emulator port given — PPM passthrough/edge tests will be skipped)")

    parser = Parser()
    passed = failed = 0

    # Base tests always run
    base_tests = [
        test_packet_rate,
        test_telemetry_content,
        test_uptime_increases,
        test_estop,
        test_sensor_enable,
        test_ppm_default_values,   # passes with or without emulator serial port
    ]

    # Extra tests only when we can talk to the emulator
    emulator_tests = [
        test_ppm_channel_passthrough,
        test_ppm_range_clamping,
    ]

    for test in base_tests + (emulator_tests if emu else []):
        try:
            test(ser, parser, emu)
            passed += 1
        except AssertionError as e:
            print(f"    *** {e}")
            failed += 1
        except Exception as e:
            print(f"    *** EXCEPTION: {e}")
            failed += 1

    ser.close()
    if emu:
        emu.close()
    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed")
    return 0 if failed == 0 else 1

if __name__ == '__main__':
    sys.exit(main())
