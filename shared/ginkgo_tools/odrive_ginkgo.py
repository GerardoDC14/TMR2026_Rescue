#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any, Callable, TypeVar

VENDOR_DIR_NAME = "Python_USB_CAN_Test_64bits"


def _resolve_vendor_dir() -> Path:
    candidates: list[Path] = []
    here = Path(__file__).resolve()

    env_path = os.environ.get("GINKGO_VENDOR_DIR")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    candidates.extend(
        [
            here.parent / VENDOR_DIR_NAME,
            here.parents[1] / VENDOR_DIR_NAME,
            here.parents[1] / "vendor" / VENDOR_DIR_NAME,
            here.parents[1] / "ros2_ws" / "src" / "ginkgo_odrive_bridge" / VENDOR_DIR_NAME,
            here.parents[2]
            / "jaguar"
            / "arm"
            / "src"
            / "ginkgo_odrive_bridge"
            / VENDOR_DIR_NAME,
        ]
    )

    for candidate in candidates:
        if (candidate / "ControlCAN.py").exists() and (candidate / "ginkgo_can").is_dir():
            return candidate

    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        f"Could not find {VENDOR_DIR_NAME}. Checked: {searched or 'no candidates'}"
    )


VENDOR_DIR = _resolve_vendor_dir()

# Mirror the supported rates from ginkgo_can.bus so argument validation works
# even before the native driver is loaded.
BIT_TIMINGS = {
    1000: (1, 2, 1, 9),
    800: (1, 3, 1, 12),
    500: (1, 4, 1, 12),
    250: (1, 6, 1, 18),
    125: (1, 6, 1, 36),
    100: (1, 7, 2, 60),
}

_BACKEND: dict[str, Any] | None = None


def _load_backend() -> dict[str, Any]:
    global _BACKEND

    if _BACKEND is not None:
        return _BACKEND

    if str(VENDOR_DIR) not in sys.path:
        sys.path.insert(0, str(VENDOR_DIR))

    try:
        ginkgo_can = import_module("ginkgo_can")
    except OSError as exc:
        linux_lib = VENDOR_DIR / "lib" / "linux" / "64bit" / "libGinkgo_Driver.so"
        raise RuntimeError(
            "Failed to load the Ginkgo native driver. "
            f"Expected vendor files under {VENDOR_DIR}. "
            f"On this machine the Linux shared library appears to be missing: {linux_lib}"
        ) from exc

    _BACKEND = {
        "CANBus": ginkgo_can.CANBus,
        "CMD_GET_ENCODER_ESTIMATES": ginkgo_can.CMD_GET_ENCODER_ESTIMATES,
        "CMD_SET_INPUT_POS": ginkgo_can.CMD_SET_INPUT_POS,
        "decode_encoder_estimates": ginkgo_can.decode_encoder_estimates,
        "encode_set_input_pos": ginkgo_can.encode_set_input_pos,
        "frame_payload": ginkgo_can.frame_payload,
        "make_cob_id": ginkgo_can.make_cob_id,
    }
    return _BACKEND

CMD_SET_AXIS_STATE = 0x007
CMD_GET_IQ = 0x014
CMD_GET_TEMPERATURE = 0x015
CMD_GET_BUS_VOLTAGE_CURRENT = 0x017
CMD_CLEAR_ERRORS = 0x018
CMD_GET_TORQUES = 0x01C
CMD_GET_POWERS = 0x01D

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

T = TypeVar("T")


def _decode_two_floats(payload: bytes) -> tuple[float, float]:
    if len(payload) < 8:
        raise ValueError("Expected at least 8 bytes in response payload.")
    return struct.unpack("<ff", payload[:8])


def _format_number(value: float, precision: int = 3) -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value:.{precision}f}"


@dataclass(frozen=True)
class TelemetrySnapshot:
    position_turns: float
    velocity_turns_s: float
    iq_setpoint_a: float
    iq_measured_a: float
    fet_temp_c: float
    motor_temp_c: float
    bus_voltage_v: float
    bus_current_a: float
    torque_target_nm: float
    torque_estimate_nm: float
    electrical_power_w: float
    mechanical_power_w: float

    def to_lines(self) -> list[str]:
        return [
            f"Encoder  : pos={_format_number(self.position_turns, 4)} turns, "
            f"vel={_format_number(self.velocity_turns_s, 4)} turns/s",
            f"Iq       : set={_format_number(self.iq_setpoint_a)} A, "
            f"measured={_format_number(self.iq_measured_a)} A",
            f"Temp     : fet={_format_number(self.fet_temp_c)} C, "
            f"motor={_format_number(self.motor_temp_c)} C",
            f"Bus      : voltage={_format_number(self.bus_voltage_v)} V, "
            f"current={_format_number(self.bus_current_a)} A",
            f"Torque   : target={_format_number(self.torque_target_nm)} Nm, "
            f"estimate={_format_number(self.torque_estimate_nm)} Nm",
            f"Power    : electrical={_format_number(self.electrical_power_w)} W, "
            f"mechanical={_format_number(self.mechanical_power_w)} W",
        ]


class ODriveGinkgoClient:
    def __init__(
        self,
        *,
        node_id: int,
        channel: int = 0,
        kbps: int = 500,
        require_linux_root: bool = False,
    ) -> None:
        if not 0 <= node_id <= 0x3F:
            raise ValueError("node_id must be in [0, 63].")
        if kbps not in BIT_TIMINGS:
            valid = ", ".join(str(rate) for rate in sorted(BIT_TIMINGS))
            raise ValueError(f"Unsupported kbps={kbps}. Valid values: {valid}")

        self.node_id = node_id
        self.channel = channel
        self.kbps = kbps
        self.require_linux_root = require_linux_root
        self.bus: Any | None = None

    def __enter__(self) -> "ODriveGinkgoClient":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self.bus is not None and self.bus.is_open

    def open(self) -> "ODriveGinkgoClient":
        if self.bus is not None:
            return self

        backend = _load_backend()
        can_bus_cls = backend["CANBus"]
        self.bus = can_bus_cls(
            channel=self.channel,
            kbps=self.kbps,
            accept_extended=False,
            require_linux_root=self.require_linux_root,
        )
        self.bus.open()
        return self

    def close(self) -> None:
        if self.bus is None:
            return
        self.bus.close()
        self.bus = None

    def _require_bus(self) -> Any:
        if self.bus is None:
            raise RuntimeError("Ginkgo CAN bus is not open.")
        return self.bus

    def _request(
        self,
        command_id: int,
        decoder: Callable[[bytes], T],
        *,
        timeout_s: float = 0.15,
        remote_data_len: int = 8,
    ) -> T:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be > 0.")

        backend = _load_backend()
        bus = self._require_bus()
        request_id = backend["make_cob_id"](self.node_id, command_id)

        bus.receive_available(max_frames=200)
        bus.send_frame(
            request_id,
            remote=True,
            extended=False,
            remote_data_len=remote_data_len,
        )

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for frame in bus.receive_available(max_frames=50):
                if int(frame.ID) != request_id or frame.RemoteFlag:
                    continue
                return decoder(backend["frame_payload"](frame))
            time.sleep(0.005)

        raise TimeoutError(
            f"Timed out waiting for response to command 0x{command_id:02X} "
            f"from node 0x{self.node_id:02X}."
        )

    def set_axis_state(self, state: int) -> None:
        backend = _load_backend()
        bus = self._require_bus()
        bus.send_frame(
            backend["make_cob_id"](self.node_id, CMD_SET_AXIS_STATE),
            struct.pack("<I", int(state)),
            extended=False,
        )

    def enter_closed_loop(self) -> None:
        self.set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def idle(self) -> None:
        self.set_axis_state(AXIS_STATE_IDLE)

    def clear_errors(self, *, identify: bool = False) -> None:
        backend = _load_backend()
        bus = self._require_bus()
        bus.send_frame(
            backend["make_cob_id"](self.node_id, CMD_CLEAR_ERRORS),
            bytes([1 if identify else 0]),
            extended=False,
        )

    def set_input_pos(self, position_turns: float, vel_ff: int = 0, torque_ff: int = 0) -> None:
        backend = _load_backend()
        bus = self._require_bus()
        bus.send_frame(
            backend["make_cob_id"](self.node_id, CMD_SET_INPUT_POS),
            backend["encode_set_input_pos"](position_turns, vel_ff=vel_ff, torque_ff=torque_ff),
            extended=False,
        )

    def read_encoder_estimates(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        backend = _load_backend()
        return self._request(
            backend["CMD_GET_ENCODER_ESTIMATES"],
            backend["decode_encoder_estimates"],
            timeout_s=timeout_s,
        )

    def read_iq(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        return self._request(CMD_GET_IQ, _decode_two_floats, timeout_s=timeout_s)

    def read_temperature(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        return self._request(CMD_GET_TEMPERATURE, _decode_two_floats, timeout_s=timeout_s)

    def read_bus_voltage_current(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        return self._request(
            CMD_GET_BUS_VOLTAGE_CURRENT,
            _decode_two_floats,
            timeout_s=timeout_s,
        )

    def read_torques(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        return self._request(CMD_GET_TORQUES, _decode_two_floats, timeout_s=timeout_s)

    def read_powers(self, *, timeout_s: float = 0.15) -> tuple[float, float]:
        return self._request(CMD_GET_POWERS, _decode_two_floats, timeout_s=timeout_s)

    def read_telemetry(
        self,
        *,
        timeout_s: float = 0.15,
        best_effort: bool = True,
    ) -> TelemetrySnapshot:
        def read_pair(reader: Callable[..., tuple[float, float]]) -> tuple[float, float]:
            try:
                return reader(timeout_s=timeout_s)
            except TimeoutError:
                if not best_effort:
                    raise
                return float("nan"), float("nan")

        position_turns, velocity_turns_s = read_pair(self.read_encoder_estimates)
        iq_setpoint_a, iq_measured_a = read_pair(self.read_iq)
        fet_temp_c, motor_temp_c = read_pair(self.read_temperature)
        bus_voltage_v, bus_current_a = read_pair(self.read_bus_voltage_current)
        torque_target_nm, torque_estimate_nm = read_pair(self.read_torques)
        electrical_power_w, mechanical_power_w = read_pair(self.read_powers)

        return TelemetrySnapshot(
            position_turns=position_turns,
            velocity_turns_s=velocity_turns_s,
            iq_setpoint_a=iq_setpoint_a,
            iq_measured_a=iq_measured_a,
            fet_temp_c=fet_temp_c,
            motor_temp_c=motor_temp_c,
            bus_voltage_v=bus_voltage_v,
            bus_current_a=bus_current_a,
            torque_target_nm=torque_target_nm,
            torque_estimate_nm=torque_estimate_nm,
            electrical_power_w=electrical_power_w,
            mechanical_power_w=mechanical_power_w,
        )


__all__ = [
    "AXIS_STATE_CLOSED_LOOP_CONTROL",
    "AXIS_STATE_IDLE",
    "BIT_TIMINGS",
    "ODriveGinkgoClient",
    "TelemetrySnapshot",
    "VENDOR_DIR",
]
