from __future__ import annotations

import os
import platform
from ctypes import byref, c_uint
from dataclasses import dataclass

import ControlCAN as can

SUCCESS_CODES = (0, 1)


@dataclass(frozen=True)
class BitTiming:
    sjw: int
    bs1: int
    bs2: int
    brp: int


BIT_TIMINGS: dict[int, BitTiming] = {
    1000: BitTiming(sjw=1, bs1=2, bs2=1, brp=9),
    800: BitTiming(sjw=1, bs1=3, bs2=1, brp=12),
    500: BitTiming(sjw=1, bs1=4, bs2=1, brp=12),
    250: BitTiming(sjw=1, bs1=6, bs2=1, brp=18),
    125: BitTiming(sjw=1, bs1=6, bs2=1, brp=36),
    100: BitTiming(sjw=1, bs1=7, bs2=2, brp=60),
}


def _raise_if_invalid_channel(channel: int) -> None:
    if channel not in (0, 1):
        raise ValueError("CAN channel must be 0 or 1.")


def _raise_if_invalid_kbps(kbps: int) -> None:
    if kbps not in BIT_TIMINGS:
        valid = ", ".join(str(v) for v in sorted(BIT_TIMINGS))
        raise ValueError(f"Unsupported baud rate {kbps}. Valid values: {valid}.")


def _ensure_linux_permissions(require_root: bool) -> None:
    if not require_root:
        return
    if platform.system() == "Linux" and hasattr(os, "geteuid") and os.geteuid() != 0:
        raise PermissionError(
            "Run with sudo on Linux, or configure udev permissions for the adapter."
        )


def _build_open_filter(accept_extended: bool) -> can.VCI_FILTER_CONFIG:
    filter_cfg = can.VCI_FILTER_CONFIG()
    filter_cfg.FilterIndex = 0
    filter_cfg.Enable = 1
    filter_cfg.ExtFrame = 1 if accept_extended else 0
    filter_cfg.FilterMode = 0
    filter_cfg.ID_IDE = 0
    filter_cfg.ID_RTR = 0
    filter_cfg.ID_Std_Ext = 0
    filter_cfg.MASK_IDE = 0
    filter_cfg.MASK_RTR = 0
    filter_cfg.MASK_Std_Ext = 0
    return filter_cfg


def create_frame(
    frame_id: int,
    payload: bytes = b"",
    *,
    remote: bool = False,
    extended: bool = False,
    send_type: int = 0,
    remote_data_len: int = 8,
) -> can.VCI_CAN_OBJ:
    if not 0 <= frame_id <= 0x1FFFFFFF:
        raise ValueError("CAN ID must be in [0, 0x1FFFFFFF].")
    if len(payload) > 8:
        raise ValueError("Payload length cannot exceed 8 bytes.")
    if not 0 <= remote_data_len <= 8:
        raise ValueError("remote_data_len must be in [0, 8].")

    frame = can.VCI_CAN_OBJ()
    frame.ID = c_uint(frame_id)
    frame.SendType = send_type
    frame.RemoteFlag = 1 if remote else 0
    frame.ExternFlag = 1 if extended else 0
    frame.DataLen = remote_data_len if remote else len(payload)

    if not remote:
        for index, value in enumerate(payload):
            frame.Data[index] = value

    return frame


def frame_payload(frame: can.VCI_CAN_OBJ) -> bytes:
    return bytes(frame.Data[: frame.DataLen])


def format_frame(frame: can.VCI_CAN_OBJ) -> str:
    frame_id = f"{frame.ID:08X}" if frame.ExternFlag else f"{frame.ID:03X}"
    if frame.RemoteFlag:
        return f"0x{frame_id} [{frame.DataLen}] RTR"
    payload = " ".join(f"{value:02X}" for value in frame_payload(frame))
    return f"0x{frame_id} [{frame.DataLen}] {payload}"


class CANBus:
    def __init__(
        self,
        *,
        channel: int = 0,
        kbps: int = 500,
        dev_index: int = 0,
        device_type: int = can.VCI_USBCAN2,
        accept_extended: bool = True,
        require_linux_root: bool = True,
    ) -> None:
        _raise_if_invalid_channel(channel)
        _raise_if_invalid_kbps(kbps)

        self.channel = channel
        self.kbps = kbps
        self.dev_index = dev_index
        self.device_type = device_type
        self.accept_extended = accept_extended
        self.require_linux_root = require_linux_root
        self._opened = False
        self._started = False

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def is_started(self) -> bool:
        return self._started

    def __enter__(self) -> "CANBus":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def open(self) -> "CANBus":
        _ensure_linux_permissions(self.require_linux_root)

        detected = can.VCI_ScanDevice(1)
        if detected <= 0:
            raise RuntimeError(
                "No Ginkgo USB-CAN adapter detected. "
                "On Windows, check USB driver installation."
            )

        rc = can.VCI_OpenDevice(self.device_type, self.dev_index, 0)
        if rc not in SUCCESS_CODES:
            raise RuntimeError(f"VCI_OpenDevice failed with code {rc}.")

        self._opened = True
        try:
            self._configure_channel()
        except Exception:
            self.close()
            raise
        return self

    def _configure_channel(self) -> None:
        timing = BIT_TIMINGS[self.kbps]

        cfg = can.VCI_INIT_CONFIG_EX()
        cfg.CAN_Mode = can.WORKING_MODE_NORMAL
        cfg.CAN_ABOM = 0
        cfg.CAN_NART = 0
        cfg.CAN_RFLM = 0
        cfg.CAN_TXFP = 1
        cfg.CAN_RELAY = 0
        cfg.CAN_SJW = timing.sjw
        cfg.CAN_BS1 = timing.bs1
        cfg.CAN_BS2 = timing.bs2
        cfg.CAN_BRP = timing.brp

        rc = can.VCI_InitCANEx(self.device_type, self.dev_index, self.channel, byref(cfg))
        if rc != 1:
            raise RuntimeError(f"VCI_InitCANEx failed with code {rc}.")

        filter_cfg = _build_open_filter(self.accept_extended)
        rc = can.VCI_SetFilter(self.device_type, self.dev_index, self.channel, byref(filter_cfg))
        if rc != 1:
            raise RuntimeError(f"VCI_SetFilter failed with code {rc}.")

        rc = can.VCI_StartCAN(self.device_type, self.dev_index, self.channel)
        if rc != 1:
            raise RuntimeError(f"VCI_StartCAN failed with code {rc}.")

        self._started = True

    def close(self) -> None:
        if not self._opened:
            return

        if self._started:
            can.VCI_ResetCAN(self.device_type, self.dev_index, self.channel)
            self._started = False

        can.VCI_CloseDevice(self.device_type, self.dev_index)
        self._opened = False

    def send(self, frame: can.VCI_CAN_OBJ) -> None:
        sent = can.VCI_Transmit(
            self.device_type,
            self.dev_index,
            self.channel,
            byref(frame),
            1,
        )
        if sent != 1:
            raise RuntimeError(f"VCI_Transmit failed with code {sent}.")

    def send_frame(
        self,
        frame_id: int,
        payload: bytes = b"",
        *,
        remote: bool = False,
        extended: bool = False,
        send_type: int = 0,
        remote_data_len: int = 8,
    ) -> None:
        frame = create_frame(
            frame_id,
            payload,
            remote=remote,
            extended=extended,
            send_type=send_type,
            remote_data_len=remote_data_len,
        )
        self.send(frame)

    def receive(self, *, max_frames: int = 200, wait_ms: int = 0) -> list[can.VCI_CAN_OBJ]:
        if max_frames <= 0:
            return []

        buffer_type = can.VCI_CAN_OBJ * max_frames
        buffer = buffer_type()
        received = can.VCI_Receive(
            self.device_type,
            self.dev_index,
            self.channel,
            byref(buffer),
            max_frames,
            wait_ms,
        )
        if received <= 0:
            return []
        return [buffer[index] for index in range(received)]

    def receive_available(self, *, max_frames: int = 200) -> list[can.VCI_CAN_OBJ]:
        pending = can.VCI_GetReceiveNum(self.device_type, self.dev_index, self.channel)
        if pending <= 0:
            return []
        return self.receive(max_frames=min(max_frames, pending), wait_ms=0)
