from __future__ import annotations

import importlib
import importlib.util
import os
import platform
import threading
from abc import ABC, abstractmethod
from ctypes import byref, c_ubyte, c_uint
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Sequence


SUCCESS_CODES = (0, 1)
MAX_VENDOR_FILTERS = 14
_STDIO_SUPPRESSION_LOCK = threading.Lock()


class CANInterfaceError(RuntimeError):
    """Base error for CAN transport failures."""


class BackendUnavailableError(CANInterfaceError):
    """Raised when the Ginkgo backend cannot be located or loaded."""


class CANTimeoutError(CANInterfaceError):
    """Raised when a CAN operation times out."""


@dataclass(frozen=True)
class CANFrame:
    arbitration_id: int
    data: bytes
    extended_id: bool = False
    remote_frame: bool = False
    timestamp: float | None = None
    channel: int | None = None


@dataclass(frozen=True)
class CANFilter:
    arbitration_id: int
    mask: int = 0x7FF
    extended_id: bool = False
    remote_frame: bool = False
    enabled: bool = True


@dataclass(frozen=True)
class BitTiming:
    sjw: int
    bs1: int
    bs2: int
    brp: int


BIT_TIMINGS_KBPS: dict[int, BitTiming] = {
    1000: BitTiming(sjw=1, bs1=2, bs2=1, brp=9),
    800: BitTiming(sjw=1, bs1=3, bs2=1, brp=12),
    500: BitTiming(sjw=1, bs1=4, bs2=1, brp=12),
    250: BitTiming(sjw=1, bs1=6, bs2=1, brp=18),
    125: BitTiming(sjw=1, bs1=6, bs2=1, brp=36),
    100: BitTiming(sjw=1, bs1=7, bs2=2, brp=60),
}


@dataclass(frozen=True)
class BackendProbe:
    available: bool
    source: str | None
    detail: str


@dataclass(frozen=True)
class CANControllerStatus:
    error_interrupt: int
    rx_error_counter: int
    tx_error_counter: int
    esr: int
    tsr: int
    buffer_size: int
    reg_status: int
    reg_mode: int


@dataclass(frozen=True)
class GinkgoAdapterConfig:
    channel: int = 0
    bitrate_kbps: int = 1000
    device_index: int = 0
    device_type: int = 4
    accept_extended: bool = False
    require_linux_root: bool = False
    sdk_path_hint: str | None = None
    suppress_vendor_debug_output: bool = True


class BaseCANInterface(ABC):
    @property
    @abstractmethod
    def is_open(self) -> bool:
        """Return True when the CAN adapter is open."""

    @abstractmethod
    def open(self) -> "BaseCANInterface":
        """Open and configure the CAN interface."""

    @abstractmethod
    def close(self) -> None:
        """Close the CAN interface."""

    @abstractmethod
    def send(self, arbitration_id: int, data: bytes, extended_id: bool = False) -> None:
        """Send one CAN frame."""

    @abstractmethod
    def recv(self, timeout: float = 0.1) -> CANFrame | None:
        """Receive one CAN frame or return None on timeout."""

    @abstractmethod
    def set_filters(self, filters: Sequence[CANFilter] | None) -> None:
        """Apply adapter-side receive filters when supported."""

    def recv_many(self, timeout: float = 0.1, max_frames: int = 100) -> list[CANFrame]:
        frames: list[CANFrame] = []
        for _ in range(max_frames):
            frame = self.recv(timeout=timeout if not frames else 0.0)
            if frame is None:
                break
            frames.append(frame)
        return frames

    def drain(self, max_rounds: int = 20, max_frames_per_round: int = 100) -> list[CANFrame]:
        drained: list[CANFrame] = []
        for _ in range(max_rounds):
            batch = self.recv_many(timeout=0.0, max_frames=max_frames_per_round)
            if not batch:
                break
            drained.extend(batch)
        return drained


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _candidate_sdk_paths(hint: str | None) -> list[Path]:
    raw_candidates: list[Path] = []
    if hint:
        raw_candidates.append(Path(hint))

    env_hint = os.environ.get("GINKGO_CONTROL_CAN")
    if env_hint:
        raw_candidates.append(Path(env_hint))

    env_root = os.environ.get("GINKGO_SDK_ROOT")
    if env_root:
        raw_candidates.append(Path(env_root) / "ControlCAN.py")

    raw_candidates.append(
        _workspace_root()
        / "src"
        / "ginkgo_odrive_bridge"
        / "Python_USB_CAN_Test_64bits"
        / "ControlCAN.py"
    )

    paths: list[Path] = []
    seen: set[str] = set()
    for candidate in raw_candidates:
        normalized = candidate / "ControlCAN.py" if candidate.is_dir() else candidate
        key = str(normalized.resolve()) if normalized.exists() else str(normalized)
        if key in seen:
            continue
        seen.add(key)
        paths.append(normalized)
    return paths


def _load_module_from_path(module_path: Path) -> ModuleType:
    if not module_path.exists():
        raise FileNotFoundError(f"Vendor ControlCAN.py not found: {module_path}")

    spec = importlib.util.spec_from_file_location(
        "lktech_ginkgo_vendor_controlcan", module_path
    )
    if spec is None or spec.loader is None:
        raise BackendUnavailableError(f"Could not create module spec for {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _SuppressNativeStdIo:
    """Best-effort suppression for noisy vendor DLL stdout or stderr."""

    def __init__(self, enabled: bool) -> None:
        self._enabled = enabled
        self._saved_stdout_fd: int | None = None
        self._saved_stderr_fd: int | None = None
        self._devnull_fd: int | None = None

    def __enter__(self) -> None:
        if not self._enabled:
            return

        with _STDIO_SUPPRESSION_LOCK:
            try:
                self._devnull_fd = os.open(os.devnull, os.O_WRONLY)
                self._saved_stdout_fd = os.dup(1)
                self._saved_stderr_fd = os.dup(2)
                os.dup2(self._devnull_fd, 1)
                os.dup2(self._devnull_fd, 2)
            except OSError:
                self._restore()

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        if not self._enabled:
            return
        with _STDIO_SUPPRESSION_LOCK:
            self._restore()

    def _restore(self) -> None:
        try:
            if self._saved_stdout_fd is not None:
                os.dup2(self._saved_stdout_fd, 1)
            if self._saved_stderr_fd is not None:
                os.dup2(self._saved_stderr_fd, 2)
        finally:
            if self._saved_stdout_fd is not None:
                os.close(self._saved_stdout_fd)
                self._saved_stdout_fd = None
            if self._saved_stderr_fd is not None:
                os.close(self._saved_stderr_fd)
                self._saved_stderr_fd = None
            if self._devnull_fd is not None:
                os.close(self._devnull_fd)
                self._devnull_fd = None


class GinkgoCANInterface(BaseCANInterface):
    """Thin abstraction around the vendor Ginkgo ctypes binding.

    This wrapper intentionally depends on a real vendor SDK. If the SDK cannot be
    loaded, it fails clearly instead of pretending to communicate with hardware.
    """

    def __init__(self, config: GinkgoAdapterConfig | None = None) -> None:
        self.config = config or GinkgoAdapterConfig()
        self._sdk: ModuleType | None = None
        self._sdk_source: str | None = None
        self._opened = False
        self._started = False
        self._filters: list[CANFilter] = []

    @property
    def is_open(self) -> bool:
        return self._opened

    @property
    def backend_source(self) -> str | None:
        return self._sdk_source

    def __enter__(self) -> "GinkgoCANInterface":
        return self.open()

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        self.close()

    @classmethod
    def probe_backend(
        cls, config: GinkgoAdapterConfig | None = None
    ) -> BackendProbe:
        interface = cls(config=config)
        try:
            _, source = interface._load_vendor_sdk()
            return BackendProbe(
                available=True,
                source=source,
                detail=f"Loaded Ginkgo vendor backend from {source}",
            )
        except Exception as exc:
            return BackendProbe(
                available=False,
                source=None,
                detail=str(exc),
            )

    def open(self) -> "GinkgoCANInterface":
        self._validate_config()
        self._ensure_platform_permissions()
        sdk, _ = self._load_vendor_sdk()

        detected = self._call_scan_device(sdk, 1)
        if detected <= 0:
            raise BackendUnavailableError(
                "No Ginkgo USB-CAN adapter detected by VCI_ScanDevice(1). "
                "Check the USB connection and Windows driver installation."
            )

        rc = self._call_open_device(
            sdk,
            self.config.device_type,
            self.config.device_index,
            0,
        )
        if rc not in SUCCESS_CODES:
            raise CANInterfaceError(f"VCI_OpenDevice failed with code {rc}.")

        self._opened = True
        try:
            self._configure_channel()
        except Exception:
            self.close()
            raise
        return self

    def close(self) -> None:
        if not self._opened:
            return

        sdk, _ = self._load_vendor_sdk()
        if self._started:
            self._call_reset_can(
                sdk,
                self.config.device_type,
                self.config.device_index,
                self.config.channel,
            )
            self._started = False

        self._call_close_device(sdk, self.config.device_type, self.config.device_index)
        self._opened = False

    def send(self, arbitration_id: int, data: bytes, extended_id: bool = False) -> None:
        if not self._opened:
            raise CANInterfaceError("Cannot send before open().")
        if len(data) > 8:
            raise ValueError("CAN payload length cannot exceed 8 bytes.")

        sdk, _ = self._load_vendor_sdk()
        frame = sdk.VCI_CAN_OBJ()
        frame.ID = arbitration_id
        frame.SendType = 0
        frame.RemoteFlag = 0
        frame.ExternFlag = 1 if extended_id else 0
        frame.DataLen = len(data)
        for index, value in enumerate(data):
            frame.Data[index] = value

        sent = self._call_transmit(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
            byref(frame),
            1,
        )
        if sent != 1:
            raise CANInterfaceError(f"VCI_Transmit failed with code {sent}.")

    def recv(self, timeout: float = 0.1) -> CANFrame | None:
        frames = self.recv_many(timeout=timeout, max_frames=1)
        return frames[0] if frames else None

    def recv_many(self, timeout: float = 0.1, max_frames: int = 100) -> list[CANFrame]:
        if not self._opened:
            raise CANInterfaceError("Cannot receive before open().")
        if max_frames <= 0:
            return []

        sdk, _ = self._load_vendor_sdk()
        wait_ms = max(0, int(timeout * 1000.0))
        buffer_type = sdk.VCI_CAN_OBJ * max_frames
        buffer = buffer_type()
        received = self._call_receive(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
            byref(buffer),
            max_frames,
            wait_ms,
        )
        if received <= 0:
            return []

        frames: list[CANFrame] = []
        for index in range(received):
            vendor_frame = buffer[index]
            timestamp = (
                float(vendor_frame.TimeStamp) / 1000.0
                if getattr(vendor_frame, "TimeFlag", 0)
                else None
            )
            frames.append(
                CANFrame(
                    arbitration_id=int(vendor_frame.ID),
                    data=bytes(vendor_frame.Data[: vendor_frame.DataLen]),
                    extended_id=bool(vendor_frame.ExternFlag),
                    remote_frame=bool(vendor_frame.RemoteFlag),
                    timestamp=timestamp,
                    channel=self.config.channel,
                )
            )
        return frames

    def set_filters(self, filters: Sequence[CANFilter] | None) -> None:
        self._filters = list(filters or [])
        if self._opened:
            self._apply_filters()

    def get_pending_receive_count(self) -> int:
        if not self._opened:
            raise CANInterfaceError("Cannot query receive count before open().")
        sdk, _ = self._load_vendor_sdk()
        return self._call_get_receive_num(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
        )

    def read_controller_status(self) -> CANControllerStatus:
        if not self._opened:
            raise CANInterfaceError("Cannot query controller status before open().")
        sdk, _ = self._load_vendor_sdk()
        status = sdk.VCI_CAN_STATUS()
        rc = self._call_read_can_status(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
            byref(status),
        )
        if rc != 1:
            raise CANInterfaceError(f"VCI_ReadCANStatus failed with code {rc}.")
        return CANControllerStatus(
            error_interrupt=int(status.ErrInterrupt) & 0xFF,
            rx_error_counter=int(status.regRECounter) & 0xFF,
            tx_error_counter=int(status.regTECounter) & 0xFF,
            esr=int(status.regESR),
            tsr=int(status.regTSR),
            buffer_size=int(status.BufferSize),
            reg_status=int(status.regStatus) & 0xFF,
            reg_mode=int(status.regMode) & 0xFF,
        )

    def _configure_channel(self) -> None:
        sdk, _ = self._load_vendor_sdk()
        timing = BIT_TIMINGS_KBPS[self.config.bitrate_kbps]

        cfg = sdk.VCI_INIT_CONFIG_EX()
        cfg.CAN_Mode = sdk.WORKING_MODE_NORMAL
        cfg.CAN_ABOM = 0
        cfg.CAN_NART = 0
        cfg.CAN_RFLM = 0
        cfg.CAN_TXFP = 1
        cfg.CAN_RELAY = 0
        cfg.CAN_SJW = timing.sjw
        cfg.CAN_BS1 = timing.bs1
        cfg.CAN_BS2 = timing.bs2
        cfg.CAN_BRP = timing.brp

        rc = self._call_init_can_ex(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
            byref(cfg),
        )
        if rc != 1:
            raise CANInterfaceError(f"VCI_InitCANEx failed with code {rc}.")

        self._apply_filters()

        rc = self._call_start_can(
            sdk,
            self.config.device_type,
            self.config.device_index,
            self.config.channel,
        )
        if rc != 1:
            raise CANInterfaceError(f"VCI_StartCAN failed with code {rc}.")

        self._started = True

    def _apply_filters(self) -> None:
        sdk, _ = self._load_vendor_sdk()
        filters = self._filters or [
            CANFilter(
                arbitration_id=0x0,
                mask=0x0,
                extended_id=False,
                remote_frame=False,
                enabled=True,
            ),
            CANFilter(
                arbitration_id=0x0,
                mask=0x0,
                extended_id=True,
                remote_frame=False,
                enabled=True,
            ),
        ]

        if len(filters) > MAX_VENDOR_FILTERS:
            raise ValueError(
                f"Ginkgo backend supports at most {MAX_VENDOR_FILTERS} hardware filters."
            )

        for filter_index, current_filter in enumerate(filters):
            filter_cfg = sdk.VCI_FILTER_CONFIG()
            filter_cfg.Enable = 1 if current_filter.enabled else 0
            filter_cfg.FilterIndex = filter_index
            filter_cfg.FilterMode = 0
            filter_cfg.ExtFrame = 1 if current_filter.extended_id else 0
            filter_cfg.ID_Std_Ext = current_filter.arbitration_id
            filter_cfg.ID_IDE = 1 if current_filter.extended_id else 0
            filter_cfg.ID_RTR = 1 if current_filter.remote_frame else 0
            filter_cfg.MASK_Std_Ext = current_filter.mask
            # Match the vendor helper's permissive behavior for the default
            # open filter so we do not accidentally reject valid traffic.
            if current_filter.mask == 0 and current_filter.arbitration_id == 0:
                filter_cfg.MASK_IDE = 0
                filter_cfg.MASK_RTR = 0
            else:
                filter_cfg.MASK_IDE = 1
                filter_cfg.MASK_RTR = 1

            rc = self._call_set_filter(
                sdk,
                self.config.device_type,
                self.config.device_index,
                self.config.channel,
                byref(filter_cfg),
            )
            if rc != 1:
                raise CANInterfaceError(
                    f"VCI_SetFilter failed for filter index {filter_index} with code {rc}."
                )

        for filter_index in range(len(filters), MAX_VENDOR_FILTERS):
            filter_cfg = sdk.VCI_FILTER_CONFIG()
            filter_cfg.Enable = 0
            filter_cfg.FilterIndex = filter_index
            filter_cfg.FilterMode = 0
            rc = self._call_set_filter(
                sdk,
                self.config.device_type,
                self.config.device_index,
                self.config.channel,
                byref(filter_cfg),
            )
            if rc != 1:
                raise CANInterfaceError(
                    f"VCI_SetFilter disable failed for filter index {filter_index} with code {rc}."
                )

    def _load_vendor_sdk(self) -> tuple[ModuleType, str]:
        if self._sdk is not None and self._sdk_source is not None:
            return self._sdk, self._sdk_source

        last_error: Exception | None = None
        for module_path in _candidate_sdk_paths(self.config.sdk_path_hint):
            try:
                module = _load_module_from_path(module_path)
                self._sdk = module
                self._sdk_source = str(module_path)
                return module, self._sdk_source
            except Exception as exc:
                last_error = BackendUnavailableError(
                    f"Found candidate Ginkgo SDK at {module_path}, but loading it failed: {exc}"
                )

        try:
            module = importlib.import_module("ControlCAN")
            self._sdk = module
            self._sdk_source = "import ControlCAN"
            return module, self._sdk_source
        except Exception as exc:
            last_error = BackendUnavailableError(
                "Could not load ControlCAN from known workspace paths or PYTHONPATH. "
                f"Last error: {exc}"
            )

        if last_error is not None:
            raise last_error
        raise BackendUnavailableError("Ginkgo vendor SDK not found.")

    def _validate_config(self) -> None:
        if self.config.channel not in (0, 1):
            raise ValueError("Ginkgo CAN channel must be 0 or 1.")
        if self.config.bitrate_kbps not in BIT_TIMINGS_KBPS:
            valid = ", ".join(str(value) for value in sorted(BIT_TIMINGS_KBPS))
            raise ValueError(
                f"Unsupported bitrate_kbps={self.config.bitrate_kbps}. Valid values: {valid}"
            )

    def _ensure_platform_permissions(self) -> None:
        if (
            self.config.require_linux_root
            and platform.system() == "Linux"
            and hasattr(os, "geteuid")
            and os.geteuid() != 0
        ):
            raise PermissionError(
                "Linux access to the Ginkgo adapter usually needs sudo or a matching udev rule."
            )

    def _raw_sdk_function(self, sdk: ModuleType, name: str) -> Any | None:
        raw_library = getattr(sdk, "GinkgoLib", None)
        if raw_library is None:
            return None
        return getattr(raw_library, name, None)

    def _call_scan_device(self, sdk: ModuleType, need_init: int) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_ScanDevice")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(raw_fn(c_ubyte(need_init)))
        return int(sdk.VCI_ScanDevice(need_init))

    def _call_open_device(
        self, sdk: ModuleType, device_type: int, device_index: int, reserved: int
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_OpenDevice")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(c_uint(device_type), c_uint(device_index), c_uint(reserved))
                )
        return int(sdk.VCI_OpenDevice(device_type, device_index, reserved))

    def _call_close_device(
        self, sdk: ModuleType, device_type: int, device_index: int
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_CloseDevice")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(raw_fn(c_uint(device_type), c_uint(device_index)))
        return int(sdk.VCI_CloseDevice(device_type, device_index))

    def _call_init_can_ex(
        self,
        sdk: ModuleType,
        device_type: int,
        device_index: int,
        channel: int,
        config_ptr: Any,
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_InitCANEx")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(
                        c_uint(device_type),
                        c_uint(device_index),
                        c_uint(channel),
                        config_ptr,
                    )
                )
        return int(sdk.VCI_InitCANEx(device_type, device_index, channel, config_ptr))

    def _call_set_filter(
        self,
        sdk: ModuleType,
        device_type: int,
        device_index: int,
        channel: int,
        filter_ptr: Any,
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_SetFilter")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(
                        c_uint(device_type),
                        c_uint(device_index),
                        c_uint(channel),
                        filter_ptr,
                    )
                )
        return int(sdk.VCI_SetFilter(device_type, device_index, channel, filter_ptr))

    def _call_start_can(
        self, sdk: ModuleType, device_type: int, device_index: int, channel: int
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_StartCAN")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(c_uint(device_type), c_uint(device_index), c_uint(channel))
                )
        return int(sdk.VCI_StartCAN(device_type, device_index, channel))

    def _call_reset_can(
        self, sdk: ModuleType, device_type: int, device_index: int, channel: int
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_ResetCAN")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(c_uint(device_type), c_uint(device_index), c_uint(channel))
                )
        return int(sdk.VCI_ResetCAN(device_type, device_index, channel))

    def _call_transmit(
        self,
        sdk: ModuleType,
        device_type: int,
        device_index: int,
        channel: int,
        frame_ptr: Any,
        length: int,
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_Transmit")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(
                        c_uint(device_type),
                        c_uint(device_index),
                        c_uint(channel),
                        frame_ptr,
                        c_uint(length),
                    )
                )
        return int(sdk.VCI_Transmit(device_type, device_index, channel, frame_ptr, length))

    def _call_receive(
        self,
        sdk: ModuleType,
        device_type: int,
        device_index: int,
        channel: int,
        buffer_ptr: Any,
        length: int,
        wait_ms: int,
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_Receive")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(
                        c_uint(device_type),
                        c_uint(device_index),
                        c_uint(channel),
                        buffer_ptr,
                        c_uint(length),
                        c_uint(wait_ms),
                    )
                )
        return int(
            sdk.VCI_Receive(
                device_type,
                device_index,
                channel,
                buffer_ptr,
                length,
                wait_ms,
            )
        )

    def _call_get_receive_num(
        self, sdk: ModuleType, device_type: int, device_index: int, channel: int
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_GetReceiveNum")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(c_uint(device_type), c_uint(device_index), c_uint(channel))
                )
        return int(sdk.VCI_GetReceiveNum(device_type, device_index, channel))

    def _call_read_can_status(
        self,
        sdk: ModuleType,
        device_type: int,
        device_index: int,
        channel: int,
        status_ptr: Any,
    ) -> int:
        raw_fn = self._raw_sdk_function(sdk, "VCI_ReadCANStatus")
        if raw_fn is not None:
            with _SuppressNativeStdIo(self.config.suppress_vendor_debug_output):
                return int(
                    raw_fn(
                        c_uint(device_type),
                        c_uint(device_index),
                        c_uint(channel),
                        status_ptr,
                    )
                )
        return int(sdk.VCI_ReadCANStatus(device_type, device_index, channel, status_ptr))
