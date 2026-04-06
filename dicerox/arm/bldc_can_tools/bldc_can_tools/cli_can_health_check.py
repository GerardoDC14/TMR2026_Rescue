from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from bldc_can_tools.ginkgo_can_interface import (
        BIT_TIMINGS_KBPS,
        CANControllerStatus,
        GinkgoAdapterConfig,
        GinkgoCANInterface,
    )
else:
    from .ginkgo_can_interface import (
        BIT_TIMINGS_KBPS,
        CANControllerStatus,
        GinkgoAdapterConfig,
        GinkgoCANInterface,
    )


ERR_CAN_BUSERR = 0x10
ERR_CAN_PASSIVE = 0x04
ERR_CAN_BUSOFF = 0x20


@dataclass(frozen=True)
class HealthCheckResult:
    channel: int
    bitrate: int
    pending_max: int
    rx_err: int
    tx_err: int
    err_irq: int
    reg_status: int
    verdict: str


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Sweep Ginkgo CAN channels and bitrates and report bus health.",
    )
    parser.add_argument(
        "--channels",
        type=int,
        nargs="+",
        default=[0, 1],
        help="CAN channels to test. Default: 0 1",
    )
    parser.add_argument(
        "--bitrates",
        type=int,
        nargs="+",
        default=sorted(BIT_TIMINGS_KBPS),
        help="CAN bitrates in kbps to test. Default: all supported bitrates.",
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="Ginkgo device index.",
    )
    parser.add_argument(
        "--sample-seconds",
        type=float,
        default=1.0,
        help="How long to observe each channel/bitrate combination.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.1,
        help="Diagnostic polling interval in seconds.",
    )
    return parser


def classify_status(pending_max: int, status: CANControllerStatus) -> str:
    if pending_max > 0:
        return "frames_seen"
    if status.error_interrupt & (ERR_CAN_BUSERR | ERR_CAN_PASSIVE | ERR_CAN_BUSOFF):
        return "bus_error"
    if status.rx_error_counter > 0:
        return "rx_errors"
    return "quiet_bus"


def run_health_check(
    channel: int,
    bitrate: int,
    device_index: int,
    sample_seconds: float,
    poll_interval: float,
) -> HealthCheckResult:
    interface = GinkgoCANInterface(
        GinkgoAdapterConfig(
            channel=channel,
            bitrate_kbps=bitrate,
            device_index=device_index,
        )
    )

    pending_max = 0
    last_status: CANControllerStatus | None = None
    try:
        interface.open()
        deadline = time.monotonic() + sample_seconds
        while time.monotonic() < deadline:
            pending = interface.get_pending_receive_count()
            pending_max = max(pending_max, pending)
            last_status = interface.read_controller_status()
            time.sleep(max(0.0, poll_interval))

        if last_status is None:
            last_status = interface.read_controller_status()

        return HealthCheckResult(
            channel=channel,
            bitrate=bitrate,
            pending_max=pending_max,
            rx_err=last_status.rx_error_counter,
            tx_err=last_status.tx_error_counter,
            err_irq=last_status.error_interrupt,
            reg_status=last_status.reg_status,
            verdict=classify_status(pending_max, last_status),
        )
    finally:
        interface.close()


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    exit_code = 0
    for channel in args.channels:
        for bitrate in args.bitrates:
            try:
                result = run_health_check(
                    channel=channel,
                    bitrate=bitrate,
                    device_index=args.device_index,
                    sample_seconds=args.sample_seconds,
                    poll_interval=args.poll_interval,
                )
                print(
                    "channel=%d bitrate=%d pending_max=%d rx_err=%d tx_err=%d err_irq=0x%02X reg_status=0x%02X verdict=%s"
                    % (
                        result.channel,
                        result.bitrate,
                        result.pending_max,
                        result.rx_err,
                        result.tx_err,
                        result.err_irq,
                        result.reg_status,
                        result.verdict,
                    )
                )
            except Exception as exc:
                exit_code = 1
                print(
                    "channel=%d bitrate=%d ERROR: %s" % (channel, bitrate, exc),
                    file=sys.stderr,
                )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

