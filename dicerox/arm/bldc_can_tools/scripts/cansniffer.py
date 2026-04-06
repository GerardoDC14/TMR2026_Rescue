#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from bldc_can_tools.ginkgo_can_interface import (  # noqa: E402
    CANFilter,
    GinkgoAdapterConfig,
    GinkgoCANInterface,
)
from bldc_can_tools.lktech_protocol import (  # noqa: E402
    decode_lktech_frame,
    extract_motor_id_from_arbitration_id,
)
from bldc_can_tools.utils import format_can_frame  # noqa: E402


def parse_int(value: str) -> int:
    return int(value, 0)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Practical CAN sniffer for a Ginkgo USB-CAN adapter.",
    )
    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel.")
    parser.add_argument("--bitrate", type=int, default=1000, help="CAN bitrate in kbps.")
    parser.add_argument("--device-index", type=int, default=0, help="Ginkgo device index.")
    parser.add_argument(
        "--extended",
        action="store_true",
        help="Accept extended CAN frames when configuring the adapter.",
    )
    parser.add_argument(
        "--motor-id",
        type=int,
        help="Optional software filter for a single motor ID based on 0x140/0x240 style IDs.",
    )
    parser.add_argument(
        "--arbitration-id",
        type=parse_int,
        help="Optional exact CAN arbitration ID filter. Accepts decimal or hex such as 0x14F.",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="Optional text log file for captured frames.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Receive timeout in seconds per poll.",
    )
    parser.add_argument(
        "--count",
        type=int,
        help="Stop after this many matching frames.",
    )
    parser.add_argument(
        "--decode",
        choices=("raw", "lktech"),
        default="raw",
        help="Optional payload decode mode.",
    )
    parser.add_argument(
        "--diagnostic",
        action="store_true",
        help="Print adapter-side pending receive counts while sniffing.",
    )
    parser.add_argument(
        "--diagnostic-status-every",
        type=int,
        default=10,
        help="In diagnostic mode, print controller status every N loops. Use 0 to disable status output.",
    )
    return parser


def frame_matches_filters(frame, args: argparse.Namespace) -> bool:
    if args.arbitration_id is not None and frame.arbitration_id != args.arbitration_id:
        return False
    if args.motor_id is not None:
        detected_motor_id = extract_motor_id_from_arbitration_id(frame.arbitration_id)
        if detected_motor_id != args.motor_id:
            return False
    return True


def format_output_line(frame, args: argparse.Namespace) -> str:
    line = format_can_frame(
        arbitration_id=frame.arbitration_id,
        data=frame.data,
        extended_id=frame.extended_id,
        remote_frame=frame.remote_frame,
        timestamp_s=time.time(),
    )
    if args.decode == "lktech":
        decoded = decode_lktech_frame(frame.arbitration_id, frame.data)
        if decoded:
            line += f"  |  {decoded}"
    return line


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    interface = GinkgoCANInterface(
        GinkgoAdapterConfig(
            channel=args.channel,
            bitrate_kbps=args.bitrate,
            device_index=args.device_index,
            accept_extended=args.extended,
        )
    )

    if args.arbitration_id is not None:
        interface.set_filters(
            [
                CANFilter(
                    arbitration_id=args.arbitration_id,
                    mask=0x1FFFFFFF if args.extended else 0x7FF,
                    extended_id=args.extended,
                )
            ]
        )

    probe = GinkgoCANInterface.probe_backend(interface.config)
    if not probe.available:
        print(f"WARNING: Ginkgo backend is not available yet: {probe.detail}", file=sys.stderr)

    log_handle = None
    matched_frames = 0
    diagnostic_loops = 0

    try:
        if args.log_file is not None:
            log_handle = args.log_file.open("a", encoding="utf-8")

        interface.open()
        print(
            "Sniffing on channel=%d bitrate=%dkbps backend=%s"
            % (
                args.channel,
                args.bitrate,
                interface.backend_source or "unknown",
            )
        )

        while args.count is None or matched_frames < args.count:
            if args.diagnostic:
                pending = interface.get_pending_receive_count()
                diagnostic_loops += 1
                print(
                    "diagnostic channel=%d bitrate=%d pending=%d"
                    % (args.channel, args.bitrate, pending)
                )
                if (
                    args.diagnostic_status_every > 0
                    and diagnostic_loops % args.diagnostic_status_every == 0
                ):
                    status = interface.read_controller_status()
                    print(
                        "diagnostic status rx_err=%d tx_err=%d err_irq=0x%02X reg_status=0x%02X buffer_size=%d"
                        % (
                            status.rx_error_counter,
                            status.tx_error_counter,
                            status.error_interrupt,
                            status.reg_status,
                            status.buffer_size,
                        )
                    )
            frame = interface.recv(timeout=args.timeout)
            if frame is None:
                continue
            if not frame_matches_filters(frame, args):
                continue

            line = format_output_line(frame, args)
            print(line)
            if log_handle is not None:
                log_handle.write(line + "\n")
                log_handle.flush()
            matched_frames += 1

    except KeyboardInterrupt:
        print("Sniffer interrupted by user.")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        interface.close()
        if log_handle is not None:
            log_handle.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

