#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from bldc_can_tools.ginkgo_can_interface import GinkgoAdapterConfig, GinkgoCANInterface  # noqa: E402
from bldc_can_tools.utils import format_can_frame  # noqa: E402


TEST_ARBITRATION_ID = 0x123
TEST_PAYLOAD = bytes.fromhex("DE AD BE EF")


def parse_int(value: str) -> int:
    return int(value, 0)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal Ginkgo receive example for a 0x123 / DE AD BE EF sanity test.",
    )
    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel.")
    parser.add_argument("--bitrate", type=int, default=500, help="CAN bitrate in kbps.")
    parser.add_argument("--device-index", type=int, default=0, help="Ginkgo device index.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Receive timeout in seconds per poll.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=20,
        help="Stop after this many received frames. Use 0 for unlimited.",
    )
    parser.add_argument(
        "--arbitration-id",
        type=parse_int,
        default=TEST_ARBITRATION_ID,
        help="Expected standard CAN arbitration ID, default 0x123.",
    )
    parser.add_argument(
        "--payload",
        default="DEADBEEF",
        help="Expected payload in hex, default DEADBEEF.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    expected_payload = bytes.fromhex(args.payload.replace(" ", ""))

    interface = GinkgoCANInterface(
        GinkgoAdapterConfig(
            channel=args.channel,
            bitrate_kbps=args.bitrate,
            device_index=args.device_index,
        )
    )

    probe = GinkgoCANInterface.probe_backend(interface.config)
    if not probe.available:
        print(f"ERROR: Ginkgo backend unavailable: {probe.detail}", file=sys.stderr)
        return 1

    received_count = 0
    matched_count = 0
    print(
        "Waiting for standard CAN frames on channel=%d bitrate=%dkbps; expecting 0x%03X#%s"
        % (args.channel, args.bitrate, args.arbitration_id, expected_payload.hex().upper())
    )

    try:
        interface.open()
        print(f"Backend: {interface.backend_source}")

        while args.count == 0 or received_count < args.count:
            frame = interface.recv(timeout=args.timeout)
            if frame is None:
                print("timeout waiting for frame...")
                continue

            received_count += 1
            line = format_can_frame(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
                extended_id=frame.extended_id,
                remote_frame=frame.remote_frame,
                timestamp_s=time.time(),
            )
            print(line)

            if (
                not frame.extended_id
                and not frame.remote_frame
                and frame.arbitration_id == args.arbitration_id
                and frame.data == expected_payload
            ):
                matched_count += 1
                print(
                    "MATCH %d: received expected test frame 0x%03X#%s"
                    % (
                        matched_count,
                        args.arbitration_id,
                        expected_payload.hex().upper(),
                    )
                )

    except KeyboardInterrupt:
        print("Interrupted by user.")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        interface.close()

    print(
        "Done. received_frames=%d matching_test_frames=%d"
        % (received_count, matched_count)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

