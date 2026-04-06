#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time

from odrive_ginkgo import ODriveGinkgoClient


def _node_id(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read live ODrive telemetry over CAN using the Ginkgo adapter.",
    )
    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel (default: 0).")
    parser.add_argument("--kbps", type=int, default=500, help="CAN bitrate in kbps (default: 500).")
    parser.add_argument(
        "--node-id",
        type=_node_id,
        default=0x10,
        help="ODrive node ID in decimal or hex (default: 0x10).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.15,
        help="Per-request timeout in seconds (default: 0.15).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between reads when monitoring continuously (default: 1.0).",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of samples to read. Use 0 to run until Ctrl+C (default: 0).",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail if any telemetry request times out instead of printing n/a values.",
    )
    parser.add_argument(
        "--require-linux-root",
        action="store_true",
        help="Require root on Linux instead of relying on udev permissions.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    with ODriveGinkgoClient(
        node_id=args.node_id,
        channel=args.channel,
        kbps=args.kbps,
        require_linux_root=args.require_linux_root,
    ) as client:
        sample_index = 0
        while True:
            sample_index += 1
            snapshot = client.read_telemetry(timeout_s=args.timeout, best_effort=not args.strict)
            timestamp = time.strftime("%H:%M:%S")

            print(f"[{timestamp}] sample={sample_index} node=0x{args.node_id:02X}")
            for line in snapshot.to_lines():
                print(f"  {line}")
            print()

            if args.count > 0 and sample_index >= args.count:
                break
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
