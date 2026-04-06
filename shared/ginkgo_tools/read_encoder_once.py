#!/usr/bin/env python3

from __future__ import annotations

import argparse

from odrive_ginkgo import ODriveGinkgoClient


def _node_id(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read one ODrive encoder estimate over CAN using the Ginkgo adapter.",
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
        help="Response timeout in seconds (default: 0.15).",
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
        position_turns, velocity_turns_s = client.read_encoder_estimates(timeout_s=args.timeout)

    print(f"node_id      : 0x{args.node_id:02X} ({args.node_id})")
    print(f"position     : {position_turns:.6f} turns")
    print(f"velocity     : {velocity_turns_s:.6f} turns/s")


if __name__ == "__main__":
    main()
