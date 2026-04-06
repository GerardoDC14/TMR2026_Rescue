#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time

from odrive_ginkgo import ODriveGinkgoClient


def _node_id(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send a small position test sequence to an ODrive over CAN using the Ginkgo adapter.",
    )
    parser.add_argument(
        "targets",
        nargs="*",
        type=float,
        help="Target positions in turns. If omitted, a small safe sequence is used.",
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
        "--hold",
        type=float,
        default=1.0,
        help="Time to wait after each move command in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.15,
        help="Encoder read timeout in seconds (default: 0.15).",
    )
    parser.add_argument(
        "--initial-target",
        type=float,
        default=0.0,
        help="Target sent before entering closed loop for safer startup (default: 0.0).",
    )
    parser.add_argument(
        "--vel-ff",
        type=int,
        default=0,
        help="Velocity feed-forward in ODrive CAN units (default: 0).",
    )
    parser.add_argument(
        "--torque-ff",
        type=int,
        default=0,
        help="Torque feed-forward in ODrive CAN units (default: 0).",
    )
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="Do not send a final return-to-zero command at the end of the test.",
    )
    parser.add_argument(
        "--leave-closed-loop",
        action="store_true",
        help="Leave the axis in closed loop instead of sending IDLE on exit.",
    )
    parser.add_argument(
        "--clear-errors",
        action="store_true",
        help="Send Clear_Errors before enabling the axis.",
    )
    parser.add_argument(
        "--require-linux-root",
        action="store_true",
        help="Require root on Linux instead of relying on udev permissions.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    targets = args.targets or [0.10, -0.10, 0.00]

    with ODriveGinkgoClient(
        node_id=args.node_id,
        channel=args.channel,
        kbps=args.kbps,
        require_linux_root=args.require_linux_root,
    ) as client:
        if args.clear_errors:
            print("Clearing ODrive errors...")
            client.clear_errors()
            time.sleep(0.05)

        print(f"Priming target to {args.initial_target:.4f} turns before closed loop...")
        client.set_input_pos(args.initial_target, vel_ff=args.vel_ff, torque_ff=args.torque_ff)
        time.sleep(0.05)

        print("Entering closed loop control...")
        client.enter_closed_loop()
        time.sleep(0.10)

        start_pos, start_vel = client.read_encoder_estimates(timeout_s=args.timeout)
        print(f"Initial encoder  : pos={start_pos:.6f} turns, vel={start_vel:.6f} turns/s")

        for index, target in enumerate(targets, start=1):
            print(f"Move {index}/{len(targets)} -> {target:.4f} turns")
            client.set_input_pos(target, vel_ff=args.vel_ff, torque_ff=args.torque_ff)
            time.sleep(args.hold)

            pos, vel = client.read_encoder_estimates(timeout_s=args.timeout)
            print(f"  Encoder after move: pos={pos:.6f} turns, vel={vel:.6f} turns/s")

        if not args.no_return:
            print("Returning to 0.0000 turns...")
            client.set_input_pos(0.0, vel_ff=args.vel_ff, torque_ff=args.torque_ff)
            time.sleep(args.hold)

        if not args.leave_closed_loop:
            print("Setting axis to IDLE...")
            client.idle()


if __name__ == "__main__":
    main()
