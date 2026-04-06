from __future__ import annotations

import argparse
from pathlib import Path
import sys


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from bldc_can_tools.ginkgo_can_interface import GinkgoAdapterConfig
    from bldc_can_tools.motor_driver import LKTechMotorDriver
else:
    from .ginkgo_can_interface import GinkgoAdapterConfig
    from .motor_driver import LKTechMotorDriver


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one LKTech position command over CAN.",
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--joint-deg",
        type=float,
        help="Desired joint or output angle in degrees, relative to startup software zero.",
    )
    target_group.add_argument(
        "--motor-deg",
        type=float,
        help="Desired motor-side multi-loop angle in degrees.",
    )

    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel.")
    parser.add_argument("--bitrate", type=int, default=1000, help="CAN bitrate in kbps.")
    parser.add_argument("--device-index", type=int, default=0, help="Ginkgo device index.")
    parser.add_argument("--motor-id", type=int, default=15, help="LKTech motor ID.")
    parser.add_argument(
        "--reduction-ratio",
        type=float,
        default=1.0,
        help="Joint-to-motor reduction ratio.",
    )
    parser.add_argument(
        "--speed-dps-joint",
        type=float,
        default=30.0,
        help="Joint-side speed limit used with --joint-deg.",
    )
    parser.add_argument(
        "--speed-dps-motor",
        type=float,
        default=30.0,
        help="Motor-side speed limit used with --motor-deg.",
    )
    parser.add_argument(
        "--no-boot-offset",
        action="store_true",
        help="Disable startup software zero logic. Joint commands will then be referenced from motor zero.",
    )
    parser.add_argument(
        "--extended",
        action="store_true",
        help="Send frames as extended CAN IDs. Standard IDs are assumed by default.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    use_boot_offset = not args.no_boot_offset
    driver = LKTechMotorDriver(
        adapter_config=GinkgoAdapterConfig(
            channel=args.channel,
            bitrate_kbps=args.bitrate,
            device_index=args.device_index,
        ),
        motor_id=args.motor_id,
        reduction_ratio=args.reduction_ratio,
        use_boot_offset_as_zero=use_boot_offset,
        extended_id=args.extended,
    )

    try:
        try:
            driver.connect(capture_boot_offset=use_boot_offset)
            if args.joint_deg is not None:
                ack = driver.command_joint_angle_deg(
                    desired_joint_deg=args.joint_deg,
                    speed_dps_joint=args.speed_dps_joint,
                )
                print(f"requested_joint_deg={args.joint_deg:.2f}")
                print(f"computed_motor_target_deg={ack.requested_motor_deg:.2f}")
                print(f"speed_limit_motor_dps={ack.speed_limit_motor_dps:.2f}")
            else:
                ack = driver.command_motor_angle_deg(
                    desired_motor_deg=args.motor_deg,
                    speed_dps_motor=args.speed_dps_motor,
                )
                print(f"requested_motor_deg={ack.requested_motor_deg:.2f}")
                print(f"speed_limit_motor_dps={ack.speed_limit_motor_dps:.2f}")

            print(
                "reply="
                f"temp_c={ack.reply.temperature_c}, "
                f"iq_a={ack.reply.torque_current_a:.2f}, "
                f"speed_dps={ack.reply.speed_dps:.1f}, "
                f"angle_deg={ack.reply.angle_deg:.1f}"
            )
            return 0
        except Exception as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
    finally:
        driver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

