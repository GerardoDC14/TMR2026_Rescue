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
        description="Read the current LKTech multi-loop angle once.",
    )
    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel.")
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1000,
        help="CAN bitrate in kbps. Typical LKTech default is 1000.",
    )
    parser.add_argument("--device-index", type=int, default=0, help="Ginkgo device index.")
    parser.add_argument("--motor-id", type=int, default=15, help="LKTech motor ID.")
    parser.add_argument(
        "--reduction-ratio",
        type=float,
        default=1.0,
        help="Joint-to-motor reduction ratio.",
    )
    parser.add_argument(
        "--capture-boot-offset",
        action="store_true",
        help="Capture the startup multi-loop angle and also print the joint angle relative to that software zero.",
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

    driver = LKTechMotorDriver(
        adapter_config=GinkgoAdapterConfig(
            channel=args.channel,
            bitrate_kbps=args.bitrate,
            device_index=args.device_index,
        ),
        motor_id=args.motor_id,
        reduction_ratio=args.reduction_ratio,
        use_boot_offset_as_zero=args.capture_boot_offset,
        extended_id=args.extended,
    )

    try:
        try:
            driver.connect(capture_boot_offset=args.capture_boot_offset)
            motor_angle_deg = driver.read_multi_loop_angle_motor_deg()
            print(f"motor_angle_deg={motor_angle_deg:.2f}")
            if args.capture_boot_offset:
                joint_angle_deg = driver.get_joint_angle_deg()
                print(f"boot_offset_motor_deg={driver.boot_offset_motor_deg:.2f}")
                print(f"joint_angle_deg={joint_angle_deg:.2f}")
            return 0
        except Exception as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
    finally:
        driver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

