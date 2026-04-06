from __future__ import annotations

import argparse
import time

from bldc_can_tools.ginkgo_can_interface import GinkgoAdapterConfig, GinkgoCANInterface
from bldc_can_tools.utils import format_can_frame
from bldc_can_tools.ze300_protocol import (
    ZE300_DEFAULT_BITRATE_KBPS,
    ZE300Request,
    build_read_angles_request,
    build_read_fault_state_request,
    build_read_realtime_request,
    build_read_versions_request,
    parse_angles_reply,
    parse_fault_state_reply,
    parse_realtime_reply,
    parse_versions_reply,
)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Basic ZE300 CAN reader using the shared Ginkgo interface abstraction.",
    )
    parser.add_argument("--channel", type=int, default=0, help="Ginkgo CAN channel.")
    parser.add_argument(
        "--bitrate",
        type=int,
        default=ZE300_DEFAULT_BITRATE_KBPS,
        help="CAN bitrate in kbps. ZE300 docs say the default is 1000.",
    )
    parser.add_argument("--device-index", type=int, default=0, help="Ginkgo device index.")
    parser.add_argument(
        "--device-address",
        type=int,
        default=1,
        help="ZE300 device address (Dev_addr).",
    )
    parser.add_argument(
        "--untagged-request",
        action="store_true",
        help="Send requests directly to Dev_addr instead of 0x100|Dev_addr.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.25,
        help="Reply timeout in seconds per command.",
    )
    parser.add_argument(
        "read",
        nargs="?",
        choices=("a0", "a3", "a4", "ae", "all"),
        default="all",
        help="Which ZE300 read command set to run.",
    )
    return parser


def wait_for_reply(interface: GinkgoCANInterface, request: ZE300Request, timeout: float):
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(
                f"Timed out waiting for ZE300 reply cmd=0x{request.command:02X} "
                f"on id=0x{(request.expected_reply_id or 0):03X}."
            )

        frame = interface.recv(timeout=min(remaining, 0.05))
        if frame is None:
            continue

        if request.expected_reply_id is not None and frame.arbitration_id != request.expected_reply_id:
            continue
        if not frame.data or frame.data[0] != request.command:
            continue
        return frame


def print_raw_reply(frame) -> None:
    print(
        format_can_frame(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            extended_id=frame.extended_id,
            remote_frame=frame.remote_frame,
            timestamp_s=time.time(),
        )
    )


def run_one(interface: GinkgoCANInterface, request: ZE300Request, timeout: float, label: str) -> None:
    interface.drain()
    interface.send(request.arbitration_id, request.data)
    frame = wait_for_reply(interface, request, timeout)
    print_raw_reply(frame)

    if request.command == 0xA0:
        decoded = parse_versions_reply(frame.data)
        print(
            f"{label}: boot=0x{decoded.boot_version_raw:04X} "
            f"app=0x{decoded.app_version_raw:04X} "
            f"hw=0x{decoded.hardware_version_raw:04X} "
            f"can_proto=0x{decoded.can_protocol_version_raw:02X}"
        )
    elif request.command == 0xA3:
        decoded = parse_angles_reply(frame.data)
        print(
            f"{label}: single_turn={decoded.single_turn_deg:.2f} deg "
            f"({decoded.single_turn_counts} counts), "
            f"multi_turn={decoded.multi_turn_deg:.2f} deg "
            f"({decoded.multi_turn_counts} counts)"
        )
    elif request.command == 0xA4:
        decoded = parse_realtime_reply(frame.data)
        print(
            f"{label}: temp={decoded.temperature_c} C "
            f"q_current={decoded.q_axis_current_a:.3f} A "
            f"speed={decoded.speed_rpm:.2f} rpm "
            f"single_turn={decoded.single_turn_deg:.2f} deg"
        )
    elif request.command == 0xAE:
        decoded = parse_fault_state_reply(frame.data)
        fault_text = ", ".join(decoded.fault_labels) if decoded.fault_labels else "none"
        print(
            f"{label}: bus_voltage={decoded.bus_voltage_v:.2f} V "
            f"bus_current={decoded.bus_current_a:.2f} A "
            f"temp={decoded.temperature_c} C "
            f"mode={decoded.run_mode_label}({decoded.run_mode}) "
            f"fault=0x{decoded.fault_code:02X} [{fault_text}]"
        )


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    tagged_request = not args.untagged_request
    interface = GinkgoCANInterface(
        GinkgoAdapterConfig(
            channel=args.channel,
            bitrate_kbps=args.bitrate,
            device_index=args.device_index,
        )
    )

    request_builders = {
        "a0": ("versions", build_read_versions_request),
        "a3": ("angles", build_read_angles_request),
        "a4": ("realtime", build_read_realtime_request),
        "ae": ("fault_state", build_read_fault_state_request),
    }

    read_sequence = ["a0", "a3", "a4", "ae"] if args.read == "all" else [args.read]

    try:
        interface.open()
        for key in read_sequence:
            label, builder = request_builders[key]
            request = builder(args.device_address, tagged_request=tagged_request)
            run_one(interface, request, timeout=args.timeout, label=label)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        interface.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
