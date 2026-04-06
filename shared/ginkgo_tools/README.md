# Shared Ginkgo ODrive Tools

Quick standalone motor test scripts that reuse the existing Ginkgo USB-CAN adapter code already in this repo.

These tools talk to the ODrive over standard 11-bit CANSimple frames through the Ginkgo adapter.

## Files

- `odrive_ginkgo.py`: shared helper used by the scripts and GUI
- `read_encoder_once.py`: read one encoder estimate sample
- `telemetry_monitor.py`: stream encoder, current, temperature, bus, torque, and power telemetry
- `position_step_test.py`: send a small position-control sequence in turns
- `ginkgo_motor_tester.py`: PyQt6 desktop GUI for bench testing
- `odrive_config_GIM8108-48_24V_clean.txt`: cleaned copy of the motor setup notes with CAN settings spelled out

## Defaults

- Ginkgo channel: `0`
- CAN bitrate: `500 kbps`
- ODrive node ID: `0x10`

Override them with CLI flags when needed.

## Examples

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install PyQt6
python3 shared/ginkgo_tools/read_encoder_once.py --node-id 0x10
python3 shared/ginkgo_tools/telemetry_monitor.py --node-id 0x10 --interval 0.5
python3 shared/ginkgo_tools/position_step_test.py --node-id 0x10 0.10 -0.10 0.00
python3 shared/ginkgo_tools/ginkgo_motor_tester.py
```

## Notes

- Start with small position commands.
- `position_step_test.py` works in ODrive turns, not radians.
- The GUI now uses `PyQt6`, so it needs that package installed in your Python environment.
- In this repo, the Ginkgo Linux native libraries live under
  `jaguar/arm/src/ginkgo_odrive_bridge/Python_USB_CAN_Test_64bits/lib/linux/`.
- On Linux, use your udev permissions first. Add `--require-linux-root` only if you still need `sudo`.
- If the vendor folder is moved, set `GINKGO_VENDOR_DIR` before running the scripts.
