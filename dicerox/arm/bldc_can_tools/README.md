# `src/bldc_can_tools`

`src/bldc_can_tools` is a Python `ament_python` package for testing multiple BLDC CAN driver families, including LKTech or LingKong motors and ZE300-style drivers.

The code is intentionally split into:

- a dedicated Ginkgo CAN transport layer
- centralized LKTech protocol helpers
- room for additional driver-family protocol helpers such as ZE300-style drivers
- a high-level motor driver with software zero-offset handling
- direct Python CLIs and a practical CAN sniffer
- optional ROS 2 nodes and launch/config files

This package is written so the direct Python tools are useful on Windows even if you are not using ROS 2 right now.

## Current backend status

This workspace already contains a real Ginkgo vendor ctypes binding and Windows DLL bundle under:

- `src/ginkgo_odrive_bridge/Python_USB_CAN_Test_64bits/ControlCAN.py`
- `src/ginkgo_odrive_bridge/Python_USB_CAN_Test_64bits/lib/windows/...`

The new package uses that real SDK when it is available instead of inventing a fake Ginkgo API. The wrapper in this package still stays isolated and clean, so the rest of the code only depends on `ginkgo_can_interface.py`.

Important:

- Ginkgo backend availability still depends on matching Python architecture and vendor DLL availability.
- The workspace copy I found includes Windows and macOS binaries. Linux libraries were not visible in the checkout I inspected.

## Protocol honesty

The LKTech protocol helpers in this package are based on a MyActuator or LingKong style command family that appears consistent with:

- `0x92` read multi-loop angle
- `0x94` read single-loop angle
- `0x9A` read state 1
- `0x9C` read state 2
- `0x9D` read state 3
- `0xA4` multi-loop or absolute position control with speed limit
- `0xA6` single-loop position control with speed limit

Some LKTech-specific details may still differ by motor firmware, manual revision, or CAN ID mapping. Comments and docstrings call out these assumptions where relevant. You should verify them on real hardware before trusting them for anything safety-critical.

## Build

From the workspace root:

```bash
colcon build --packages-select bldc_can_tools
```

## Source And Run

Linux ROS 2 shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Windows ROS 2 prompt:

```powershell
call install\setup.bat
```

For direct Python usage on Windows, you can also run the tools from inside `src/bldc_can_tools` without building the ROS package first, as long as Python can see the package folder and the Ginkgo vendor SDK can load.

## Windows-First Direct Python Usage

Read the current angle once:

```powershell
python -m bldc_can_tools.cli_read_angle --channel 0 --bitrate 1000 --motor-id 15
```

Read the current angle and treat startup position as software zero:

```powershell
python -m bldc_can_tools.cli_read_angle --channel 0 --bitrate 1000 --motor-id 15 --capture-boot-offset
```

Send a motor-side target directly:

```powershell
python -m bldc_can_tools.cli_position_test --channel 0 --bitrate 1000 --motor-id 15 --motor-deg 180 --speed-dps-motor 90
```

Send a joint-side target that is converted through the reduction ratio and boot offset:

```powershell
python -m bldc_can_tools.cli_position_test --channel 0 --bitrate 1000 --motor-id 15 --reduction-ratio 6 --joint-deg 30 --speed-dps-joint 20
```

## Boot-Offset Logic

This package does not ask the motor to redefine its zero in RAM.

Instead, the high-level driver uses a pure software offset:

1. On startup, read the current multi-loop motor angle.
2. Store that value as `boot_offset_motor_deg`.
3. Define the current joint angle as:

```text
joint_angle_deg = (current_motor_deg - boot_offset_motor_deg) / reduction_ratio
```

4. When commanding a joint target relative to startup zero, convert it back to a motor target:

```text
motor_target_deg = boot_offset_motor_deg + desired_joint_deg * reduction_ratio
```

That means the position at startup becomes software zero.

## Joint Degrees Versus Motor Degrees

- Joint or output degrees are the user-facing quantity.
- Motor-side degrees are what the protocol command uses.
- The conversion uses `reduction_ratio`.
- Speed limits for joint commands are also scaled up internally to motor-side speed before the `0xA4` command is built.

## Monitor Node

If you later want ROS 2 monitoring again:

```bash
ros2 run bldc_can_tools lktech_monitor --ros-args --params-file \
  $(ros2 pkg prefix bldc_can_tools)/share/bldc_can_tools/config/default_params.yaml
```

Published topics:

- `/lktech/current_motor_deg`
- `/lktech/current_joint_deg`
- `/lktech/boot_offset_motor_deg`

Optional:

- `sensor_msgs/msg/JointState` for one joint

## Publish A Test Target Joint Angle

```bash
ros2 topic pub --once /lktech/target_joint_deg std_msgs/msg/Float64 "{data: 15.0}"
```

## Direct CLI Tools

Read angle:

```powershell
python -m bldc_can_tools.cli_read_angle --help
```

Send one target:

```powershell
python -m bldc_can_tools.cli_position_test --help
```

## Run `cansniffer.py`

Directly from the package root:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode raw
```

LKTech-aware decode mode:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode lktech --motor-id 15
```

Log frames to a file:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode lktech --log-file can_log.txt
```

## ESP32 + MCP2515 ODrive Sniffer Example

For a hardware sniffer on ESP32 with an external MCP2515 module, use:

- `src/bldc_can_tools/examples/esp32_mcp2515_odrive_sniffer/esp32_mcp2515_odrive_sniffer.ino`

What it does:

- reuses the same ESP32 SPI wiring pattern as the other MCP2515 examples in this repo
- supports node IDs `0x10`, `0x11`, and `0x12`
- selects one node at a time for interactive commands while keeping per-node runtime state
- uses a fixed CAN bitrate of `1 Mbps`
- uses the same `48:1` gear ratio and `-1.0` direction convention as the Ginkgo bridge
- decodes the most useful ODrive messages for step-by-step debugging:
  heartbeat, `Get_Error`, `Set_Axis_State`, `Get_Encoder_Estimates`, and `Set_Input_Pos`

Default wiring:

- `GPIO23 -> SI`
- `GPIO19 -> SO`
- `GPIO18 -> SCK`
- `GPIO5 -> CS`
- `GPIO4 -> INT`

Default CAN setup:

- bitrate: `1 Mbps`
- MCP2515 oscillator: `8 MHz`
- mode: `normal`
- default active node ID: `0x12`

Useful serial commands at `115200`:

- `p` prints the active configuration
- `1`, `2`, `3` select node `0x10`, `0x11`, or `0x12`
- `b` manually retries automatic bringup for the selected node
- `s` sends a `123#DEADBEEF` self-test frame for MCP2515 TX/RX checks
- `c` sends `Set_Axis_State = CLOSED_LOOP_CONTROL` to the selected node and waits for confirmation
- `x` sends `Set_Axis_State = IDLE` to the selected node
- `e` sends `Get_Encoder_Estimates` RTR to the selected node
- `r` sends `Get_Error` RTR to the selected node
- `z` captures the current encoder estimate as zero and holds that position
- `g <degrees>` sends a relative joint-angle target from the captured zero
- `t <turns>` sends a relative motor-turn target from zero for low-level debugging
- `o` prints the cached motion / heartbeat / error state
- `k` toggles the 20 Hz resend of the last `Set_Input_Pos`
- `v` toggles throttled encoder telemetry
- `a` toggles raw RX debug
- `n`, `i`, `l` switch mode between normal, listen-only, and loopback
- `8` or `6` switch the MCP2515 oscillator assumption between `8 MHz` and `16 MHz`

Default runtime behavior:

- serial output is reduced by default to keep command latency low
- heartbeat and error state are cached so closed-loop transitions can be confirmed
- raw RX printing is off unless `a` is enabled
- encoder telemetry printing is off unless `v` is enabled
- the sketch automatically attempts bringup for `0x10`, `0x11`, and `0x12` on startup
- bringup automatically retries up to 3 times per node
- if you send `g <degrees>` or `t <turns>` and the selected node is not ready, the sketch auto-brings it up first
- the last position target is resent at `20 Hz` by default to behave more like the Ginkgo bridge
- each node keeps its own cached zero / target / heartbeat state when you switch motors
- when streaming is enabled, the sketch keeps resending the last target for every brought-up node

Loopback test:

- send `l` to switch the MCP2515 into loopback mode
- send `s` to transmit the self-test frame
- you should then see `TX`, `RX`, and `MATCH received expected 123#DEADBEEF`

Recommended motion flow:

- reset the board or let the sketch boot normally
- wait for the automatic bringup messages to complete
- send `1`, `2`, or `3` to select the motor you want to move
- send `g 45` or `g -45` to command joint angles from the captured zero
- use `t <turns>` only when you want raw motor-turn debugging

Active ODrive encoder poll:

- leave bitrate at `1 Mbps` to match `baud_rate=1000000`
- use `n` for a real bus test
- select a node with `1`, `2`, or `3`
- send `e` to request encoder estimates from the selected node
- expected request IDs are `0x209` for `0x10`, `0x229` for `0x11`, and `0x249` for `0x12`
- the ODrive should reply on the same ID with an 8-byte payload decoded as `pos_turns` and `vel_turns_s`

Heartbeat and error checks:

- heartbeat IDs are `0x201` for `0x10`, `0x221` for `0x11`, and `0x241` for `0x12`
- `r` requests `Get_Error` for the selected node
- heartbeat state changes and active error changes are printed when they change
- `o` prints the cached axis state, heartbeat errors, and disarm reason on demand

Bus-level debug:

- `a` turns on passthrough mode and prints every standard frame the MCP2515 receives
- this is the quickest way to confirm whether the ESP32 sees any real bus traffic at all

Important note:

- If your bus only has the ODrives and this MCP2515 node, `normal` mode is often needed so the MCP2515 can ACK frames.
- If another active CAN node is already ACKing the bus, `listen-only` is safer for pure sniffing.

## ESP32 + MCP2515 Jaguar 6-Joint Controller

For one ESP32 controlling all six joints on the same CAN bus, use:

- `src/bldc_can_tools/examples/esp32_mcp2515_jaguar_6_joint_controller/esp32_mcp2515_jaguar_6_joint_controller.ino`

What it combines:

- `joint1`, `joint2`, `joint3`: ODrive CANSimple on node IDs `0x10`, `0x11`, `0x12`
- `joint4`: ZE300 on device ID `1`
- `joint5`, `joint6`: LKTech on IDs `14`, `15`
- one shared MCP2515 instance on the same `1 Mbps` bus

Why the bus does not collide:

- ODrive frames live in the `0x200` range, for example `0x209`, `0x229`, `0x249`, `0x20C`
- LKTech requests use `0x140 + motor_id`, so joints `5` and `6` use `0x14E` and `0x14F`
- ZE300 uses request ID `0x101` and reply ID `0x001`

Default behavior:

- ODrive joints `1-3` auto-bring up on startup with retries and keep their `Set_Input_Pos` streaming behavior
- mixed-driver joints `4-6` keep their existing manual `on/zero/goto/speed` workflow
- the sketch keeps the ODrive debug shortcuts and also adds a unified line-based interface for all six joints

Useful commands at `250000`:

- `list`
- `use joint1` through `use joint6`
- `status [TARGET|all]`
- `pos [TARGET|all]`
- `zero [TARGET|all]`
- `on [TARGET|all]`
- `off [TARGET|all]`
- `stop [TARGET|all]`
- `goto TARGET DEG`
- `speed TARGET DPS` for joints `4-6`
- `raw on|off`
- `telemetry on|off`
- `stream on|off`
- `mode normal|listen|loopback`
- `osc 8|16`
- `selftest`

ODrive shortcut compatibility is still there:

- `1`..`6` select active joints
- `b`, `z`, `o`, `a`, `v`, `k`, `s`
- `c`, `x`, `e`, `r` when the active target is `joint1`, `joint2`, or `joint3`
- `g <deg>` for active-target goto
- `t <turns>` for active ODrive motor-turn debug

Recommended flow:

- power the arm with all six motors on the same CAN bus
- let the sketch finish automatic ODrive bringup for joints `1-3`
- run `on joint4`, `on joint5`, `on joint6` after placing those joints at the desired software-zero pose
- then command any joint with `goto jointN DEG`

Latency notes:

- use a serial monitor baud of `250000` with this unified sketch
- the ESP32 ROM boot banner still comes out at `115200`, so the first reset lines can look garbled if the monitor is already set higher
- leave `raw` and `telemetry` off during normal control, and only enable them temporarily for debugging
- `10000000` and `20000000` are not recommended serial-monitor rates for this workflow

## Notes You Must Verify On Hardware

- The exact arbitration ID mapping is assumed to be `0x140 + motor_id` for requests and `0x240 + motor_id` for replies.
- `0xA4` is treated as the correct multi-loop angle control with speed limit for this motor family.
- The `0xA6` single-loop control helper is included as a scaffold and needs hardware confirmation before relying on it.
- The meaning of returned angle fields in status replies may vary slightly across firmware revisions.
- Ginkgo adapter access depends on the vendor SDK loading correctly on your machine.
