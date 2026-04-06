# ESP32 + MCP2515 — Dicerox 6-Joint CAN Controller

Controls six joints across three CAN driver families from a single ESP32 over one shared 1 Mbps bus.

| Joint | Driver | Protocol | CAN IDs |
|-------|--------|----------|---------|
| joint1 | ODrive v3.6 | CANSimple | node `0x10` → arb `0x20x` |
| joint2 | ODrive v3.6 | CANSimple | node `0x11` → arb `0x22x` |
| joint3 | ODrive v3.6 | CANSimple | node `0x12` → arb `0x24x` |
| joint4 | ZE300 | ZE300 positional | req `0x101` / reply `0x001` |
| joint5 | LKTech / MyActuator | 0x140+id | `0x14E` |
| joint6 | LKTech / MyActuator | 0x140+id | `0x14F` |

## Hardware

**ESP32 ↔ MCP2515 SPI wiring:**

| ESP32 GPIO | MCP2515 pin |
|------------|-------------|
| 23 | SI (MOSI) |
| 19 | SO (MISO) |
| 18 | SCK |
| 5  | CS |
| 4  | INT |

CAN bus: 120 Ω termination at each end, 1 Mbps, shared across all six motor controllers.

## Code structure

The sketch is split into purpose-specific headers — the `.ino` is just global state + `setup()` + `loop()`.

```
esp32_mcp2515_dicerox_6_joint_controller.ino   ← globals + setup/loop only
dicerox_globals.h          ← extern declarations for all globals
dicerox_can_bus.h          ← MCP2515 send/receive helpers
dicerox_odrive_control.h   ← ODrive bringup, streaming, position control
dicerox_mixed_control.h    ← ZE300 + LKTech enable/zero/goto
dicerox_joint_control.h    ← unified 6-joint abstraction + status printers
dicerox_serial_ui.h        ← serial command parser and help text
dicerox_mixed_motor_config.h  ← motor config table (gear ratios, IDs, speeds)
```

Shared protocol support lives one directory up in `../esp32_mcp2515_odrive_sniffer/`:
- `odrive_can_support.h` — ODrive CANSimple constants, frame helpers
- `odrive_can_state.h` — per-node runtime state management

## Motor config

Edit `dicerox_mixed_motor_config.h` to change gear ratios, CAN IDs, or default speeds for joints 4–6.
Edit `odrive_can_support.h` constants (`DEFAULT_GEAR_RATIO`, `DEFAULT_DIRECTION`) for joints 1–3.

## Startup sequence

1. ESP32 boots, MCP2515 configured at 1 Mbps / 8 MHz oscillator.
2. Single 10-second shared scan waits for heartbeats from all three ODrives simultaneously.
3. Bringup runs sequentially for each ODrive that responded: `Set_Axis_State → CLOSED_LOOP_CONTROL` → encoder read → software zero.
4. Joints 4–6 are left in a safe idle state; run `on all` after placing them at the desired zero pose.

## Serial interface (115200 baud)

### Motion commands

```
on  [TARGET|all]       enable + capture software zero
off [TARGET|all]       disable / IDLE
goto TARGET DEG        move to angle relative to software zero
speed TARGET DPS       set speed limit for joints 4-6
zero [TARGET|all]      re-capture software zero at current position
pos  [TARGET|all]      read and print current position
stop [TARGET|all]      stop motion
status [TARGET|all]    print cached state
list                   same as status all
use TARGET             set active target (for shortcuts below)
```

`TARGET` is any of: `joint1`…`joint6`, `j1`…`j6`, `ze300`, `lktech14`, `lktech15`, or `1`…`6`.

### Debug commands

```
raw on|off             print every received CAN frame
telemetry on|off       print ODrive encoder estimates at 10 Hz
stream on|off          toggle 20 Hz ODrive position re-send
mode normal|listen|loopback   change MCP2515 bus mode
osc 8|16               change MCP2515 oscillator assumption
selftest               send 123#DEADBEEF for TX/RX verification
```

### Single-key shortcuts (active target)

```
1..6   select joint
b      on (bringup + zero)
z      zero
o      status
g DEG  goto DEG
p      print config
```

ODrive-only (joints 1–3):
```
c   request CLOSED_LOOP_CONTROL
x   request IDLE
e   request encoder estimates
r   request Get_Error
t TURNS   raw motor-turn command (debug)
```

Debug toggles:
```
a   toggle raw RX
v   toggle ODrive telemetry
k   toggle ODrive streaming
s   send selftest frame
```

## Recommended workflow

```
# 1. Power everything, wait for startup banner and automatic bringup to finish.

# 2. Enable joints 4-6 (place them at desired zero pose first):
on joint4
on joint5
on joint6

# 3. Command angles:
goto joint1 45
goto all 0
goto joint4 30

# 4. If an ODrive drops out and recovers, re-enable it:
on joint2
```

## Known limitations

- All six joints share one CAN bus; simultaneous motion is interleaved at the firmware level, not truly parallel.
- ODrive joints use position streaming (20 Hz resend) to hold position; disabling `stream` will let them coast.
- ZE300 speed limit must be re-applied after each power cycle (handled automatically by `on joint4`).
- LKTech motors report multi-turn absolute angle; repositioning the arm while powered off will shift the zero reference.
