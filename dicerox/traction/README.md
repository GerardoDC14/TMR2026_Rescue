# Traction

This folder contains the current ODrive traction bring-up notes and the ESP32 + MCP2515 sketches used to talk to one ODrive over CAN.

## Contents

- [odrive_6384_hall_first_time_config_notes.txt](./odrive_6384_hall_first_time_config_notes.txt)
  First-time ODrive v3.6 clone + 6384 120KV hall motor notes, including calibration, CAN setup, and velocity unit notes.

- [esp32_mcp2515_odrive_reader](./esp32_mcp2515_odrive_reader)
  Read-only ESP32 sketch for checking heartbeat, encoder estimates, bus voltage/current, and errors from one ODrive node.

- [esp32_mcp2515_odrive_velocity_controller](./esp32_mcp2515_odrive_velocity_controller)
  ESP32 sketch for velocity control over CAN.

## Default CAN setup

The sketches in this folder currently assume:

- ODrive node ID: `0x02`
- CAN bitrate: `1 Mbps`
- MCP2515 in normal mode
- MCP2515 crystal: `8 MHz`

If your MCP2515 module uses a `16 MHz` crystal, change the clock selection in the sketch config header before testing.

## ESP32 <-> MCP2515 wiring

Both sketches use the same default wiring:

- `GPIO23 -> MCP2515 SI`
- `GPIO19 -> MCP2515 SO`
- `GPIO18 -> MCP2515 SCK`
- `GPIO5  -> MCP2515 CS`
- `GPIO4  -> MCP2515 INT`

## Velocity controller layout

The velocity controller sketch is split so the common edits are easier:

- [odrive_velocity_config.h](./esp32_mcp2515_odrive_velocity_controller/odrive_velocity_config.h)
  Pins, node ID, bitrate, MCP2515 clock, and timing constants.

- [odrive_velocity_protocol.h](./esp32_mcp2515_odrive_velocity_controller/odrive_velocity_protocol.h)
  ODrive CANSimple command IDs, state enums, frame helpers, and byte packing helpers.

- [odrive_velocity_controller_app.h](./esp32_mcp2515_odrive_velocity_controller/odrive_velocity_controller_app.h)
  The controller app logic: serial commands, heartbeat handling, closed-loop confirmation, telemetry polling, and velocity commands.

- [esp32_mcp2515_odrive_velocity_controller.ino](./esp32_mcp2515_odrive_velocity_controller/esp32_mcp2515_odrive_velocity_controller.ino)
  Thin Arduino entry point that just creates the app and calls `begin()` / `update()`.

## Quick test flow

Open the velocity controller sketch and the serial monitor at `115200`, then use:

1. `on`
2. `vel -5`
3. `stop`
4. `idle`

Useful extra commands:

- `status`
- `raw on`
- `telemetry on`
- `err`

Velocity commands use `turns/s`, which matches `input_vel` on ODrive CANSimple.
