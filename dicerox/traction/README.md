# Traction

This folder contains the current ODrive traction bring-up notes and the ESP32 + MCP2515 sketches used to talk to one ODrive over CAN.

## Contents

- [odrive_6384_hall_first_time_config_notes.txt](./odrive_6384_hall_first_time_config_notes.txt)
  First-time ODrive v3.6 clone + 6384 120KV hall motor notes, including calibration, CAN setup, and velocity unit notes.

- [odrive_6384_hall_pwm_bidirectional_notes.txt](./odrive_6384_hall_pwm_bidirectional_notes.txt)
  Hall-based 6384 bring-up plus bidirectional PWM speed control, using the centered 50 Hz RC convention where `5% = reverse`, `7.5% = stop`, and `10% = forward`.

- [odrive_6384_hall_pwm_bidirectional_dual_axis_notes.txt](./odrive_6384_hall_pwm_bidirectional_dual_axis_notes.txt)
  Dual-axis Hall-based 6384 bring-up plus bidirectional PWM speed control for `axis0` and `axis1`, with one PWM input per axis and centered neutral at `7.5%`.

- [odrive_6384_sensorless_pwm_notes.txt](./odrive_6384_sensorless_pwm_notes.txt)
  Sensorless 6384 bring-up notes for running without Hall sensors, including open-loop startup ramp behavior, PWM speed input mapping, and the main low-speed limitations to expect.

- [esp32_mcp2515_odrive_reader](./esp32_mcp2515_odrive_reader)
  Read-only ESP32 sketch for checking heartbeat, encoder estimates, bus voltage/current, and errors from one ODrive node.

- [esp32_mcp2515_odrive_velocity_controller](./esp32_mcp2515_odrive_velocity_controller)
  ESP32 sketch for velocity control over CAN.

- [esp32_pwm_bidirectional_generator](./esp32_pwm_bidirectional_generator)
  Tiny ESP32 sketch that generates a centered RC-style PWM signal for Hall-based bidirectional speed control, with neutral at `1500 us` / `7.5%` duty on boot.

- [esp32_dual_pwm_bidirectional_generator](./esp32_dual_pwm_bidirectional_generator)
  Tiny ESP32 sketch that generates two centered RC-style PWM signals, one per ODrive axis, for Hall-based bidirectional dual-motor control.

- [esp32_pwm_signal_generator](./esp32_pwm_signal_generator)
  Tiny ESP32 sketch that generates an RC-style PWM pulse on one GPIO for testing ODrive `gpio*_pwm_mapping` without an RC receiver.

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

## PWM signal generator

Open [esp32_pwm_signal_generator.ino](./esp32_pwm_signal_generator/esp32_pwm_signal_generator.ino), flash it to an ESP32, then connect:

- `ESP32 GPIO23 -> ODrive GPIO3`
- `ESP32 GND -> ODrive GND`

Open the serial monitor at `115200` and send:

1. `1000`
2. `1500`
3. `2000`
4. `sweep on`

This sketch outputs a standard RC-style pulse train at `50 Hz`, so it is suitable for testing `gpio3_pwm_mapping` style input on older ODrive firmware.
It boots at `1000 us` by default, so with a `min=0` mapping the ODrive sees `0 turns/s` at startup.
If you later reuse the same ESP32 for the MCP2515 CAN sketches in this folder, remember that `GPIO23` is also the default SPI `MOSI` pin there.

## Bidirectional PWM generator

Open [esp32_pwm_bidirectional_generator.ino](./esp32_pwm_bidirectional_generator/esp32_pwm_bidirectional_generator.ino), flash it to an ESP32, then connect:

- `ESP32 GPIO23 -> ODrive GPIO3`
- `ESP32 GND -> ODrive GND`

Open the serial monitor at `115200` and send:

1. `stop`
2. `fwd`
3. `rev`
4. `1500`
5. `2000`
6. `1000`

This sketch is intended for the Hall-based bidirectional PWM setup. It boots at `1500 us`, which is neutral for the centered `5% / 7.5% / 10%` convention.

## Dual-axis bidirectional PWM generator

Open [esp32_dual_pwm_bidirectional_generator.ino](./esp32_dual_pwm_bidirectional_generator/esp32_dual_pwm_bidirectional_generator.ino), flash it to an ESP32, then connect:

- `ESP32 GPIO23 -> ODrive GPIO3` for `axis0`
- `ESP32 GPIO25 -> ODrive GPIO4` for `axis1`
- `ESP32 GND -> ODrive GND`

Open the serial monitor at `115200` and send:

1. `both stop`
2. `a0 fwd`
3. `a1 rev`
4. `a0 1500`
5. `a1 2000`
6. `status`

This sketch is intended for the dual-axis Hall-based bidirectional PWM setup. It boots both channels at `1500 us`, which is neutral for the centered `5% / 7.5% / 10%` convention.
