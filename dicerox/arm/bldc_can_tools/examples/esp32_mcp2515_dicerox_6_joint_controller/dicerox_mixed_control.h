#pragma once
#include "dicerox_can_bus.h"

using namespace odrive_can_sniffer;

// ── Request / reply CAN-ID helpers ────────────────────────────────────────────

inline uint16_t mixedRequestId(size_t index) {
  const MixedMotorConfig &config = kMixedMotorConfigs[index];
  if (config.driver_type == DRIVER_LKTECH) {
    return static_cast<uint16_t>(kLKTechStdIdBase + config.device_id);
  }
  if (config.tagged_requests) {
    return static_cast<uint16_t>(kZE300TaggedRequestMask | config.device_id);
  }
  return config.device_id;
}

inline uint16_t mixedReplyId(size_t index) {
  if (kMixedMotorConfigs[index].driver_type == DRIVER_LKTECH) {
    return mixedRequestId(index);
  }
  return kMixedMotorConfigs[index].device_id;
}

// ── Conversion helpers ────────────────────────────────────────────────────────

inline double outputDegreesToCommandDegrees(size_t index, double output_deg) {
  return output_deg * kMixedMotorConfigs[index].position_command_ratio;
}

inline double commandDegreesToOutputDegrees(size_t index, double command_deg) {
  return command_deg / kMixedMotorConfigs[index].position_command_ratio;
}

inline double absoluteOutputToCommandDegrees(size_t index, double output_deg) {
  return g_mixed_states[index].zero_offset_command_deg + outputDegreesToCommandDegrees(index, output_deg);
}

inline double currentMixedOutputDegrees(size_t index) {
  return commandDegreesToOutputDegrees(
      index, g_mixed_states[index].last_command_deg - g_mixed_states[index].zero_offset_command_deg);
}

inline double outputSpeedToCommandDps(size_t index, double output_dps) {
  return output_dps * kMixedMotorConfigs[index].speed_command_ratio;
}

inline uint16_t lkOutputSpeedToMotorDps(size_t index, double output_dps) {
  const double motor_dps = outputSpeedToCommandDps(index, output_dps);
  if (motor_dps <= 0.0) {
    return 0;
  }
  if (motor_dps >= 65535.0) {
    return 65535;
  }
  return static_cast<uint16_t>(motor_dps + 0.5);
}

inline uint32_t ze300OutputSpeedToMotorCentiRpm(size_t index, double output_dps) {
  const double command_dps = outputSpeedToCommandDps(index, output_dps);
  const double motor_rpm   = command_dps / 6.0;
  const double scaled      = motor_rpm * 100.0;
  if (scaled <= 0.0) {
    return 0;
  }
  if (scaled >= 2147483647.0) {
    return 2147483647UL;
  }
  return static_cast<uint32_t>(scaled + 0.5);
}

inline double ze300MotorCentiRpmToOutputDps(size_t index, uint32_t motor_centirpm) {
  const double motor_rpm   = static_cast<double>(motor_centirpm) / 100.0;
  const double command_dps = motor_rpm * 6.0;
  return command_dps / kMixedMotorConfigs[index].speed_command_ratio;
}

inline double ze300CountsToCommandDegrees(int32_t counts) {
  return static_cast<double>(counts) * 360.0 / static_cast<double>(kZE300CountsPerRev);
}

inline int32_t ze300CommandDegreesToCounts(double command_deg) {
  const double scaled = command_deg * static_cast<double>(kZE300CountsPerRev) / 360.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

inline int32_t degreesToCentiDegrees(double degrees) {
  const double scaled = degrees * 100.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

inline double centiDegreesToDegrees(int64_t centi_degrees) {
  return static_cast<double>(centi_degrees) / 100.0;
}

// ── Byte-parsing helpers ──────────────────────────────────────────────────────

inline uint16_t readU16LE(const uint8_t *data, uint8_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}

inline int32_t readI32LE(const uint8_t *data, uint8_t offset) {
  uint32_t raw = static_cast<uint32_t>(data[offset]) |
                 (static_cast<uint32_t>(data[offset + 1]) << 8) |
                 (static_cast<uint32_t>(data[offset + 2]) << 16) |
                 (static_cast<uint32_t>(data[offset + 3]) << 24);
  return static_cast<int32_t>(raw);
}

inline int64_t readSigned56LE(const uint8_t *data) {
  uint64_t raw = 0;
  for (int index = 0; index < 7; ++index) {
    raw |= static_cast<uint64_t>(data[index]) << (8 * index);
  }
  if (data[6] & 0x80) {
    raw |= 0xFF00000000000000ULL;
  }
  return static_cast<int64_t>(raw);
}

// ── Blocking reply waiter ─────────────────────────────────────────────────────

inline bool waitForMixedReply(size_t index, uint8_t expected_command, can_frame *reply_frame) {
  const unsigned long start_ms = millis();
  can_frame frame = {};

  while (millis() - start_ms < REPLY_TIMEOUT_MS) {
    // Drain all available frames each iteration to avoid MCP2515 RX buffer overflow
    while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      const uint32_t arbitration_id  = frame.can_id & CAN_SFF_MASK;
      const bool     matching_reply  = arbitration_id == mixedReplyId(index) &&
                                       frame.can_dlc >= 1 &&
                                       frame.data[0] == expected_command;
      if (matching_reply || g_show_all_frames) {
        printCanFrame(frame, matching_reply ? "RX" : "RX(other)");
      }
      handleReceivedCanFrame(frame, true);

      if (arbitration_id == mixedReplyId(index) &&
          frame.can_dlc >= 1 &&
          frame.data[0] == expected_command) {
        if (reply_frame != nullptr) {
          *reply_frame = frame;
        }
        return true;
      }
    }

    delay(2);
  }

  Serial.printf("timeout waiting for reply cmd=0x%02X on id=0x%03X (%s)\n",
                static_cast<unsigned>(expected_command),
                mixedReplyId(index),
                kMixedMotorConfigs[index].joint_name);
  return false;
}

// ── Generic command senders ───────────────────────────────────────────────────

inline bool sendMixedSimpleCommandAndWait(size_t index, uint8_t command, can_frame *reply_frame = nullptr) {
  if (!ensureCanNormalMode("mixed command")) {
    return false;
  }

  g_pause_odrive_stream_until_ms = millis() + 60;
  drainReceiveQueue();
  const uint8_t payload_lk[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t payload_ze[1] = {command};

  if (kMixedMotorConfigs[index].driver_type == DRIVER_LKTECH) {
    if (!sendStandardFrame(mixedRequestId(index), payload_lk, sizeof(payload_lk))) {
      return false;
    }
  } else {
    if (!sendStandardFrame(mixedRequestId(index), payload_ze, sizeof(payload_ze))) {
      return false;
    }
  }

  return waitForMixedReply(index, command, reply_frame);
}

inline bool sendMixedValueCommandAndWait(size_t index, uint8_t command, int32_t value, can_frame *reply_frame = nullptr) {
  if (!ensureCanNormalMode("mixed command")) {
    return false;
  }

  g_pause_odrive_stream_until_ms = millis() + 60;
  drainReceiveQueue();
  uint8_t payload[8] = {
      command,
      static_cast<uint8_t>(value         & 0xFF),
      static_cast<uint8_t>((value >>  8)  & 0xFF),
      static_cast<uint8_t>((value >> 16)  & 0xFF),
      static_cast<uint8_t>((value >> 24)  & 0xFF),
      0x00,
      0x00,
      0x00,
  };

  const uint8_t dlc = kMixedMotorConfigs[index].driver_type == DRIVER_LKTECH ? 8 : 5;
  if (!sendStandardFrame(mixedRequestId(index), payload, dlc)) {
    return false;
  }

  return waitForMixedReply(index, command, reply_frame);
}

// ── Position / speed API ──────────────────────────────────────────────────────

inline bool readMixedCurrentPosition(size_t index) {
  const MixedMotorConfig    &config = kMixedMotorConfigs[index];
  MixedMotorRuntimeState    &state  = g_mixed_states[index];
  can_frame reply = {};

  if (config.driver_type == DRIVER_LKTECH) {
    if (!sendMixedSimpleCommandAndWait(index, LK_CMD_READ_MULTI_LOOP_ANGLE, &reply)) {
      return false;
    }
    const int64_t angle_cdeg = readSigned56LE(&reply.data[1]);
    state.last_command_deg = centiDegreesToDegrees(angle_cdeg);
    Serial.printf("%s LKTech motor angle = %.2f deg\n", config.joint_name, state.last_command_deg);
  } else {
    if (!sendMixedSimpleCommandAndWait(index, ZE_CMD_READ_ABSOLUTE_ANGLES, &reply)) {
      return false;
    }
    if (reply.can_dlc != 7) {
      Serial.printf("%s unexpected A3 reply length: %u\n",
                    config.joint_name,
                    static_cast<unsigned>(reply.can_dlc));
      return false;
    }
    const uint16_t single_turn_counts = readU16LE(reply.data, 1);
    const int32_t  multi_turn_counts  = readI32LE(reply.data, 3);
    state.last_native_counts = multi_turn_counts;
    state.last_command_deg   = ze300CountsToCommandDegrees(multi_turn_counts);
    Serial.printf("%s ZE300 single=%.2f deg (%u counts) multi=%.2f deg (%ld counts)\n",
                  config.joint_name,
                  ze300CountsToCommandDegrees(single_turn_counts),
                  static_cast<unsigned>(single_turn_counts),
                  state.last_command_deg,
                  static_cast<long>(multi_turn_counts));
  }

  if (state.have_zero_offset) {
    Serial.printf("%s current output angle = %.2f deg relative to software zero\n",
                  config.joint_name,
                  currentMixedOutputDegrees(index));
  }
  return true;
}

inline bool captureMixedZeroOffset(size_t index) {
  if (!readMixedCurrentPosition(index)) {
    return false;
  }

  MixedMotorRuntimeState &state  = g_mixed_states[index];
  state.zero_offset_command_deg  = state.last_command_deg;
  state.have_zero_offset         = true;
  Serial.printf("%s software zero captured at command_deg=%.2f, output zero is now 0.00 deg\n",
                kMixedMotorConfigs[index].joint_name,
                state.zero_offset_command_deg);
  return true;
}

inline bool applyMixedOutputSpeed(size_t index) {
  const MixedMotorConfig    &config = kMixedMotorConfigs[index];
  MixedMotorRuntimeState    &state  = g_mixed_states[index];

  if (config.driver_type == DRIVER_LKTECH) {
    Serial.printf("%s LKTech output speed stored as %.2f dps -> motor speed %.2f dps\n",
                  config.joint_name,
                  state.output_speed_dps,
                  outputSpeedToCommandDps(index, state.output_speed_dps));
    return true;
  }

  can_frame reply = {};
  const uint32_t motor_centirpm = ze300OutputSpeedToMotorCentiRpm(index, state.output_speed_dps);
  if (!sendMixedValueCommandAndWait(index, ZE_CMD_SET_POSITION_MAX_SPEED,
                                    static_cast<int32_t>(motor_centirpm), &reply)) {
    return false;
  }

  if (reply.can_dlc != 5) {
    Serial.printf("%s unexpected B2 reply length: %u\n",
                  config.joint_name,
                  static_cast<unsigned>(reply.can_dlc));
    return false;
  }

  const uint32_t applied_motor_centirpm = static_cast<uint32_t>(readI32LE(reply.data, 1));
  state.ze300_speed_applied = true;
  Serial.printf("%s ZE300 speed applied: output=%.2f dps command=%.2f dps motor=%.2f rpm\n",
                config.joint_name,
                state.output_speed_dps,
                outputSpeedToCommandDps(index, state.output_speed_dps),
                static_cast<double>(applied_motor_centirpm) / 100.0);
  return true;
}

inline bool setMixedOutputSpeed(size_t index, double output_dps) {
  if (output_dps < 0.0) {
    Serial.println("speed must be non-negative");
    return false;
  }
  g_mixed_states[index].output_speed_dps = output_dps;
  return applyMixedOutputSpeed(index);
}

// ── Enable / disable / stop ───────────────────────────────────────────────────

inline bool sendMixedOn(size_t index) {
  const MixedMotorConfig &config = kMixedMotorConfigs[index];
  MixedMotorRuntimeState &state  = g_mixed_states[index];

  if (config.driver_type == DRIVER_LKTECH) {
    for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
      can_frame reply = {};
      if (!sendMixedSimpleCommandAndWait(index, LK_CMD_MOTOR_ON, &reply)) {
        Serial.printf("%s motor on attempt %u/3 timed out, retrying...\n",
                      config.joint_name, static_cast<unsigned>(attempt));
        delay(300);
        continue;
      }
      state.enabled = true;
      Serial.printf("%s motor on acknowledged\n", config.joint_name);
      delay(100);  // let motor controller settle before querying angle
      if (captureMixedZeroOffset(index)) {
        return true;
      }
      Serial.printf("%s angle read failed after motor on, retrying...\n", config.joint_name);
      delay(300);
    }
    return false;
  }

  for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
    if (!applyMixedOutputSpeed(index)) {
      Serial.printf("%s ZE300 speed apply attempt %u/3 failed, retrying...\n",
                    config.joint_name, static_cast<unsigned>(attempt));
      delay(300);
      continue;
    }
    state.enabled = true;
    Serial.printf("%s ZE300 ready: speed applied, capturing software zero next\n", config.joint_name);
    delay(20);
    if (captureMixedZeroOffset(index)) {
      return true;
    }
    delay(300);
  }
  return false;
}

inline bool sendMixedOff(size_t index) {
  MixedMotorRuntimeState &state  = g_mixed_states[index];
  const MixedMotorConfig &config = kMixedMotorConfigs[index];
  can_frame reply = {};

  if (config.driver_type == DRIVER_LKTECH) {
    if (!sendMixedSimpleCommandAndWait(index, LK_CMD_MOTOR_OFF, &reply)) {
      return false;
    }
  } else {
    if (!sendMixedSimpleCommandAndWait(index, ZE_CMD_DISABLE_OUTPUT, &reply)) {
      return false;
    }
    state.ze300_speed_applied = false;
  }

  state.enabled = false;
  Serial.printf("%s output disabled\n", config.joint_name);
  return true;
}

inline bool sendMixedStop(size_t index) {
  const MixedMotorConfig &config = kMixedMotorConfigs[index];
  if (config.driver_type == DRIVER_LKTECH) {
    can_frame reply = {};
    if (!sendMixedSimpleCommandAndWait(index, LK_CMD_MOTOR_STOP, &reply)) {
      return false;
    }
    Serial.printf("%s stop acknowledged\n", config.joint_name);
    return true;
  }
  Serial.printf("%s ZE300 stop is mapped to output disable (0xCF)\n", config.joint_name);
  return sendMixedOff(index);
}

// ── Absolute position command ─────────────────────────────────────────────────

inline bool commandMixedAbsoluteOutput(size_t index, double target_output_deg) {
  const MixedMotorConfig    &config = kMixedMotorConfigs[index];
  const MixedMotorRuntimeState &state = g_mixed_states[index];

  if (!state.have_zero_offset) {
    Serial.printf("%s software zero not captured yet, send `on %s` or `zero %s` first\n",
                  config.joint_name,
                  config.joint_name,
                  config.joint_name);
    return false;
  }

  const double target_command_deg = absoluteOutputToCommandDegrees(index, target_output_deg);

  if (config.driver_type == DRIVER_LKTECH) {
    const uint16_t motor_speed_dps = lkOutputSpeedToMotorDps(index, g_mixed_states[index].output_speed_dps);
    const int32_t  target_cdeg     = degreesToCentiDegrees(target_command_deg);
    const uint8_t payload[8] = {
        LK_CMD_MULTI_LOOP_CONTROL_2,
        0x00,
        static_cast<uint8_t>(motor_speed_dps         & 0xFF),
        static_cast<uint8_t>((motor_speed_dps >> 8)  & 0xFF),
        static_cast<uint8_t>(target_cdeg              & 0xFF),
        static_cast<uint8_t>((target_cdeg  >>  8)    & 0xFF),
        static_cast<uint8_t>((target_cdeg  >> 16)    & 0xFF),
        static_cast<uint8_t>((target_cdeg  >> 24)    & 0xFF),
    };

    if (!sendStandardFrame(mixedRequestId(index), payload, sizeof(payload))) {
      return false;
    }
    g_mixed_states[index].last_command_deg = target_command_deg;
    Serial.printf("%s goto output=%.2f deg -> command=%.2f deg speed=%.2f output dps\n",
                  config.joint_name,
                  target_output_deg,
                  target_command_deg,
                  g_mixed_states[index].output_speed_dps);
    return true;
  }

  if (!g_mixed_states[index].ze300_speed_applied) {
    if (!applyMixedOutputSpeed(index)) {
      return false;
    }
  }

  const int32_t target_counts = ze300CommandDegreesToCounts(target_command_deg);
  can_frame reply = {};
  if (!sendMixedValueCommandAndWait(index, ZE_CMD_ABSOLUTE_POSITION, target_counts, &reply)) {
    return false;
  }
  g_mixed_states[index].last_command_deg = target_command_deg;
  if (reply.can_dlc == 7) {
    g_mixed_states[index].last_native_counts = readI32LE(reply.data, 3);
    g_mixed_states[index].last_command_deg   = ze300CountsToCommandDegrees(g_mixed_states[index].last_native_counts);
  }

  Serial.printf("%s goto output=%.2f deg -> command=%.2f deg -> counts=%ld speed=%.2f output dps\n",
                config.joint_name,
                target_output_deg,
                target_command_deg,
                static_cast<long>(target_counts),
                g_mixed_states[index].output_speed_dps);
  return true;
}
