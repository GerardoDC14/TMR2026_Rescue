#pragma once
#include "dicerox_can_bus.h"

using namespace odrive_can_sniffer;

// ── Accessor helpers ──────────────────────────────────────────────────────────

inline const UnifiedMotorTarget *targetForOdriveNode(uint8_t node_id) {
  for (size_t index = 0; index < kUnifiedMotorCount; ++index) {
    const UnifiedMotorTarget &target = kUnifiedTargets[index];
    if (target.driver_type == UNIFIED_DRIVER_ODRIVE && target.odrive_node_id == node_id) {
      return &target;
    }
  }
  return nullptr;
}

inline NodeRuntimeState *odriveStateForNode(uint8_t node_id) {
  return findNodeState(g_odrive_states, SUPPORTED_NODE_COUNT, node_id);
}

// ── Decode helpers ────────────────────────────────────────────────────────────

inline void decodeOdriveHeartbeat(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 5) {
    return;
  }

  const uint32_t active_errors = bytesToU32(frame.data);
  const uint8_t  axis_state    = frame.data[4];
  const bool changed = !state.haveHeartbeat ||
                       active_errors != state.lastHeartbeatActiveErrors ||
                       axis_state    != state.lastAxisState;

  state.haveHeartbeat              = true;
  state.lastHeartbeatActiveErrors  = active_errors;
  state.lastAxisState              = axis_state;
  state.lastHeartbeatMs            = millis();

  const UnifiedMotorTarget *target  = targetForOdriveNode(state.nodeId);
  const bool highlight = target != nullptr && target == &kUnifiedTargets[g_active_target_index];
  if (changed && highlight) {
    Serial.printf("%s heartbeat state=%s active_errors=0x%lX\n",
                  target->name,
                  axisStateName(axis_state),
                  static_cast<unsigned long>(active_errors));
  }
}

inline void decodeOdriveGetError(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    return;
  }

  const uint32_t active_errors  = bytesToU32(frame.data);
  const uint32_t disarm_reason  = bytesToU32(frame.data + 4);
  const bool changed = !state.haveErrorStatus ||
                       active_errors != state.lastErrorActiveErrors ||
                       disarm_reason != state.lastDisarmReason;

  state.haveErrorStatus         = true;
  state.lastErrorActiveErrors   = active_errors;
  state.lastDisarmReason        = disarm_reason;
  state.lastErrorMs             = millis();

  const UnifiedMotorTarget *target  = targetForOdriveNode(state.nodeId);
  const bool highlight = target != nullptr && target == &kUnifiedTargets[g_active_target_index];
  if (highlight && (changed || g_show_all_frames)) {
    Serial.printf("%s odrive active_errors=0x%lX disarm_reason=0x%lX\n",
                  target->name,
                  static_cast<unsigned long>(active_errors),
                  static_cast<unsigned long>(disarm_reason));
  }
}

inline void decodeOdriveEncoderEstimates(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    return;
  }

  const float pos_turns   = bytesToFloat(frame.data);
  const float vel_turns_s = bytesToFloat(frame.data + 4);
  state.lastEncoderPosTurns              = pos_turns;
  state.lastEncoderVelTurnsPerSecond     = vel_turns_s;
  state.haveLatestEncoderEstimate        = true;
  state.lastEncoderEstimateMs            = millis();

  const UnifiedMotorTarget *target = targetForOdriveNode(state.nodeId);
  const bool highlight = target != nullptr && target == &kUnifiedTargets[g_active_target_index];
  if (highlight &&
      g_print_odrive_telemetry &&
      (state.lastEncoderTelemetryPrintMs == 0 ||
       state.lastEncoderEstimateMs - state.lastEncoderTelemetryPrintMs >= ENCODER_TELEMETRY_INTERVAL_MS)) {
    state.lastEncoderTelemetryPrintMs = state.lastEncoderEstimateMs;
    Serial.printf("%s odrive pos_turns=%.6f vel_turns_s=%.6f\n",
                  target->name,
                  pos_turns,
                  vel_turns_s);
  }
}

// ── Frame dispatch (satisfies the forward-decl in dicerox_can_bus.h) ───────────

inline void handleReceivedCanFrame(const can_frame &frame, bool already_printed) {
  if (!already_printed && g_show_all_frames) {
    printCanFrame(frame, "RX");
  }

  if (isExtendedFrame(frame) || isRemoteFrame(frame)) {
    return;
  }

  const uint8_t       node_id = extractNodeId(getStandardId(frame));
  NodeRuntimeState   *state   = odriveStateForNode(node_id);
  if (state == nullptr) {
    return;
  }

  switch (extractCommandId(getStandardId(frame))) {
    case CMD_HEARTBEAT:
      decodeOdriveHeartbeat(*state, frame);
      break;
    case CMD_GET_ERROR:
      decodeOdriveGetError(*state, frame);
      break;
    case CMD_GET_ENCODER_ESTIMATES:
      decodeOdriveEncoderEstimates(*state, frame);
      break;
    default:
      break;
  }
}

inline void processIncomingFrames() {
  uint16_t frame_count = 0;
  while (digitalRead(PIN_CAN_INT) == LOW && frame_count < MAX_RX_FRAMES_PER_PASS) {
    if (g_mcp2515.readMessage(&g_rx_frame) != MCP2515::ERROR_OK) {
      break;
    }
    handleReceivedCanFrame(g_rx_frame, false);
    ++frame_count;
  }
}

// ── ODrive CAN command senders ────────────────────────────────────────────────

inline bool sendOdriveAxisStateRequest(uint8_t node_id, uint32_t axis_state, const char *message, bool log_frame) {
  can_frame frame = {};
  frame.can_id    = makeCanId(node_id, CMD_SET_AXIS_STATE);
  frame.can_dlc   = 4;
  frame.data[0]   = static_cast<uint8_t>(axis_state         & 0xFF);
  frame.data[1]   = static_cast<uint8_t>((axis_state >>  8) & 0xFF);
  frame.data[2]   = static_cast<uint8_t>((axis_state >> 16) & 0xFF);
  frame.data[3]   = static_cast<uint8_t>((axis_state >> 24) & 0xFF);
  return sendCanFrame(frame, message, log_frame);
}

inline bool sendOdriveRemoteRequest(uint8_t node_id, uint8_t command, uint8_t dlc, const char *message, bool log_frame) {
  can_frame frame = {};
  frame.can_id  = makeCanId(node_id, command) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  return sendCanFrame(frame, message, log_frame);
}

inline bool sendOdriveInputPositionToNode(uint8_t node_id,
                                          float   absolute_turns,
                                          int16_t vel_feedforward,
                                          int16_t torque_feedforward,
                                          const char *message,
                                          bool log_frame) {
  can_frame frame = {};
  frame.can_id  = makeCanId(node_id, CMD_SET_INPUT_POS);
  frame.can_dlc = 8;
  writeFloatToBytes(absolute_turns,   &frame.data[0]);
  writeI16ToBytes(vel_feedforward,    &frame.data[4]);
  writeI16ToBytes(torque_feedforward, &frame.data[6]);
  return sendCanFrame(frame, message, log_frame);
}

inline void serviceOdriveTargetStreaming() {
  if (!g_stream_odrive_targets || g_mode != CanMode::Normal) {
    return;
  }

  const unsigned long now = millis();
  if (now < g_pause_odrive_stream_until_ms) {
    return;
  }

  for (size_t index = 0; index < SUPPORTED_NODE_COUNT; ++index) {
    NodeRuntimeState &state = g_odrive_states[index];
    if (!state.haveActiveTarget) {
      continue;
    }
    if (now - state.lastTargetStreamMs < TARGET_STREAM_INTERVAL_MS) {
      continue;
    }

    state.lastTargetStreamMs = now;
    sendOdriveInputPositionToNode(state.nodeId, state.lastAbsoluteCommandTurns, 0, 0, nullptr, false);
  }
}

// ── Heartbeat / encoder freshness checks ─────────────────────────────────────

inline bool odriveHeartbeatFresh(const NodeRuntimeState &state) {
  return state.haveHeartbeat &&
         millis() - state.lastHeartbeatMs <= ODRIVE_HEARTBEAT_STALE_MS;
}

inline bool odriveEncoderFresh(const NodeRuntimeState &state) {
  return state.haveLatestEncoderEstimate &&
         millis() - state.lastEncoderEstimateMs <= ODRIVE_ENCODER_STALE_MS;
}

inline bool odriveNodeInUnifiedClosedLoop(const NodeRuntimeState &state) {
  return odriveHeartbeatFresh(state) &&
         state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
         state.lastHeartbeatActiveErrors == 0;
}

inline bool odriveNodeReadyForUnifiedMotion(const NodeRuntimeState &state) {
  return odriveNodeInUnifiedClosedLoop(state) &&
         odriveEncoderFresh(state) &&
         state.haveZeroReference;
}

// ── Blocking wait helpers ─────────────────────────────────────────────────────

inline bool waitForOdriveHeartbeat(uint8_t node_id, uint32_t timeout_ms) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }
  if (state->haveHeartbeat && millis() - state->lastHeartbeatMs < 2000) {
    return true;
  }

  const unsigned long start_ms = millis();
  while (millis() - start_ms < timeout_ms) {
    processIncomingFrames();
    if (state->haveHeartbeat && state->lastHeartbeatMs >= start_ms) {
      return true;
    }
    delay(10);
  }

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.printf("%s no heartbeat within %lums, skipping bringup\n",
                target != nullptr ? target->name : "odrive",
                static_cast<unsigned long>(timeout_ms));
  return false;
}

inline bool waitForOdriveEncoderEstimate(uint8_t node_id, uint32_t timeout_ms) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }

  // Always request a fresh estimate — never trust a cached value that may have
  // arrived during startup (e.g. 0.0 from a just-booted ODrive) before closed-loop
  // was confirmed. Require the reply timestamp to be >= start_ms.
  sendOdriveRemoteRequest(node_id, CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates", true);
  const unsigned long start_ms     = millis();
  unsigned long       last_retry_ms = start_ms;

  while (millis() - start_ms < timeout_ms) {
    processIncomingFrames();
    serviceOdriveTargetStreaming();

    if (state->haveLatestEncoderEstimate && state->lastEncoderEstimateMs >= start_ms) {
      return true;
    }

    if (millis() - last_retry_ms >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      last_retry_ms = millis();
      sendOdriveRemoteRequest(node_id, CMD_GET_ENCODER_ESTIMATES, 8, nullptr, false);
    }

    delay(1);
  }

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.printf("%s timed out waiting for encoder estimate.\n",
                target != nullptr ? target->name : "odrive");
  return false;
}

inline bool requestOdriveClosedLoopAndConfirm(uint8_t node_id, uint32_t timeout_ms) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }

  if (state->haveHeartbeat &&
      millis() - state->lastHeartbeatMs < 300 &&
      state->lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
      state->lastHeartbeatActiveErrors == 0) {
    const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
    Serial.printf("%s already in closed loop (fresh heartbeat), skipping re-request.\n",
                  target != nullptr ? target->name : "odrive");
    return true;
  }

  sendOdriveAxisStateRequest(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, "Requested CLOSED_LOOP_CONTROL", true);

  const unsigned long start_ms     = millis();
  unsigned long       last_retry_ms = start_ms;

  while (millis() - start_ms < timeout_ms) {
    processIncomingFrames();
    serviceOdriveTargetStreaming();

    if (state->haveHeartbeat &&
        state->lastHeartbeatMs >= start_ms &&
        state->lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
        state->lastHeartbeatActiveErrors == 0) {
      const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
      Serial.printf("%s closed loop confirmed.\n", target != nullptr ? target->name : "odrive");
      return true;
    }

    if (millis() - last_retry_ms >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      last_retry_ms = millis();
      sendOdriveAxisStateRequest(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, nullptr, false);
    }

    delay(1);
  }

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.printf("%s closed loop not confirmed.\n", target != nullptr ? target->name : "odrive");
  sendOdriveRemoteRequest(node_id, CMD_GET_ERROR, 8, "Requested Get_Error after closed-loop timeout", true);
  return false;
}

inline bool captureOdriveZero(uint8_t node_id) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }
  if (!state->haveLatestEncoderEstimate) {
    const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
    Serial.printf("%s has no encoder estimate yet.\n", target != nullptr ? target->name : "odrive");
    return false;
  }

  state->zeroReferenceTurns         = state->lastEncoderPosTurns;
  state->haveZeroReference          = true;
  state->haveActiveTarget           = true;
  state->lastRelativeCommandTurns   = 0.0f;
  state->lastRelativeCommandDegrees = 0.0f;
  state->lastAbsoluteCommandTurns   = state->zeroReferenceTurns;
  state->lastTargetStreamMs         = millis();

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.printf("%s zero captured at %.6f turns\n",
                target != nullptr ? target->name : "odrive",
                state->zeroReferenceTurns);

  if (g_mode == CanMode::Normal) {
    sendOdriveInputPositionToNode(node_id, state->lastAbsoluteCommandTurns, 0, 0, "Holding current position as zero reference", true);
  }
  return true;
}

// ── Bringup sequence ──────────────────────────────────────────────────────────

inline bool runOdriveBringupAttempt(uint8_t node_id) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }
  state->haveZeroReference          = false;
  state->haveActiveTarget           = false;
  state->lastRelativeCommandTurns   = 0.0f;
  state->lastRelativeCommandDegrees = 0.0f;

  if (!requestOdriveClosedLoopAndConfirm(node_id, CLOSED_LOOP_CONFIRM_TIMEOUT_MS)) {
    return false;
  }
  if (!waitForOdriveEncoderEstimate(node_id, ENCODER_WAIT_TIMEOUT_MS)) {
    return false;
  }
  if (!captureOdriveZero(node_id)) {
    return false;
  }
  return odriveNodeReadyForUnifiedMotion(*state);
}

inline bool runOdriveBringupWithRetries(uint8_t node_id, const char *reason) {
  if (!ensureCanNormalMode("odrive bringup")) {
    return false;
  }

  if (!waitForOdriveHeartbeat(node_id, 3000)) {
    return false;
  }

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.print("ODrive auto bringup ");
  Serial.print(target != nullptr ? target->name : "odrive");
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" trigger=");
    Serial.print(reason);
  }
  Serial.println();

  for (uint8_t attempt = 1; attempt <= BRINGUP_MAX_ATTEMPTS; ++attempt) {
    Serial.printf("bringup attempt %u/%u\n",
                  static_cast<unsigned>(attempt),
                  static_cast<unsigned>(BRINGUP_MAX_ATTEMPTS));
    if (runOdriveBringupAttempt(node_id)) {
      Serial.printf("%s bringup complete.\n", target != nullptr ? target->name : "odrive");
      return true;
    }
    sendOdriveAxisStateRequest(node_id, AXIS_STATE_IDLE, nullptr, false);
    delay(BRINGUP_RETRY_SETTLE_MS);
  }

  Serial.printf("%s bringup failed after %u attempts.\n",
                target != nullptr ? target->name : "odrive",
                static_cast<unsigned>(BRINGUP_MAX_ATTEMPTS));
  return false;
}

inline bool ensureOdriveReadyForMotion(uint8_t node_id, const char *reason) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state != nullptr && g_mode == CanMode::Normal && odriveNodeReadyForUnifiedMotion(*state)) {
    return true;
  }
  return runOdriveBringupWithRetries(node_id, reason);
}

inline void runAutomaticOdriveBringupAllNodes() {
  Serial.println("Automatic ODrive bringup starting for joint1, joint2, joint3");

  // Single shared window so all nodes accumulate heartbeats together.
  // Avoids 3x3s sequential waits when ODrives boot slower than the ESP32.
  Serial.println("Scanning for ODrive heartbeats (up to 10s)...");
  const unsigned long scan_start_ms = millis();
  while (millis() - scan_start_ms < 10000) {
    processIncomingFrames();
    bool all_heard = true;
    for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
      if (!g_odrive_states[i].haveHeartbeat) {
        all_heard = false;
        break;
      }
    }
    if (all_heard) {
      Serial.printf("All ODrive heartbeats received (%lums).\n",
                    static_cast<unsigned long>(millis() - scan_start_ms));
      break;
    }
    delay(10);
  }

  for (size_t index = 0; index < SUPPORTED_NODE_COUNT; ++index) {
    runOdriveBringupWithRetries(g_odrive_states[index].nodeId, "startup");
  }
  Serial.println("Automatic ODrive bringup finished.");
}

// ── High-level ODrive motion API ──────────────────────────────────────────────

inline bool readOdriveCurrentPosition(uint8_t node_id) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }

  if (!odriveEncoderFresh(*state)) {
    if (!ensureCanNormalMode("odrive position request")) {
      return false;
    }
    if (!waitForOdriveEncoderEstimate(node_id, ENCODER_WAIT_TIMEOUT_MS)) {
      return false;
    }
  }

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  const float output_deg = state->haveZeroReference
                               ? motorTurnsToJointDegrees(node_id, state->lastEncoderPosTurns - state->zeroReferenceTurns)
                               : 0.0f;
  Serial.printf("%s odrive pos_turns=%.6f vel_turns_s=%.6f zero=%s output_deg=%.3f\n",
                target != nullptr ? target->name : "odrive",
                state->lastEncoderPosTurns,
                state->lastEncoderVelTurnsPerSecond,
                state->haveZeroReference ? "yes" : "no",
                output_deg);
  return true;
}

inline bool sendOdriveOutputDegrees(uint8_t node_id, double output_deg) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }

  if (!ensureOdriveReadyForMotion(node_id, "goto")) {
    return false;
  }

  const float relative_turns = jointDegreesToMotorTurns(node_id, static_cast<float>(output_deg));
  const float absolute_turns = state->zeroReferenceTurns + relative_turns;

  state->haveActiveTarget           = true;
  state->lastRelativeCommandTurns   = relative_turns;
  state->lastRelativeCommandDegrees = static_cast<float>(output_deg);
  state->lastAbsoluteCommandTurns   = absolute_turns;
  state->lastTargetStreamMs         = millis();

  const UnifiedMotorTarget *target = targetForOdriveNode(node_id);
  Serial.printf("%s goto output_deg=%.3f -> motor_delta=%.6f turns -> absolute_turns=%.6f\n",
                target != nullptr ? target->name : "odrive",
                output_deg,
                relative_turns,
                absolute_turns);

  return sendOdriveInputPositionToNode(node_id, absolute_turns, 0, 0, "Sent Set_Input_Pos", true);
}

inline bool setOdriveIdle(uint8_t node_id) {
  NodeRuntimeState *state = odriveStateForNode(node_id);
  if (state == nullptr) {
    return false;
  }
  if (!ensureCanNormalMode("odrive idle")) {
    return false;
  }
  state->haveActiveTarget = false;
  return sendOdriveAxisStateRequest(node_id, AXIS_STATE_IDLE, "Requested IDLE", true);
}
