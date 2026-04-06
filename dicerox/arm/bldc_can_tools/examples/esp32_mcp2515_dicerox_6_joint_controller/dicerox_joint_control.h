#pragma once
#include "dicerox_odrive_control.h"
#include "dicerox_mixed_control.h"

using namespace odrive_can_sniffer;

// ── kUnifiedTargets – the one authoritative definition ────────────────────────
// (extern-declared in dicerox_globals.h; defined here exactly once)
constexpr UnifiedMotorTarget kUnifiedTargets[kUnifiedMotorCount] = {
    {"joint1", "j1",       UNIFIED_DRIVER_ODRIVE,  NODE_ID_10, 0},
    {"joint2", "j2",       UNIFIED_DRIVER_ODRIVE,  NODE_ID_11, 0},
    {"joint3", "j3",       UNIFIED_DRIVER_ODRIVE,  NODE_ID_12, 0},
    {"joint4", "ze300",    UNIFIED_DRIVER_ZE300,   0,          0},
    {"joint5", "lktech14", UNIFIED_DRIVER_LKTECH,  0,          1},
    {"joint6", "lktech15", UNIFIED_DRIVER_LKTECH,  0,          2},
};

// ── Accessor wrappers ─────────────────────────────────────────────────────────

inline const UnifiedMotorTarget &unifiedTarget(size_t index) {
  return kUnifiedTargets[index];
}

inline const UnifiedMotorTarget &activeTarget() {
  return unifiedTarget(g_active_target_index);
}

inline const MixedMotorConfig &mixedConfig(size_t index) {
  return kMixedMotorConfigs[index];
}

inline MixedMotorRuntimeState &mixedState(size_t index) {
  return g_mixed_states[index];
}

inline bool isOdriveTarget(size_t target_index) {
  return unifiedTarget(target_index).driver_type == UNIFIED_DRIVER_ODRIVE;
}

inline bool isMixedTarget(size_t target_index) {
  return unifiedTarget(target_index).driver_type != UNIFIED_DRIVER_ODRIVE;
}

inline bool activeTargetIsOdrive() {
  return isOdriveTarget(g_active_target_index);
}

inline const char *unifiedDriverName(const UnifiedMotorTarget &target) {
  switch (target.driver_type) {
    case UNIFIED_DRIVER_ODRIVE:  return "ODrive";
    case UNIFIED_DRIVER_LKTECH:  return "LKTech";
    case UNIFIED_DRIVER_ZE300:   return "ZE300";
    default:                     return "Unknown";
  }
}

// ── Initializers ─────────────────────────────────────────────────────────────

inline void initializeOdriveStates() {
  initializeNodeStates(g_odrive_states, SUPPORTED_NODE_COUNT);
}

inline void initializeMixedStates() {
  for (size_t index = 0; index < kMixedMotorConfigCount; ++index) {
    MixedMotorRuntimeState &state = g_mixed_states[index];
    state.have_zero_offset        = false;
    state.enabled                 = false;
    state.ze300_speed_applied     = false;
    state.zero_offset_command_deg = 0.0;
    state.last_command_deg        = 0.0;
    state.output_speed_dps        = kMixedMotorConfigs[index].default_output_speed_dps;
    state.last_native_counts      = 0;
  }
}

inline void selectActiveTarget(size_t target_index) {
  if (target_index >= kUnifiedMotorCount) {
    return;
  }
  g_active_target_index = target_index;
  const UnifiedMotorTarget &target = activeTarget();
  Serial.printf("active target set to %s (%s)\n",
                target.name,
                unifiedDriverName(target));
}

// ── String helpers ────────────────────────────────────────────────────────────

inline bool equalsIgnoreCase(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) {
    return false;
  }
  while (*left != '\0' && *right != '\0') {
    if (tolower(static_cast<unsigned char>(*left)) !=
        tolower(static_cast<unsigned char>(*right))) {
      return false;
    }
    ++left;
    ++right;
  }
  return *left == '\0' && *right == '\0';
}

inline char *trimWhitespace(char *text) {
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }
  char *end = text + strlen(text);
  while (end > text && isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  *end = '\0';
  return text;
}

inline bool parseUnsignedLongToken(const char *token, unsigned long *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }
  char *end = nullptr;
  const unsigned long parsed = strtoul(token, &end, 0);
  if (end == token || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

inline bool parseDoubleToken(const char *token, double *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }
  char *end = nullptr;
  const double parsed = strtod(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

inline int tokenize(char *text, char *tokens[], int max_tokens) {
  int count = 0;
  for (char *token = strtok(text, " \t"); token != nullptr && count < max_tokens;
       token = strtok(nullptr, " \t")) {
    tokens[count++] = token;
  }
  return count;
}

// ── Target lookup ─────────────────────────────────────────────────────────────

inline bool parseJointNumberAlias(const char *token, size_t *target_index) {
  if (token == nullptr || target_index == nullptr) {
    return false;
  }
  if (tolower(static_cast<unsigned char>(token[0])) != 'j' || token[1] == '\0' || token[2] != '\0') {
    return false;
  }
  const char digit = token[1];
  if (digit < '1' || digit > '6') {
    return false;
  }
  *target_index = static_cast<size_t>(digit - '1');
  return true;
}

inline bool findUnifiedTargetByToken(const char *token, size_t *target_index) {
  if (token == nullptr || target_index == nullptr) {
    return false;
  }

  for (size_t index = 0; index < kUnifiedMotorCount; ++index) {
    const UnifiedMotorTarget &target = unifiedTarget(index);
    if (equalsIgnoreCase(token, target.name) || equalsIgnoreCase(token, target.alias)) {
      *target_index = index;
      return true;
    }
  }

  if (parseJointNumberAlias(token, target_index)) {
    return true;
  }

  unsigned long parsed = 0;
  if (!parseUnsignedLongToken(token, &parsed)) {
    return false;
  }

  if (parsed >= 1 && parsed <= 6) {
    *target_index = static_cast<size_t>(parsed - 1);
    return true;
  }

  if (parsed == NODE_ID_10) {
    *target_index = 0;
    return true;
  }
  if (parsed == NODE_ID_11) {
    *target_index = 1;
    return true;
  }
  if (parsed == NODE_ID_12) {
    *target_index = 2;
    return true;
  }

  if (parsed == mixedConfig(0).device_id) {
    *target_index = 3;
    return true;
  }
  if (parsed == mixedConfig(1).device_id) {
    *target_index = 4;
    return true;
  }
  if (parsed == mixedConfig(2).device_id) {
    *target_index = 5;
    return true;
  }

  return false;
}

inline void printUnknownTarget(const char *token) {
  Serial.printf("unknown target: %s\n", token == nullptr ? "<null>" : token);
  Serial.println("use joint1..joint6, j1..j6, or ze300/lktech14/lktech15");
}

// ── Unified action functions ──────────────────────────────────────────────────

inline bool readUnifiedPosition(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    return readOdriveCurrentPosition(target.odrive_node_id);
  }
  return readMixedCurrentPosition(target.mixed_index);
}

inline bool zeroUnifiedMotor(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    if (!ensureOdriveReadyForMotion(target.odrive_node_id, "zero")) {
      return false;
    }
    return captureOdriveZero(target.odrive_node_id);
  }
  return captureMixedZeroOffset(target.mixed_index);
}

inline bool onUnifiedMotor(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    return ensureOdriveReadyForMotion(target.odrive_node_id, "on");
  }
  return sendMixedOn(target.mixed_index);
}

inline bool offUnifiedMotor(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    return setOdriveIdle(target.odrive_node_id);
  }
  return sendMixedOff(target.mixed_index);
}

inline bool stopUnifiedMotor(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    Serial.printf("%s ODrive stop maps to IDLE\n", target.name);
    return setOdriveIdle(target.odrive_node_id);
  }
  return sendMixedStop(target.mixed_index);
}

inline bool gotoUnifiedMotor(size_t target_index, double output_deg) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    return sendOdriveOutputDegrees(target.odrive_node_id, output_deg);
  }
  return commandMixedAbsoluteOutput(target.mixed_index, output_deg);
}

inline bool speedUnifiedMotor(size_t target_index, double output_dps) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  if (target.driver_type == UNIFIED_DRIVER_ODRIVE) {
    Serial.printf("%s uses ODrive position control; speed command is not implemented in this sketch\n",
                  target.name);
    return false;
  }
  return setMixedOutputSpeed(target.mixed_index, output_dps);
}

// ── Status printers ───────────────────────────────────────────────────────────

inline void printOdriveStatusLine(size_t target_index) {
  const UnifiedMotorTarget &target = unifiedTarget(target_index);
  const NodeRuntimeState   *state  = odriveStateForNode(target.odrive_node_id);
  const float output_deg = (state != nullptr && state->haveZeroReference)
                               ? motorTurnsToJointDegrees(target.odrive_node_id,
                                                           state->lastEncoderPosTurns - state->zeroReferenceTurns)
                               : 0.0f;

  Serial.printf("%s name=%s driver=ODrive node=0x%02X state=%s hb_err=0x%lX disarm=0x%lX zero=%s have_encoder=%s pos_turns=%.4f vel_turns_s=%.4f output_deg=%.2f stream=%s last_target_deg=%.2f\n",
                target_index == g_active_target_index ? "*" : " ",
                target.name,
                static_cast<unsigned>(target.odrive_node_id),
                state != nullptr ? axisStateName(state->lastAxisState) : "UNKNOWN",
                static_cast<unsigned long>(state != nullptr ? state->lastHeartbeatActiveErrors : 0),
                static_cast<unsigned long>(state != nullptr ? state->lastDisarmReason : 0),
                (state != nullptr && state->haveZeroReference)          ? "yes" : "no",
                (state != nullptr && state->haveLatestEncoderEstimate)  ? "yes" : "no",
                state != nullptr ? state->lastEncoderPosTurns           : 0.0f,
                state != nullptr ? state->lastEncoderVelTurnsPerSecond  : 0.0f,
                output_deg,
                (state != nullptr && state->haveActiveTarget)           ? "yes" : "no",
                state != nullptr ? state->lastRelativeCommandDegrees    : 0.0f);
}

inline void printMixedStatusLine(size_t target_index) {
  const UnifiedMotorTarget     &target = unifiedTarget(target_index);
  const MixedMotorConfig       &config = kMixedMotorConfigs[target.mixed_index];
  const MixedMotorRuntimeState &state  = g_mixed_states[target.mixed_index];

  Serial.printf("%s name=%s driver=%s device_id=%u req_id=0x%03X reply_id=0x%03X speed_output=%.2f dps enabled=%s zero=%s last_cmd_deg=%.2f",
                target_index == g_active_target_index ? "*" : " ",
                target.name,
                target.driver_type == UNIFIED_DRIVER_LKTECH ? "LKTech" : "ZE300",
                static_cast<unsigned>(config.device_id),
                mixedRequestId(target.mixed_index),
                mixedReplyId(target.mixed_index),
                state.output_speed_dps,
                state.enabled        ? "yes" : "no",
                state.have_zero_offset ? "yes" : "no",
                state.last_command_deg);

  if (config.driver_type == DRIVER_LKTECH) {
    Serial.printf(" lk_motor_speed=%u dps",
                  static_cast<unsigned>(lkOutputSpeedToMotorDps(target.mixed_index, state.output_speed_dps)));
  } else {
    Serial.printf(" ze300_speed_limit=%.2f dps",
                  ze300MotorCentiRpmToOutputDps(
                      target.mixed_index,
                      ze300OutputSpeedToMotorCentiRpm(target.mixed_index, state.output_speed_dps)));
  }

  if (state.have_zero_offset) {
    Serial.printf(" zero_cmd_deg=%.2f last_output_deg=%.2f",
                  state.zero_offset_command_deg,
                  currentMixedOutputDegrees(target.mixed_index));
  }
  Serial.println();
}

inline void printTargetStatusLine(size_t target_index) {
  if (isOdriveTarget(target_index)) {
    printOdriveStatusLine(target_index);
  } else {
    printMixedStatusLine(target_index);
  }
}

inline void printAllStatus() {
  Serial.println("Configured motors:");
  for (size_t index = 0; index < kUnifiedMotorCount; ++index) {
    printTargetStatusLine(index);
  }
}

inline void printConfig() {
  Serial.printf("config active=%s driver=%s bitrate=1Mbps oscillator=%s mode=%s raw=%s odrive_telemetry=%s odrive_stream=%s\n",
                activeTarget().name,
                unifiedDriverName(activeTarget()),
                oscillatorName(g_oscillator),
                modeName(g_mode),
                g_show_all_frames         ? "on" : "off",
                g_print_odrive_telemetry  ? "on" : "off",
                g_stream_odrive_targets   ? "on" : "off");
}

// ── Utility helpers ───────────────────────────────────────────────────────────

inline bool runActionOnAll(bool (*fn)(size_t)) {
  bool all_ok = true;
  for (size_t index = 0; index < kUnifiedMotorCount; ++index) {
    if (!fn(index)) {
      all_ok = false;
    }
    serviceOdriveTargetStreaming();
    delay(10);
  }
  return all_ok;
}

inline void sendSelfTestFrame() {
  can_frame frame = {};
  frame.can_id    = 0x123;
  frame.can_dlc   = 4;
  frame.data[0]   = 0xDE;
  frame.data[1]   = 0xAD;
  frame.data[2]   = 0xBE;
  frame.data[3]   = 0xEF;
  sendCanFrame(frame, "Sent self-test frame 123#DEADBEEF", true);
}

inline void printSingleTargetStatus(const char *token) {
  size_t target_index = 0;
  if (!findUnifiedTargetByToken(token, &target_index)) {
    printUnknownTarget(token);
    return;
  }
  printTargetStatusLine(target_index);
}
