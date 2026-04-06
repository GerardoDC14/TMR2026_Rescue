#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif

#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000UL
#endif

#include "odrive_can_support.h"
#include "odrive_can_state.h"

namespace {

using namespace odrive_can_sniffer;

MCP2515 canController(PIN_CAN_CS);
struct can_frame rxFrame;

CanMode g_mode = CanMode::Normal;
Oscillator g_oscillator = Oscillator::MHz8;
bool g_showAllStandardFrames = false;
bool g_printEncoderTelemetry = false;
bool g_streamLastTarget = true;
uint8_t g_activeNodeId = DEFAULT_ODRIVE_NODE_ID;
NodeRuntimeState g_nodeStates[SUPPORTED_NODE_COUNT];
char g_serialLine[SERIAL_LINE_BUFFER_SIZE] = {};
size_t g_serialLineLen = 0;

void processIncomingFrames();
void serviceTargetStreaming();
void printHexByte(uint8_t value);
bool ensureActiveNodeReadyForMotion(const char *reason);
void runAutomaticBringupAllNodes();

bool isTrackedNodeId(uint8_t nodeId) {
  return odrive_can_sniffer::isSupportedNodeId(nodeId);
}

NodeRuntimeState *lookupNodeState(uint8_t nodeId) {
  return odrive_can_sniffer::findNodeState(g_nodeStates, SUPPORTED_NODE_COUNT, nodeId);
}

NodeRuntimeState &selectedNodeState() {
  return odrive_can_sniffer::activeNodeState(g_nodeStates, SUPPORTED_NODE_COUNT, g_activeNodeId);
}

void initializeRuntimeStates() {
  odrive_can_sniffer::initializeNodeStates(g_nodeStates, SUPPORTED_NODE_COUNT);
}

void selectActiveNode(uint8_t nodeId) {
  if (!isTrackedNodeId(nodeId)) {
    Serial.print("Unsupported node ID 0x");
    printHexByte(nodeId);
    Serial.println();
    return;
  }

  g_activeNodeId = nodeId;

  Serial.print("Active node set to 0x");
  printHexByte(g_activeNodeId);
  Serial.print(". Cached zero=");
  Serial.print(selectedNodeState().haveZeroReference ? "yes" : "no");
  Serial.println(".");
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

// -----------------------------
// CAN / MCP2515 configuration
// -----------------------------
bool applyCanMode() {
  switch (g_mode) {
    case CanMode::Normal:
      return canController.setNormalMode() == MCP2515::ERROR_OK;
    case CanMode::ListenOnly:
      return canController.setListenOnlyMode() == MCP2515::ERROR_OK;
    case CanMode::Loopback:
      return canController.setLoopbackMode() == MCP2515::ERROR_OK;
    default:
      return false;
  }
}

bool configureCan() {
  canController.reset();

  auto osc = (g_oscillator == Oscillator::MHz8) ? MCP_8MHZ : MCP_16MHZ;

  if (canController.setBitrate(CAN_1000KBPS, osc) != MCP2515::ERROR_OK) {
    Serial.println("ERROR: failed to set MCP2515 bitrate");
    return false;
  }

  if (!applyCanMode()) {
    Serial.println("ERROR: failed to set MCP2515 mode");
    return false;
  }

  Serial.println("MCP2515 configured OK");
  return true;
}

// -----------------------------
// Printing / decoding
// -----------------------------
void printRawFrame(const char *prefix, const can_frame &frame) {
  const uint32_t stdId = getStandardId(frame);
  const uint8_t nodeId = extractNodeId(stdId);
  const uint8_t cmdId  = extractCommandId(stdId);

  Serial.print(prefix);
  Serial.print(" id=0x");
  Serial.print(stdId, HEX);
  Serial.print(" node=0x");
  printHexByte(nodeId);
  Serial.print(" cmd=0x");
  printHexByte(cmdId);
  Serial.print(" (");
  Serial.print(commandName(cmdId));
  Serial.print(")");
  Serial.print(" ext=");
  Serial.print(isExtendedFrame(frame) ? 1 : 0);
  Serial.print(" rtr=");
  Serial.print(isRemoteFrame(frame) ? 1 : 0);
  Serial.print(" dlc=");
  Serial.print(frame.can_dlc);
  Serial.print(" data=");

  if (frame.can_dlc == 0) {
    Serial.print("(empty)");
  } else {
    for (uint8_t i = 0; i < frame.can_dlc; ++i) {
      if (i > 0) Serial.print(' ');
      printHexByte(frame.data[i]);
    }
  }

  Serial.println();
}

void printCachedAxisStatus(const char *prefix) {
  const NodeRuntimeState &state = selectedNodeState();
  Serial.print(prefix);
  Serial.print(" state=");
  Serial.print(axisStateName(state.lastAxisState));
  Serial.print(" heartbeat_errors=0x");
  Serial.print(state.lastHeartbeatActiveErrors, HEX);
  Serial.print(" active_errors=0x");
  Serial.print(state.lastErrorActiveErrors, HEX);
  Serial.print(" disarm_reason=0x");
  Serial.println(state.lastDisarmReason, HEX);
}

void decodeHeartbeat(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 5) {
    if (g_showAllStandardFrames) {
      Serial.println("  Heartbeat frame too short");
    }
    return;
  }

  const uint32_t activeErrors = bytesToU32(frame.data);
  const uint8_t axisState = frame.data[4];
  const bool changed = !state.haveHeartbeat ||
                       activeErrors != state.lastHeartbeatActiveErrors ||
                       axisState != state.lastAxisState;

  state.haveHeartbeat = true;
  state.lastHeartbeatActiveErrors = activeErrors;
  state.lastAxisState = axisState;
  state.lastHeartbeatMs = millis();

  if (changed && state.nodeId == g_activeNodeId) {
    Serial.print("Heartbeat state=");
    Serial.print(axisStateName(axisState));
    Serial.print(" active_errors=0x");
    Serial.println(activeErrors, HEX);
  }
}

void decodeGetError(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    if (g_showAllStandardFrames) {
      Serial.println("  Get_Error frame too short");
    }
    return;
  }

  const uint32_t activeErrors = bytesToU32(frame.data);
  const uint32_t disarmReason = bytesToU32(frame.data + 4);
  const bool changed = !state.haveErrorStatus ||
                       activeErrors != state.lastErrorActiveErrors ||
                       disarmReason != state.lastDisarmReason;

  state.haveErrorStatus = true;
  state.lastErrorActiveErrors = activeErrors;
  state.lastDisarmReason = disarmReason;
  state.lastErrorMs = millis();

  if (state.nodeId == g_activeNodeId && (changed || g_showAllStandardFrames)) {
    Serial.print("ODrive active_errors=0x");
    Serial.print(activeErrors, HEX);
    Serial.print(" disarm_reason=0x");
    Serial.println(disarmReason, HEX);
  }
}

void decodeEncoderEstimates(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    if (g_showAllStandardFrames) {
      Serial.println("  Encoder frame too short");
    }
    return;
  }

  const float posTurns = bytesToFloat(frame.data);
  const float velTurnsPerSecond = bytesToFloat(frame.data + 4);
  state.lastEncoderPosTurns = posTurns;
  state.lastEncoderVelTurnsPerSecond = velTurnsPerSecond;
  state.haveLatestEncoderEstimate = true;
  state.lastEncoderEstimateMs = millis();

  if (state.nodeId == g_activeNodeId &&
      g_printEncoderTelemetry &&
      (state.lastEncoderTelemetryPrintMs == 0 ||
       state.lastEncoderEstimateMs - state.lastEncoderTelemetryPrintMs >= ENCODER_TELEMETRY_INTERVAL_MS)) {
    state.lastEncoderTelemetryPrintMs = state.lastEncoderEstimateMs;
    Serial.print("Encoder pos_turns=");
    Serial.print(posTurns, 6);
    Serial.print(" vel_turns_s=");
    Serial.println(velTurnsPerSecond, 6);
  }
}

void decodeFrame(const can_frame &frame) {
  if (isExtendedFrame(frame) || isRemoteFrame(frame)) {
    if (g_showAllStandardFrames) {
      printRawFrame("RX", frame);
    }
    return;
  }

  const uint32_t stdId = getStandardId(frame);
  const uint8_t nodeId = extractNodeId(stdId);
  const uint8_t cmdId  = extractCommandId(stdId);

  NodeRuntimeState *state = lookupNodeState(nodeId);
  if (state == nullptr) {
    if (g_showAllStandardFrames) {
      printRawFrame("RX", frame);
    }
    return;
  }

  if (g_showAllStandardFrames) {
    printRawFrame("RX", frame);
  }

  switch (cmdId) {
    case CMD_HEARTBEAT:
      decodeHeartbeat(*state, frame);
      break;
    case CMD_GET_ERROR:
      decodeGetError(*state, frame);
      break;
    case CMD_GET_ENCODER_ESTIMATES:
      decodeEncoderEstimates(*state, frame);
      break;
    default:
      break;
  }
}

// -----------------------------
// TX helpers
// -----------------------------
bool sendFrame(const can_frame &frame, const char *message = nullptr, bool logFrame = true) {
  can_frame tx = frame;  // sendMessage expects non-const pointer
  if (logFrame) {
    printRawFrame("TX", tx);
  }

  if (canController.sendMessage(&tx) != MCP2515::ERROR_OK) {
    Serial.println("ERROR: send failed");
    return false;
  }

  if (message != nullptr) {
    Serial.println(message);
  }

  return true;
}

void sendAxisStateRequest(uint32_t state) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, CMD_SET_AXIS_STATE);
  frame.can_dlc = 4;
  frame.data[0] = static_cast<uint8_t>( state        & 0xFF);
  frame.data[1] = static_cast<uint8_t>((state >> 8 ) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((state >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((state >> 24) & 0xFF);

  if (state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
    sendFrame(frame, "Requested CLOSED_LOOP_CONTROL");
  } else if (state == AXIS_STATE_IDLE) {
    sendFrame(frame, "Requested IDLE");
  } else {
    sendFrame(frame, "Requested axis state");
  }
}

bool sendAxisStateRequest(uint32_t state, const char *message, bool logFrame) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, CMD_SET_AXIS_STATE);
  frame.can_dlc = 4;
  frame.data[0] = static_cast<uint8_t>( state        & 0xFF);
  frame.data[1] = static_cast<uint8_t>((state >> 8 ) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((state >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((state >> 24) & 0xFF);
  return sendFrame(frame, message, logFrame);
}

void sendRemoteRequest(uint8_t cmd, uint8_t dlc, const char *message) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, cmd) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  sendFrame(frame, message);
}

bool sendRemoteRequest(uint8_t cmd, uint8_t dlc, const char *message, bool logFrame) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, cmd) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  return sendFrame(frame, message, logFrame);
}

void sendInputPositionToNode(uint8_t nodeId,
                             float absoluteTurns,
                             int16_t velFeedforward = 0,
                             int16_t torqueFeedforward = 0,
                             const char *message = "Sent Set_Input_Pos",
                             bool logFrame = true) {
  can_frame frame = {};
  frame.can_id = makeCanId(nodeId, CMD_SET_INPUT_POS);
  frame.can_dlc = 8;
  writeFloatToBytes(absoluteTurns, &frame.data[0]);
  writeI16ToBytes(velFeedforward, &frame.data[4]);
  writeI16ToBytes(torqueFeedforward, &frame.data[6]);
  sendFrame(frame, message, logFrame);
}

void sendInputPosition(float absoluteTurns,
                       int16_t velFeedforward = 0,
                       int16_t torqueFeedforward = 0,
                       const char *message = "Sent Set_Input_Pos",
                       bool logFrame = true) {
  sendInputPositionToNode(g_activeNodeId, absoluteTurns, velFeedforward, torqueFeedforward, message, logFrame);
}

void sendSelfTestFrame() {
  can_frame frame = {};
  frame.can_id = 0x123;
  frame.can_dlc = 4;
  frame.data[0] = 0xDE;
  frame.data[1] = 0xAD;
  frame.data[2] = 0xBE;
  frame.data[3] = 0xEF;
  sendFrame(frame, "Sent self-test frame 123#DEADBEEF");
}

void captureZeroFromLatestPosition() {
  NodeRuntimeState &state = selectedNodeState();

  if (!state.haveLatestEncoderEstimate) {
    Serial.println("Cannot set zero yet: no encoder estimate received. Wait for 0x249 or send 'e'.");
    return;
  }

  state.zeroReferenceTurns = state.lastEncoderPosTurns;
  state.haveZeroReference = true;
  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = 0.0f;
  state.lastRelativeCommandDegrees = 0.0f;
  state.lastAbsoluteCommandTurns = state.zeroReferenceTurns;
  state.lastTargetStreamMs = millis();

  Serial.print("Zero reference captured at ");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.println(" turns");

  if (g_mode == CanMode::Normal) {
    sendInputPosition(state.lastAbsoluteCommandTurns, 0, 0, "Holding current position as zero reference");
  }
}

void sendRelativePositionTurns(float relativeTurns) {
  NodeRuntimeState &state = selectedNodeState();

  if (!ensureActiveNodeReadyForMotion("turn command")) {
    Serial.println("Cannot send relative turns: automatic bringup did not complete.");
    return;
  }

  const float absoluteTurns = state.zeroReferenceTurns + relativeTurns;
  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = relativeTurns;
  state.lastRelativeCommandDegrees = motorTurnsToJointDegrees(g_activeNodeId, relativeTurns);
  state.lastAbsoluteCommandTurns = absoluteTurns;
  state.lastTargetStreamMs = millis();

  Serial.print("Relative target=");
  Serial.print(relativeTurns, 6);
  Serial.print(" turns -> absolute target=");
  Serial.print(absoluteTurns, 6);
  Serial.println(" turns");

  sendInputPosition(absoluteTurns);
}

void sendRelativePositionDegrees(float relativeDegrees) {
  NodeRuntimeState &state = selectedNodeState();

  if (!ensureActiveNodeReadyForMotion("angle command")) {
    Serial.println("Cannot send relative angle: automatic bringup did not complete.");
    return;
  }

  const float relativeTurns = jointDegreesToMotorTurns(g_activeNodeId, relativeDegrees);
  const float absoluteTurns = state.zeroReferenceTurns + relativeTurns;

  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = relativeTurns;
  state.lastRelativeCommandDegrees = relativeDegrees;
  state.lastAbsoluteCommandTurns = absoluteTurns;
  state.lastTargetStreamMs = millis();

  Serial.print("Relative target=");
  Serial.print(relativeDegrees, 3);
  Serial.print(" deg -> motor_delta=");
  Serial.print(relativeTurns, 6);
  Serial.print(" turns -> absolute target=");
  Serial.print(absoluteTurns, 6);
  Serial.println(" turns");

  sendInputPosition(absoluteTurns);
}

void printMotionState() {
  const NodeRuntimeState &state = selectedNodeState();
  const float relativeTurnsFromZero = state.lastEncoderPosTurns - state.zeroReferenceTurns;
  const float relativeJointDegrees = motorTurnsToJointDegrees(g_activeNodeId, relativeTurnsFromZero);
  Serial.print("motion node=0x");
  printHexByte(g_activeNodeId);
  Serial.print(" axis_state=");
  Serial.print(axisStateName(state.lastAxisState));
  Serial.print(" heartbeat_errors=0x");
  Serial.print(state.lastHeartbeatActiveErrors, HEX);
  Serial.print(" disarm=0x");
  Serial.print(state.lastDisarmReason, HEX);
  Serial.print(" have_encoder=");
  Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
  Serial.print(" pos_turns=");
  Serial.print(state.lastEncoderPosTurns, 6);
  Serial.print(" vel_turns_s=");
  Serial.print(state.lastEncoderVelTurnsPerSecond, 6);
  Serial.print(" have_zero=");
  Serial.print(state.haveZeroReference ? "yes" : "no");
  Serial.print(" zero_turns=");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.print(" joint_deg=");
  Serial.print(relativeJointDegrees, 3);
  Serial.print(" gear_ratio=");
  Serial.print(nodeGearRatio(g_activeNodeId), 1);
  Serial.print(" direction=");
  Serial.print(nodeDirection(g_activeNodeId), 1);
  Serial.print(" stream=");
  Serial.print(g_streamLastTarget ? "on" : "off");
  Serial.print(" last_rel_cmd=");
  Serial.print(state.lastRelativeCommandTurns, 6);
  Serial.print(" last_rel_deg=");
  Serial.print(state.lastRelativeCommandDegrees, 3);
  Serial.print(" last_abs_cmd=");
  Serial.println(state.lastAbsoluteCommandTurns, 6);
}

// -----------------------------
// UI / commands
// -----------------------------
void printConfig() {
  const NodeRuntimeState &state = selectedNodeState();
  Serial.print("config node=0x");
  printHexByte(g_activeNodeId);
  Serial.print(" bitrate=1Mbps oscillator=");
  Serial.print(oscillatorName(g_oscillator));
  Serial.print(" mode=");
  Serial.print(modeName(g_mode));
  Serial.print(" show_all_std=");
  Serial.print(g_showAllStandardFrames ? "on" : "off");
  Serial.print(" telemetry=");
  Serial.print(g_printEncoderTelemetry ? "on" : "off");
  Serial.print(" have_zero=");
  Serial.print(state.haveZeroReference ? "yes" : "no");
  Serial.print(" zero_turns=");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.print(" gear_ratio=");
  Serial.print(nodeGearRatio(g_activeNodeId), 1);
  Serial.print(" direction=");
  Serial.print(nodeDirection(g_activeNodeId), 1);
  Serial.print(" stream=");
  Serial.print(g_streamLastTarget ? "on" : "off");
  Serial.print(" have_target=");
  Serial.print(state.haveActiveTarget ? "yes" : "no");
  Serial.print(" have_encoder=");
  Serial.println(state.haveLatestEncoderEstimate ? "yes" : "no");
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h or ? : help");
  Serial.println("  p      : print config");
  Serial.println("  1      : select node 0x10");
  Serial.println("  2      : select node 0x11");
  Serial.println("  3      : select node 0x12");
  Serial.println("  b      : retry automatic bringup for selected node");
  Serial.println("  a      : toggle raw RX debug");
  Serial.println("  v      : toggle throttled encoder telemetry");
  Serial.println("  s      : send self-test frame 123#DEADBEEF");
  Serial.println("  c      : request and confirm CLOSED_LOOP_CONTROL");
  Serial.println("  x      : request IDLE");
  Serial.println("  e      : RTR Get_Encoder_Estimates");
  Serial.println("  r      : RTR Get_Error");
  Serial.println("  z      : capture current encoder position as zero");
  Serial.println("  o      : print motion state");
  Serial.println("  g <n>  : send relative joint angle in degrees from zero");
  Serial.println("  t <n>  : send relative motor turns from zero (debug)");
  Serial.println("  k      : toggle 20 Hz resend of last Set_Input_Pos");
  Serial.println("  n      : normal mode");
  Serial.println("  i      : listen-only mode");
  Serial.println("  l      : loopback mode");
  Serial.println("  8      : oscillator = 8 MHz");
  Serial.println("  6      : oscillator = 16 MHz");
  Serial.println();
}

bool waitForEncoderEstimate(uint32_t timeoutMs) {
  NodeRuntimeState &state = selectedNodeState();

  if (state.haveLatestEncoderEstimate && millis() - state.lastEncoderEstimateMs <= ENCODER_WAIT_TIMEOUT_MS) {
    return true;
  }

  sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates", true);
  const unsigned long startMs = millis();
  unsigned long lastRetryMs = startMs;

  while (millis() - startMs < timeoutMs) {
    processIncomingFrames();
    serviceTargetStreaming();

    if (state.haveLatestEncoderEstimate && state.lastEncoderEstimateMs >= startMs) {
      return true;
    }

    if (millis() - lastRetryMs >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      lastRetryMs = millis();
      sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, nullptr, false);
    }

    delay(1);
  }

  Serial.println("Timed out waiting for encoder estimate.");
  return false;
}

bool requestClosedLoopAndConfirm(uint32_t timeoutMs) {
  NodeRuntimeState &state = selectedNodeState();
  sendAxisStateRequest(AXIS_STATE_CLOSED_LOOP_CONTROL, "Requested CLOSED_LOOP_CONTROL", true);

  const unsigned long startMs = millis();
  unsigned long lastRetryMs = startMs;

  while (millis() - startMs < timeoutMs) {
    processIncomingFrames();
    serviceTargetStreaming();

    if (state.haveHeartbeat &&
        state.lastHeartbeatMs >= startMs &&
        state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
        state.lastHeartbeatActiveErrors == 0) {
      Serial.println("Closed loop confirmed.");
      return true;
    }

    if (millis() - lastRetryMs >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      lastRetryMs = millis();
      sendAxisStateRequest(AXIS_STATE_CLOSED_LOOP_CONTROL, nullptr, false);
    }

    delay(1);
  }

  Serial.println("Closed loop not confirmed.");
  sendRemoteRequest(CMD_GET_ERROR, 8, "Requested Get_Error after closed-loop timeout", true);

  const unsigned long errorWaitStartMs = millis();
  while (millis() - errorWaitStartMs < 150) {
    processIncomingFrames();
    delay(1);
  }

  printCachedAxisStatus("Axis status:");
  return false;
}

bool runSelectedNodeBringupAttempt() {
  NodeRuntimeState &state = selectedNodeState();
  state.haveZeroReference = false;
  state.haveActiveTarget = false;
  state.lastRelativeCommandTurns = 0.0f;
  state.lastRelativeCommandDegrees = 0.0f;

  if (!requestClosedLoopAndConfirm(CLOSED_LOOP_CONFIRM_TIMEOUT_MS)) {
    return false;
  }

  if (!waitForEncoderEstimate(ENCODER_WAIT_TIMEOUT_MS)) {
    return false;
  }

  captureZeroFromLatestPosition();
  return nodeIsReadyForMotion(state);
}

bool runSelectedNodeBringupWithRetries(const char *reason) {
  if (g_mode != CanMode::Normal) {
    g_mode = CanMode::Normal;
    if (!configureCan()) {
      Serial.println("Bringup failed: could not switch MCP2515 to normal mode.");
      return false;
    }
  }

  Serial.print("Auto bringup node=0x");
  printHexByte(g_activeNodeId);
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" trigger=");
    Serial.print(reason);
  }
  Serial.println();

  for (uint8_t attempt = 1; attempt <= BRINGUP_MAX_ATTEMPTS; ++attempt) {
    Serial.print("Bringup attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(BRINGUP_MAX_ATTEMPTS);

    if (runSelectedNodeBringupAttempt()) {
      printMotionState();
      Serial.println("Bringup complete.");
      return true;
    }

    sendAxisStateRequest(AXIS_STATE_IDLE, nullptr, false);
    delay(BRINGUP_RETRY_SETTLE_MS);
  }

  Serial.print("Bringup failed after ");
  Serial.print(BRINGUP_MAX_ATTEMPTS);
  Serial.println(" attempts.");
  printCachedAxisStatus("Axis status:");
  return false;
}

bool ensureActiveNodeReadyForMotion(const char *reason) {
  if (g_mode == CanMode::Normal && nodeIsReadyForMotion(selectedNodeState())) {
    return true;
  }
  return runSelectedNodeBringupWithRetries(reason);
}

void runBringup() {
  (void)ensureActiveNodeReadyForMotion("manual");
}

void runAutomaticBringupAllNodes() {
  const uint8_t previousActiveNodeId = g_activeNodeId;

  Serial.println("Automatic bringup starting for nodes 0x10, 0x11, 0x12");
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    g_activeNodeId = g_nodeStates[i].nodeId;
    (void)ensureActiveNodeReadyForMotion("startup");
  }

  g_activeNodeId = previousActiveNodeId;
  Serial.print("Automatic bringup finished. Active node restored to 0x");
  printHexByte(g_activeNodeId);
  Serial.println();
}

void handleCommand(char c) {
  switch (c) {
    case 'h':
    case '?':
      printHelp();
      break;

    case 'p':
      printConfig();
      break;

    case '1':
      selectActiveNode(NODE_ID_10);
      break;

    case '2':
      selectActiveNode(NODE_ID_11);
      break;

    case '3':
      selectActiveNode(NODE_ID_12);
      break;

    case 'b':
      runBringup();
      break;

    case 'a':
      g_showAllStandardFrames = !g_showAllStandardFrames;
      Serial.print("raw_rx_debug = ");
      Serial.println(g_showAllStandardFrames ? "ON" : "OFF");
      break;

    case 'v':
      g_printEncoderTelemetry = !g_printEncoderTelemetry;
      selectedNodeState().lastEncoderTelemetryPrintMs = 0;
      Serial.print("encoder_telemetry = ");
      Serial.println(g_printEncoderTelemetry ? "ON" : "OFF");
      break;

    case 's':
      sendSelfTestFrame();
      break;

    case 'c':
      requestClosedLoopAndConfirm(CLOSED_LOOP_CONFIRM_TIMEOUT_MS);
      break;

    case 'x':
      sendAxisStateRequest(AXIS_STATE_IDLE);
      break;

    case 'e':
      sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates");
      break;

    case 'r':
      sendRemoteRequest(CMD_GET_ERROR, 8, "Requested Get_Error");
      break;

    case 'z':
      captureZeroFromLatestPosition();
      break;

    case 'o':
      printMotionState();
      break;

    case 'k':
      g_streamLastTarget = !g_streamLastTarget;
      Serial.print("stream_last_target = ");
      Serial.println(g_streamLastTarget ? "ON" : "OFF");
      break;

    case 'n':
      g_mode = CanMode::Normal;
      configureCan();
      printConfig();
      break;

    case 'i':
      g_mode = CanMode::ListenOnly;
      configureCan();
      printConfig();
      break;

    case 'l':
      g_mode = CanMode::Loopback;
      configureCan();
      printConfig();
      break;

    case '8':
      g_oscillator = Oscillator::MHz8;
      configureCan();
      printConfig();
      break;

    case '6':
      g_oscillator = Oscillator::MHz16;
      configureCan();
      printConfig();
      break;

    case '\r':
    case '\n':
    case ' ':
    case '\t':
      break;

    default:
      Serial.print("Unknown command: ");
      Serial.println(c);
      printHelp();
      break;
  }
}

void handleLineCommand(const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }

  const bool isTurnCommand = line[0] == 't';
  const bool isDegreeShortCommand = line[0] == 'g';
  const bool isDegreeWordCommand =
      strncmp(line, "deg", 3) == 0 &&
      (line[3] == ' ' || line[3] == '\t' || line[3] == '\0');

  if (isTurnCommand || isDegreeShortCommand || isDegreeWordCommand) {
    const char *valueText = line + (isDegreeWordCommand ? 3 : 1);
    while (*valueText == ' ' || *valueText == '\t') {
      ++valueText;
    }

    if (*valueText == '\0') {
      if (isTurnCommand) {
        Serial.println("Usage: t <turns>");
      } else {
        Serial.println("Usage: g <degrees>");
      }
      return;
    }

    char *endPtr = nullptr;
    const float value = strtof(valueText, &endPtr);
    if (endPtr == valueText) {
      if (isTurnCommand) {
        Serial.println("Invalid turns value. Example: t 0.25");
      } else {
        Serial.println("Invalid degree value. Example: g 45");
      }
      return;
    }

    while (*endPtr == ' ' || *endPtr == '\t') {
      ++endPtr;
    }
    if (*endPtr != '\0') {
      Serial.println("Unexpected extra text after numeric value.");
      return;
    }

    if (isTurnCommand) {
      sendRelativePositionTurns(value);
    } else {
      sendRelativePositionDegrees(value);
    }
    return;
  }

  if (line[1] == '\0') {
    handleCommand(line[0]);
    return;
  }

  Serial.print("Unknown line command: ");
  Serial.println(line);
  printHelp();
}

void serviceTargetStreaming() {
  if (!g_streamLastTarget || g_mode != CanMode::Normal) {
    return;
  }

  const unsigned long now = millis();
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    NodeRuntimeState &state = g_nodeStates[i];
    if (!state.haveActiveTarget) {
      continue;
    }
    if (now - state.lastTargetStreamMs < TARGET_STREAM_INTERVAL_MS) {
      continue;
    }

    state.lastTargetStreamMs = now;
    sendInputPositionToNode(state.nodeId, state.lastAbsoluteCommandTurns, 0, 0, nullptr, false);
  }
}

// -----------------------------
// RX processing
// -----------------------------
void processIncomingFrames() {
  uint16_t frameCount = 0;

  while (digitalRead(PIN_CAN_INT) == LOW && frameCount < MAX_RX_FRAMES_PER_PASS) {
    if (canController.readMessage(&rxFrame) != MCP2515::ERROR_OK) {
      break;
    }

    decodeFrame(rxFrame);
    ++frameCount;
  }
}

} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  initializeRuntimeStates();
  pinMode(PIN_CAN_INT, INPUT);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

  Serial.println("ESP32 + MCP2515 ODrive CAN debugger");
  Serial.println("Supported nodes: 0x10, 0x11, 0x12");
  Serial.println("Default active node: 0x12, bitrate: 1 Mbps, serial: 115200");

  if (!configureCan()) {
    Serial.println("Initial CAN configuration failed");
  }

  runAutomaticBringupAllNodes();
  printConfig();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      if (g_serialLineLen > 0) {
        g_serialLine[g_serialLineLen] = '\0';
        handleLineCommand(g_serialLine);
        g_serialLineLen = 0;
      }
      continue;
    }

    const bool beginsLineCommand = (c == 't' || c == 'g' || c == 'd');
    if (g_serialLineLen == 0 && !beginsLineCommand) {
      switch (c) {
        case 'h':
        case '?':
        case 'p':
        case '1':
        case '2':
        case '3':
        case 'b':
        case 'a':
        case 'v':
        case 's':
        case 'c':
        case 'x':
        case 'e':
        case 'r':
        case 'z':
        case 'o':
        case 'k':
        case 'n':
        case 'i':
        case 'l':
        case '8':
        case '6':
          handleCommand(c);
          continue;
        case ' ':
        case '\t':
          continue;
        default:
          break;
      }
    }

    if (g_serialLineLen + 1 < SERIAL_LINE_BUFFER_SIZE) {
      g_serialLine[g_serialLineLen++] = c;
    } else {
      Serial.println("Command too long, clearing input buffer.");
      g_serialLineLen = 0;
    }
  }

  processIncomingFrames();
  serviceTargetStreaming();
}
