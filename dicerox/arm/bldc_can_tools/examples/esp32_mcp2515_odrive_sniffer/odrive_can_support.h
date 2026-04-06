#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <string.h>

namespace odrive_can_sniffer {

constexpr uint8_t PIN_CAN_CS   = 5;
constexpr uint8_t PIN_CAN_INT  = 4;
constexpr uint8_t PIN_CAN_SCK  = 18;
constexpr uint8_t PIN_CAN_MISO = 19;
constexpr uint8_t PIN_CAN_MOSI = 23;

constexpr uint8_t NODE_ID_10 = 0x10;
constexpr uint8_t NODE_ID_11 = 0x11;
constexpr uint8_t NODE_ID_12 = 0x12;
constexpr uint8_t DEFAULT_ODRIVE_NODE_ID = NODE_ID_12;
constexpr size_t SUPPORTED_NODE_COUNT = 3;
constexpr uint8_t SUPPORTED_NODE_IDS[SUPPORTED_NODE_COUNT] = {
  NODE_ID_10,
  NODE_ID_11,
  NODE_ID_12,
};

constexpr uint8_t CMD_HEARTBEAT             = 0x01;
constexpr uint8_t CMD_GET_ERROR             = 0x03;
constexpr uint8_t CMD_SET_AXIS_STATE        = 0x07;
constexpr uint8_t CMD_GET_ENCODER_ESTIMATES = 0x09;
constexpr uint8_t CMD_SET_INPUT_POS         = 0x0C;

constexpr uint32_t AXIS_STATE_IDLE                = 1;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

constexpr uint16_t MAX_RX_FRAMES_PER_PASS = 64;
constexpr size_t SERIAL_LINE_BUFFER_SIZE = 64;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint32_t TARGET_STREAM_INTERVAL_MS = 50;
constexpr uint32_t ENCODER_TELEMETRY_INTERVAL_MS = 100;
constexpr uint32_t CLOSED_LOOP_CONFIRM_TIMEOUT_MS = 600;
constexpr uint32_t CLOSED_LOOP_RETRY_INTERVAL_MS = 100;
constexpr uint32_t ENCODER_WAIT_TIMEOUT_MS = 400;
constexpr uint8_t BRINGUP_MAX_ATTEMPTS = 3;
constexpr uint32_t BRINGUP_RETRY_SETTLE_MS = 40;
constexpr float DEFAULT_GEAR_RATIO = 48.0f;
constexpr float DEFAULT_DIRECTION = -1.0f;

enum class CanMode : uint8_t {
  Normal,
  ListenOnly,
  Loopback
};

enum class Oscillator : uint8_t {
  MHz8,
  MHz16
};

struct NodeRuntimeState {
  uint8_t nodeId;
  bool haveLatestEncoderEstimate;
  bool haveZeroReference;
  bool haveActiveTarget;
  bool haveHeartbeat;
  bool haveErrorStatus;
  float lastEncoderPosTurns;
  float lastEncoderVelTurnsPerSecond;
  float zeroReferenceTurns;
  float lastRelativeCommandTurns;
  float lastRelativeCommandDegrees;
  float lastAbsoluteCommandTurns;
  uint8_t lastAxisState;
  uint32_t lastHeartbeatActiveErrors;
  uint32_t lastErrorActiveErrors;
  uint32_t lastDisarmReason;
  unsigned long lastTargetStreamMs;
  unsigned long lastHeartbeatMs;
  unsigned long lastErrorMs;
  unsigned long lastEncoderEstimateMs;
  unsigned long lastEncoderTelemetryPrintMs;
};

inline uint32_t makeCanId(uint8_t nodeId, uint8_t commandId) {
  return (static_cast<uint32_t>(nodeId) << 5) | (commandId & 0x1F);
}

inline uint8_t extractNodeId(uint32_t standardId) {
  return static_cast<uint8_t>((standardId >> 5) & 0x3F);
}

inline uint8_t extractCommandId(uint32_t standardId) {
  return static_cast<uint8_t>(standardId & 0x1F);
}

inline bool isExtendedFrame(const can_frame &frame) {
  return (frame.can_id & CAN_EFF_FLAG) != 0;
}

inline bool isRemoteFrame(const can_frame &frame) {
  return (frame.can_id & CAN_RTR_FLAG) != 0;
}

inline uint32_t getStandardId(const can_frame &frame) {
  return frame.can_id & 0x7FFUL;
}

inline float bytesToFloat(const uint8_t *data) {
  float value = 0.0f;
  memcpy(&value, data, sizeof(value));
  return value;
}

inline uint32_t bytesToU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

inline void writeFloatToBytes(float value, uint8_t *data) {
  memcpy(data, &value, sizeof(value));
}

inline void writeI16ToBytes(int16_t value, uint8_t *data) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline float nodeGearRatio(uint8_t nodeId) {
  (void)nodeId;
  return DEFAULT_GEAR_RATIO;
}

inline float nodeDirection(uint8_t nodeId) {
  (void)nodeId;
  return DEFAULT_DIRECTION;
}

inline float jointDegreesToMotorTurns(uint8_t nodeId, float jointDegrees) {
  return (jointDegrees / 360.0f) * nodeGearRatio(nodeId) * nodeDirection(nodeId);
}

inline float motorTurnsToJointDegrees(uint8_t nodeId, float motorTurns) {
  const float scale = nodeGearRatio(nodeId) * nodeDirection(nodeId);
  if (scale == 0.0f) {
    return 0.0f;
  }
  return (motorTurns / scale) * 360.0f;
}

inline bool nodeIsInClosedLoop(const NodeRuntimeState &state) {
  const unsigned long now = millis();
  return state.haveHeartbeat &&
         now - state.lastHeartbeatMs <= 500 &&
         state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
         state.lastHeartbeatActiveErrors == 0;
}

inline bool nodeIsReadyForMotion(const NodeRuntimeState &state) {
  return nodeIsInClosedLoop(state) &&
         state.haveLatestEncoderEstimate &&
         millis() - state.lastEncoderEstimateMs <= 500 &&
         state.haveZeroReference;
}

inline const char *modeName(CanMode mode) {
  switch (mode) {
    case CanMode::Normal:     return "normal";
    case CanMode::ListenOnly: return "listen-only";
    case CanMode::Loopback:   return "loopback";
    default:                  return "unknown";
  }
}

inline const char *oscillatorName(Oscillator osc) {
  switch (osc) {
    case Oscillator::MHz8:  return "8 MHz";
    case Oscillator::MHz16: return "16 MHz";
    default:                return "unknown";
  }
}

inline const char *commandName(uint8_t cmd) {
  switch (cmd) {
    case CMD_HEARTBEAT:             return "Heartbeat";
    case CMD_GET_ERROR:             return "Get_Error";
    case CMD_SET_AXIS_STATE:        return "Set_Axis_State";
    case CMD_GET_ENCODER_ESTIMATES: return "Get_Encoder_Estimates";
    case CMD_SET_INPUT_POS:         return "Set_Input_Pos";
    default:                        return "Unknown";
  }
}

inline const char *axisStateName(uint8_t state) {
  switch (state) {
    case 0:  return "UNDEFINED";
    case 1:  return "IDLE";
    case 2:  return "STARTUP_SEQUENCE";
    case 3:  return "FULL_CALIBRATION_SEQUENCE";
    case 4:  return "MOTOR_CALIBRATION";
    case 5:  return "ENCODER_INDEX_SEARCH";
    case 6:  return "ENCODER_OFFSET_CALIBRATION";
    case 7:  return "CLOSED_LOOP_CONTROL_OLD";
    case 8:  return "CLOSED_LOOP_CONTROL";
    case 9:  return "LOCKIN_SPIN";
    case 10: return "ENCODER_DIR_FIND";
    case 11: return "HOMING";
    case 12: return "ENCODER_HALL_POLARITY_CALIBRATION";
    case 13: return "ENCODER_HALL_PHASE_CALIBRATION";
    default: return "UNKNOWN_STATE";
  }
}

}  // namespace odrive_can_sniffer
