#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <string.h>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif

#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000UL
#endif

#ifndef CAN_SFF_MASK
#define CAN_SFF_MASK 0x7FFUL
#endif

namespace odrive_velocity_protocol {

constexpr uint8_t kCmdHeartbeat               = 0x01;
constexpr uint8_t kCmdGetError                = 0x03;
constexpr uint8_t kCmdSetAxisState            = 0x07;
constexpr uint8_t kCmdGetEncoderEstimates     = 0x09;
constexpr uint8_t kCmdSetControllerMode       = 0x0B;
constexpr uint8_t kCmdSetInputVel             = 0x0D;
constexpr uint8_t kCmdGetBusVoltageCurrent    = 0x17;
constexpr uint8_t kCmdClearErrors             = 0x18;

constexpr uint32_t kAxisStateIdle              = 1;
constexpr uint32_t kAxisStateClosedLoopControl = 8;

constexpr uint32_t kControlModeVelocityControl = 2;
constexpr uint32_t kInputModeVelRamp           = 2;

struct ODriveState {
  bool haveHeartbeat = false;
  bool haveEncoder = false;
  bool haveBus = false;
  bool haveError = false;

  uint32_t heartbeatActiveErrors = 0;
  uint8_t axisState = 0;

  uint32_t activeErrors = 0;
  uint32_t disarmReason = 0;

  float posTurns = 0.0f;
  float velTurnsPerSecond = 0.0f;
  float busVoltage = 0.0f;
  float busCurrent = 0.0f;
  float lastCommandedVelTurnsPerSecond = 0.0f;

  unsigned long lastHeartbeatMs = 0;
  unsigned long lastEncoderMs = 0;
  unsigned long lastBusMs = 0;
  unsigned long lastErrorMs = 0;
};

inline uint32_t makeCanId(uint8_t nodeId, uint8_t cmdId) {
  return (static_cast<uint32_t>(nodeId) << 5) | (cmdId & 0x1F);
}

inline uint32_t getStandardId(const can_frame &frame) {
  return frame.can_id & CAN_SFF_MASK;
}

inline bool isExtendedFrame(const can_frame &frame) {
  return (frame.can_id & CAN_EFF_FLAG) != 0;
}

inline bool isRemoteFrame(const can_frame &frame) {
  return (frame.can_id & CAN_RTR_FLAG) != 0;
}

inline uint8_t extractNodeId(uint32_t standardId) {
  return static_cast<uint8_t>((standardId >> 5) & 0x3F);
}

inline uint8_t extractCommandId(uint32_t standardId) {
  return static_cast<uint8_t>(standardId & 0x1F);
}

inline uint32_t bytesToU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

inline float bytesToFloat(const uint8_t *data) {
  float value = 0.0f;
  memcpy(&value, data, sizeof(value));
  return value;
}

inline void writeU32ToBytes(uint32_t value, uint8_t *data) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline void writeFloatToBytes(float value, uint8_t *data) {
  memcpy(data, &value, sizeof(value));
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
    default: return "UNKNOWN";
  }
}

}  // namespace odrive_velocity_protocol
