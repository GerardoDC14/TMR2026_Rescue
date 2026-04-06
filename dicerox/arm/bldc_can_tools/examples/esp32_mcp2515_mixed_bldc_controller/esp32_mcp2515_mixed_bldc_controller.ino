#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <math.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#include "mixed_bldc_motor_config.h"

// Unified three-motor controller for a mixed CAN bus:
// - LKTech joint5 on ID 14
// - LKTech joint6 on ID 15
// - ZE300 driver on address 1
//
// Shared user workflow:
// 1. Physically place each joint at the desired home pose.
// 2. Send `on all` or `zero all` to capture the current pose as software zero.
// 3. Command output-side angles with `goto <motor> <deg>`.
// 4. Use `speed <motor> <dps>` to set output-side speed.
//
// Important honesty notes:
// - LKTech uses A4 Multi Loop Angle Control 2 with per-command speed.
// - ZE300 has no explicit "motor on" command in this sketch; `on ze300`
//   applies the configured B2 position speed and captures software zero.
// - ZE300 position scaling is currently empirical. Even though the physical
//   reducer is 1:8, the driver behaved as if position commands were already
//   output-referenced, so this config uses position_command_ratio = 1.0.

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;

static constexpr uint8_t LK_CMD_MOTOR_OFF = 0x80;
static constexpr uint8_t LK_CMD_MOTOR_STOP = 0x81;
static constexpr uint8_t LK_CMD_MOTOR_ON = 0x88;
static constexpr uint8_t LK_CMD_READ_MULTI_LOOP_ANGLE = 0x92;
static constexpr uint8_t LK_CMD_MULTI_LOOP_CONTROL_2 = 0xA4;

static constexpr uint8_t ZE_CMD_READ_ABSOLUTE_ANGLES = 0xA3;
static constexpr uint8_t ZE_CMD_SET_POSITION_MAX_SPEED = 0xB2;
static constexpr uint8_t ZE_CMD_ABSOLUTE_POSITION = 0xC2;
static constexpr uint8_t ZE_CMD_RELATIVE_POSITION = 0xC3;
static constexpr uint8_t ZE_CMD_DISABLE_OUTPUT = 0xCF;

struct MotorRuntimeState {
  bool have_zero_offset;
  bool enabled;
  bool ze300_speed_applied;
  double zero_offset_command_deg;
  double last_command_deg;
  double output_speed_dps;
  int32_t last_native_counts;
};

MCP2515 g_mcp2515(MCP2515_CS_PIN);
MotorRuntimeState g_motor_states[kMotorConfigCount] = {};
size_t g_active_motor_index = kDefaultActiveMotorIndex;
char g_serial_line[192] = {};
size_t g_serial_line_len = 0;

const MixedMotorConfig &motorConfig(size_t index) {
  return kMotorConfigs[index];
}

MotorRuntimeState &motorState(size_t index) {
  return g_motor_states[index];
}

const char *driverName(MixedDriverType driver_type) {
  switch (driver_type) {
    case DRIVER_LKTECH:
      return "LKTech";
    case DRIVER_ZE300:
      return "ZE300";
    default:
      return "Unknown";
  }
}

uint16_t requestId(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  if (config.driver_type == DRIVER_LKTECH) {
    return static_cast<uint16_t>(kLKTechStdIdBase + config.device_id);
  }
  if (config.ze300_tagged_requests) {
    return static_cast<uint16_t>(kZE300TaggedRequestMask | config.device_id);
  }
  return config.device_id;
}

uint16_t replyId(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  if (config.driver_type == DRIVER_LKTECH) {
    return requestId(index);
  }
  return config.device_id;
}

double outputDegreesToCommandDegrees(size_t index, double output_deg) {
  return output_deg * motorConfig(index).position_command_ratio;
}

double commandDegreesToOutputDegrees(size_t index, double command_deg) {
  return command_deg / motorConfig(index).position_command_ratio;
}

double absoluteOutputToCommandDegrees(size_t index, double output_deg) {
  return motorState(index).zero_offset_command_deg + outputDegreesToCommandDegrees(index, output_deg);
}

double currentOutputDegrees(size_t index) {
  return commandDegreesToOutputDegrees(
      index, motorState(index).last_command_deg - motorState(index).zero_offset_command_deg);
}

double outputSpeedToCommandDps(size_t index, double output_dps) {
  return output_dps * motorConfig(index).speed_command_ratio;
}

uint16_t lkOutputSpeedToMotorDps(size_t index, double output_dps) {
  const double motor_dps = outputSpeedToCommandDps(index, output_dps);
  if (motor_dps <= 0.0) {
    return 0;
  }
  if (motor_dps >= 65535.0) {
    return 65535;
  }
  return static_cast<uint16_t>(motor_dps + 0.5);
}

uint32_t ze300OutputSpeedToMotorCentiRpm(size_t index, double output_dps) {
  const double command_dps = outputSpeedToCommandDps(index, output_dps);
  const double motor_rpm = command_dps / 6.0;
  const double scaled = motor_rpm * 100.0;
  if (scaled <= 0.0) {
    return 0;
  }
  if (scaled >= 2147483647.0) {
    return 2147483647UL;
  }
  return static_cast<uint32_t>(scaled + 0.5);
}

double ze300MotorCentiRpmToOutputDps(size_t index, uint32_t motor_centirpm) {
  const double motor_rpm = static_cast<double>(motor_centirpm) / 100.0;
  const double command_dps = motor_rpm * 6.0;
  return command_dps / motorConfig(index).speed_command_ratio;
}

double ze300CountsToCommandDegrees(int32_t counts) {
  return static_cast<double>(counts) * 360.0 / static_cast<double>(kZE300CountsPerRev);
}

int32_t ze300CommandDegreesToCounts(double command_deg) {
  const double scaled = command_deg * static_cast<double>(kZE300CountsPerRev) / 360.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

int32_t degreesToCentiDegrees(double degrees) {
  const double scaled = degrees * 100.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

double centiDegreesToDegrees(int64_t centi_degrees) {
  return static_cast<double>(centi_degrees) / 100.0;
}

uint16_t readU16LE(const uint8_t *data, uint8_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}

int32_t readI32LE(const uint8_t *data, uint8_t offset) {
  uint32_t raw = static_cast<uint32_t>(data[offset]) |
                 (static_cast<uint32_t>(data[offset + 1]) << 8) |
                 (static_cast<uint32_t>(data[offset + 2]) << 16) |
                 (static_cast<uint32_t>(data[offset + 3]) << 24);
  return static_cast<int32_t>(raw);
}

int64_t readSigned56LE(const uint8_t *data) {
  uint64_t raw = 0;
  for (int index = 0; index < 7; ++index) {
    raw |= static_cast<uint64_t>(data[index]) << (8 * index);
  }
  if (data[6] & 0x80) {
    raw |= 0xFF00000000000000ULL;
  }
  return static_cast<int64_t>(raw);
}

bool equalsIgnoreCase(const char *left, const char *right) {
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

char *trimWhitespace(char *text) {
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

bool parseUnsignedLongToken(const char *token, unsigned long *value) {
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

bool parseDoubleToken(const char *token, double *value) {
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

int tokenize(char *text, char *tokens[], int max_tokens) {
  int count = 0;
  for (char *token = strtok(text, " \t"); token != nullptr && count < max_tokens;
       token = strtok(nullptr, " \t")) {
    tokens[count++] = token;
  }
  return count;
}

void initializeMotorStates() {
  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    MotorRuntimeState &state = motorState(index);
    state.have_zero_offset = false;
    state.enabled = false;
    state.ze300_speed_applied = false;
    state.zero_offset_command_deg = 0.0;
    state.last_command_deg = 0.0;
    state.output_speed_dps = motorConfig(index).default_output_speed_dps;
    state.last_native_counts = 0;
  }
}

void printFrame(const can_frame &frame, const char *prefix) {
  const uint32_t arbitration_id = frame.can_id & CAN_SFF_MASK;
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const bool remote = (frame.can_id & CAN_RTR_FLAG) != 0;

  Serial.printf("%s id=0x%0*lX ext=%d rtr=%d dlc=%u data=",
                prefix,
                extended ? 8 : 3,
                static_cast<unsigned long>(arbitration_id),
                extended ? 1 : 0,
                remote ? 1 : 0,
                static_cast<unsigned>(frame.can_dlc));
  for (uint8_t index = 0; index < frame.can_dlc; ++index) {
    Serial.printf("%02X", frame.data[index]);
    if (index + 1 < frame.can_dlc) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void printMotorStatus(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  const MotorRuntimeState &state = motorState(index);

  Serial.printf("%s name=%s driver=%s device_id=%u req_id=0x%03X reply_id=0x%03X physical_ratio=%.2f pos_ratio=%.2f speed_ratio=%.2f speed_output=%.2f dps enabled=%s zero=%s last_cmd_deg=%.2f",
                index == g_active_motor_index ? "*" : " ",
                config.name,
                driverName(config.driver_type),
                static_cast<unsigned>(config.device_id),
                requestId(index),
                replyId(index),
                config.physical_gear_ratio,
                config.position_command_ratio,
                config.speed_command_ratio,
                state.output_speed_dps,
                state.enabled ? "yes" : "no",
                state.have_zero_offset ? "yes" : "no",
                state.last_command_deg);

  if (config.driver_type == DRIVER_LKTECH) {
    Serial.printf(" lk_motor_speed=%u dps",
                  static_cast<unsigned>(lkOutputSpeedToMotorDps(index, state.output_speed_dps)));
  } else {
    Serial.printf(" ze300_speed_limit=%.2f dps",
                  ze300MotorCentiRpmToOutputDps(
                      index, ze300OutputSpeedToMotorCentiRpm(index, state.output_speed_dps)));
  }

  if (state.have_zero_offset) {
    Serial.printf(" zero_cmd_deg=%.2f last_output_deg=%.2f",
                  state.zero_offset_command_deg,
                  currentOutputDegrees(index));
  }
  Serial.println();
}

void printAllMotorStatus() {
  Serial.println("Configured motors:");
  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    printMotorStatus(index);
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("     show this help");
  Serial.println("  list");
  Serial.println("     list configured motors");
  Serial.println("  use TARGET");
  Serial.println("     set active motor, for example: use joint6 or use ze300");
  Serial.println("  status [TARGET|all]");
  Serial.println("     print motor status summary");
  Serial.println("  pos [TARGET|all]");
  Serial.println("     read current position");
  Serial.println("  zero [TARGET|all]");
  Serial.println("     capture current position as software zero");
  Serial.println("  on [TARGET|all]");
  Serial.println("     LKTech: motor on + zero; ZE300: apply speed config + zero");
  Serial.println("  goto DEG");
  Serial.println("  goto TARGET DEG");
  Serial.println("     command output-side absolute angle relative to software zero");
  Serial.println("  speed DPS");
  Serial.println("  speed TARGET DPS");
  Serial.println("     set output-side speed in deg/s");
  Serial.println("  off [TARGET|all]");
  Serial.println("     LKTech: 0x80, ZE300: 0xCF");
  Serial.println("  stop [TARGET|all]");
  Serial.println("     LKTech: 0x81, ZE300: maps to 0xCF");
}

bool configureController() {
  g_mcp2515.reset();

  const MCP2515::ERROR bitrate_result = g_mcp2515.setBitrate(kMixedCanSpeed, kMixedCanClock);
  if (bitrate_result != MCP2515::ERROR_OK) {
    Serial.printf("setBitrate failed: %d\n", static_cast<int>(bitrate_result));
    return false;
  }

  const MCP2515::ERROR mode_result = g_mcp2515.setNormalMode();
  if (mode_result != MCP2515::ERROR_OK) {
    Serial.printf("setNormalMode failed: %d\n", static_cast<int>(mode_result));
    return false;
  }

  Serial.println("MCP2515 configured successfully.");
  printAllMotorStatus();
  return true;
}

bool sendFrame(uint16_t arbitration_id, const uint8_t *data, uint8_t dlc) {
  can_frame frame = {};
  frame.can_id = arbitration_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d id=0x%03X\n",
                  static_cast<int>(result),
                  arbitration_id);
    return false;
  }

  printFrame(frame, "TX");
  return true;
}

void drainReceiveQueue() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printFrame(frame, "RX(stale)");
  }
}

bool waitForReply(size_t index, uint8_t expected_command, can_frame *reply_frame) {
  const unsigned long start_ms = millis();
  can_frame frame = {};

  while (millis() - start_ms < REPLY_TIMEOUT_MS) {
    if (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      printFrame(frame, "RX");
      const uint32_t arbitration_id = frame.can_id & CAN_SFF_MASK;
      if (arbitration_id == replyId(index) &&
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
                replyId(index),
                motorConfig(index).name);
  return false;
}

bool sendSimpleCommandAndWait(size_t index, uint8_t command, can_frame *reply_frame = nullptr) {
  drainReceiveQueue();
  const uint8_t payload_lk[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t payload_ze[1] = {command};

  if (motorConfig(index).driver_type == DRIVER_LKTECH) {
    if (!sendFrame(requestId(index), payload_lk, sizeof(payload_lk))) {
      return false;
    }
  } else {
    if (!sendFrame(requestId(index), payload_ze, sizeof(payload_ze))) {
      return false;
    }
  }
  return waitForReply(index, command, reply_frame);
}

bool sendValueCommandAndWait(size_t index, uint8_t command, int32_t value, can_frame *reply_frame = nullptr) {
  drainReceiveQueue();
  uint8_t payload[8] = {
      command,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF),
      0x00,
      0x00,
      0x00,
  };

  const uint8_t dlc = motorConfig(index).driver_type == DRIVER_LKTECH ? 8 : 5;
  if (!sendFrame(requestId(index), payload, dlc)) {
    return false;
  }
  return waitForReply(index, command, reply_frame);
}

bool findMotorIndexByToken(const char *token, size_t *index_out) {
  if (token == nullptr || index_out == nullptr) {
    return false;
  }

  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    if (equalsIgnoreCase(token, motorConfig(index).name)) {
      *index_out = index;
      return true;
    }
  }

  unsigned long parsed = 0;
  if (parseUnsignedLongToken(token, &parsed)) {
    for (size_t index = 0; index < kMotorConfigCount; ++index) {
      if (motorConfig(index).device_id == parsed) {
        *index_out = index;
        return true;
      }
    }
  }
  return false;
}

void printUnknownTarget(const char *token) {
  Serial.printf("unknown motor target: %s\n", token == nullptr ? "<null>" : token);
  Serial.println("use one of the names from `list`, for example: joint5, joint6, or ze300");
}

bool readCurrentPosition(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  MotorRuntimeState &state = motorState(index);
  can_frame reply = {};

  if (config.driver_type == DRIVER_LKTECH) {
    if (!sendSimpleCommandAndWait(index, LK_CMD_READ_MULTI_LOOP_ANGLE, &reply)) {
      return false;
    }
    const int64_t angle_cdeg = readSigned56LE(&reply.data[1]);
    state.last_command_deg = centiDegreesToDegrees(angle_cdeg);
    Serial.printf("%s LKTech motor angle = %.2f deg\n", config.name, state.last_command_deg);
  } else {
    if (!sendSimpleCommandAndWait(index, ZE_CMD_READ_ABSOLUTE_ANGLES, &reply)) {
      return false;
    }
    if (reply.can_dlc != 7) {
      Serial.printf("%s unexpected A3 reply length: %u\n",
                    config.name,
                    static_cast<unsigned>(reply.can_dlc));
      return false;
    }
    const uint16_t single_turn_counts = readU16LE(reply.data, 1);
    const int32_t multi_turn_counts = readI32LE(reply.data, 3);
    state.last_native_counts = multi_turn_counts;
    state.last_command_deg = ze300CountsToCommandDegrees(multi_turn_counts);
    Serial.printf("%s ZE300 single=%.2f deg (%u counts) multi=%.2f deg (%ld counts)\n",
                  config.name,
                  ze300CountsToCommandDegrees(single_turn_counts),
                  static_cast<unsigned>(single_turn_counts),
                  state.last_command_deg,
                  static_cast<long>(multi_turn_counts));
  }

  if (state.have_zero_offset) {
    Serial.printf("%s current output angle = %.2f deg relative to software zero\n",
                  config.name,
                  currentOutputDegrees(index));
  }
  return true;
}

bool captureZeroOffset(size_t index) {
  if (!readCurrentPosition(index)) {
    return false;
  }

  MotorRuntimeState &state = motorState(index);
  state.zero_offset_command_deg = state.last_command_deg;
  state.have_zero_offset = true;
  Serial.printf("%s software zero captured at command_deg=%.2f, output zero is now 0.00 deg\n",
                motorConfig(index).name,
                state.zero_offset_command_deg);
  return true;
}

bool applyOutputSpeed(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  MotorRuntimeState &state = motorState(index);

  if (config.driver_type == DRIVER_LKTECH) {
    Serial.printf("%s LKTech output speed stored as %.2f dps -> motor speed %.2f dps\n",
                  config.name,
                  state.output_speed_dps,
                  outputSpeedToCommandDps(index, state.output_speed_dps));
    return true;
  }

  can_frame reply = {};
  const uint32_t motor_centirpm = ze300OutputSpeedToMotorCentiRpm(index, state.output_speed_dps);
  if (!sendValueCommandAndWait(
          index, ZE_CMD_SET_POSITION_MAX_SPEED, static_cast<int32_t>(motor_centirpm), &reply)) {
    return false;
  }

  if (reply.can_dlc != 5) {
    Serial.printf("%s unexpected B2 reply length: %u\n",
                  config.name,
                  static_cast<unsigned>(reply.can_dlc));
    return false;
  }

  const uint32_t applied_motor_centirpm = static_cast<uint32_t>(readI32LE(reply.data, 1));
  state.ze300_speed_applied = true;
  Serial.printf("%s ZE300 speed applied: output=%.2f dps command=%.2f dps motor=%.2f rpm\n",
                config.name,
                state.output_speed_dps,
                outputSpeedToCommandDps(index, state.output_speed_dps),
                static_cast<double>(applied_motor_centirpm) / 100.0);
  return true;
}

bool setOutputSpeed(size_t index, double output_dps) {
  if (output_dps < 0.0) {
    Serial.println("speed must be non-negative");
    return false;
  }
  motorState(index).output_speed_dps = output_dps;
  return applyOutputSpeed(index);
}

bool sendOn(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  MotorRuntimeState &state = motorState(index);

  if (config.driver_type == DRIVER_LKTECH) {
    can_frame reply = {};
    if (!sendSimpleCommandAndWait(index, LK_CMD_MOTOR_ON, &reply)) {
      return false;
    }
    state.enabled = true;
    Serial.printf("%s motor on acknowledged\n", config.name);
    delay(20);
    return captureZeroOffset(index);
  }

  // ZE300 does not expose an explicit "motor on" command in this controller.
  if (!applyOutputSpeed(index)) {
    return false;
  }
  state.enabled = true;
  Serial.printf("%s ZE300 ready: speed applied, capturing software zero next\n", config.name);
  delay(20);
  return captureZeroOffset(index);
}

bool sendOff(size_t index) {
  MotorRuntimeState &state = motorState(index);
  const MixedMotorConfig &config = motorConfig(index);
  can_frame reply = {};

  if (config.driver_type == DRIVER_LKTECH) {
    if (!sendSimpleCommandAndWait(index, LK_CMD_MOTOR_OFF, &reply)) {
      return false;
    }
  } else {
    if (!sendSimpleCommandAndWait(index, ZE_CMD_DISABLE_OUTPUT, &reply)) {
      return false;
    }
    state.ze300_speed_applied = false;
  }
  state.enabled = false;
  Serial.printf("%s output disabled\n", config.name);
  return true;
}

bool sendStop(size_t index) {
  const MixedMotorConfig &config = motorConfig(index);
  if (config.driver_type == DRIVER_LKTECH) {
    can_frame reply = {};
    if (!sendSimpleCommandAndWait(index, LK_CMD_MOTOR_STOP, &reply)) {
      return false;
    }
    Serial.printf("%s stop acknowledged\n", config.name);
    return true;
  }
  Serial.printf("%s ZE300 stop is mapped to output disable (0xCF)\n", config.name);
  return sendOff(index);
}

bool commandAbsoluteOutput(size_t index, double target_output_deg) {
  const MixedMotorConfig &config = motorConfig(index);
  const MotorRuntimeState &state = motorState(index);
  if (!state.have_zero_offset) {
    Serial.printf("%s software zero not captured yet, send `on %s` or `zero %s` first\n",
                  config.name,
                  config.name,
                  config.name);
    return false;
  }

  const double target_command_deg = absoluteOutputToCommandDegrees(index, target_output_deg);

  if (config.driver_type == DRIVER_LKTECH) {
    const uint16_t motor_speed_dps = lkOutputSpeedToMotorDps(index, motorState(index).output_speed_dps);
    const int32_t target_cdeg = degreesToCentiDegrees(target_command_deg);
    const uint8_t payload[8] = {
        LK_CMD_MULTI_LOOP_CONTROL_2,
        0x00,
        static_cast<uint8_t>(motor_speed_dps & 0xFF),
        static_cast<uint8_t>((motor_speed_dps >> 8) & 0xFF),
        static_cast<uint8_t>(target_cdeg & 0xFF),
        static_cast<uint8_t>((target_cdeg >> 8) & 0xFF),
        static_cast<uint8_t>((target_cdeg >> 16) & 0xFF),
        static_cast<uint8_t>((target_cdeg >> 24) & 0xFF),
    };

    if (!sendFrame(requestId(index), payload, sizeof(payload))) {
      return false;
    }
    motorState(index).last_command_deg = target_command_deg;
    Serial.printf("%s goto output=%.2f deg -> command=%.2f deg speed=%.2f output dps\n",
                  config.name,
                  target_output_deg,
                  target_command_deg,
                  motorState(index).output_speed_dps);
    return true;
  }

  if (!motorState(index).ze300_speed_applied) {
    if (!applyOutputSpeed(index)) {
      return false;
    }
  }

  const int32_t target_counts = ze300CommandDegreesToCounts(target_command_deg);
  can_frame reply = {};
  if (!sendValueCommandAndWait(index, ZE_CMD_ABSOLUTE_POSITION, target_counts, &reply)) {
    return false;
  }
  motorState(index).last_command_deg = target_command_deg;
  if (reply.can_dlc == 7) {
    motorState(index).last_native_counts = readI32LE(reply.data, 3);
    motorState(index).last_command_deg = ze300CountsToCommandDegrees(motorState(index).last_native_counts);
  }

  Serial.printf("%s goto output=%.2f deg -> command=%.2f deg -> counts=%ld speed=%.2f output dps\n",
                config.name,
                target_output_deg,
                target_command_deg,
                static_cast<long>(target_counts),
                motorState(index).output_speed_dps);
  return true;
}

void runOnAllMotors(bool (*fn)(size_t)) {
  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    fn(index);
    delay(10);
  }
}

bool onMotor(size_t index) {
  return sendOn(index);
}

bool zeroMotor(size_t index) {
  return captureZeroOffset(index);
}

bool posMotor(size_t index) {
  return readCurrentPosition(index);
}

bool offMotor(size_t index) {
  return sendOff(index);
}

bool stopMotor(size_t index) {
  return sendStop(index);
}

void handleListCommand() {
  printAllMotorStatus();
}

void handleUseCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: use TARGET");
    return;
  }
  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }
  g_active_motor_index = index;
  Serial.printf("active motor set to %s (%s, id=%u req_id=0x%03X)\n",
                motorConfig(index).name,
                driverName(motorConfig(index).driver_type),
                static_cast<unsigned>(motorConfig(index).device_id),
                requestId(index));
}

void handleStatusCommand(int argc, char *argv[]) {
  if (argc == 1 || (argc == 2 && equalsIgnoreCase(argv[1], "all"))) {
    printAllMotorStatus();
    return;
  }
  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }
  printMotorStatus(index);
}

void handleSimpleMotorAction(int argc, char *argv[], const char *command_name, bool (*fn)(size_t)) {
  if (argc == 1) {
    fn(g_active_motor_index);
    return;
  }
  if (argc != 2) {
    Serial.printf("usage: %s [TARGET|all]\n", command_name);
    return;
  }
  if (equalsIgnoreCase(argv[1], "all")) {
    runOnAllMotors(fn);
    return;
  }
  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }
  fn(index);
}

void handleSpeedCommand(int argc, char *argv[]) {
  size_t index = g_active_motor_index;
  const char *value_token = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findMotorIndexByToken(argv[1], &index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: speed DPS  or  speed TARGET DPS");
    return;
  }

  double output_dps = 0.0;
  if (!parseDoubleToken(value_token, &output_dps)) {
    Serial.println("invalid speed value, expected something like: speed joint6 180");
    return;
  }
  setOutputSpeed(index, output_dps);
}

void handleGotoCommand(int argc, char *argv[]) {
  size_t index = g_active_motor_index;
  const char *value_token = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findMotorIndexByToken(argv[1], &index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: goto DEG  or  goto TARGET DEG");
    return;
  }

  double output_deg = 0.0;
  if (!parseDoubleToken(value_token, &output_deg)) {
    Serial.println("invalid goto angle, expected something like: goto joint5 30 or goto -15");
    return;
  }
  commandAbsoluteOutput(index, output_deg);
}

void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  char *argv[4] = {};
  const int argc = tokenize(trimmed, argv, 4);
  if (argc <= 0) {
    return;
  }

  if (equalsIgnoreCase(argv[0], "help") || equalsIgnoreCase(argv[0], "h")) {
    printHelp();
    return;
  }
  if (equalsIgnoreCase(argv[0], "list")) {
    handleListCommand();
    return;
  }
  if (equalsIgnoreCase(argv[0], "use")) {
    handleUseCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "status")) {
    handleStatusCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "pos")) {
    handleSimpleMotorAction(argc, argv, "pos", posMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "zero")) {
    handleSimpleMotorAction(argc, argv, "zero", zeroMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "on")) {
    handleSimpleMotorAction(argc, argv, "on", onMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "off")) {
    handleSimpleMotorAction(argc, argv, "off", offMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "stop")) {
    handleSimpleMotorAction(argc, argv, "stop", stopMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "speed")) {
    handleSpeedCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "goto")) {
    handleGotoCommand(argc, argv);
    return;
  }

  Serial.printf("unknown command: %s\n", argv[0]);
  printHelp();
}

void processSerialInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      g_serial_line[g_serial_line_len] = '\0';
      handleSerialLine(g_serial_line);
      g_serial_line_len = 0;
      g_serial_line[0] = '\0';
      continue;
    }
    if (g_serial_line_len + 1 >= sizeof(g_serial_line)) {
      Serial.println("serial command too long, clearing buffer");
      g_serial_line_len = 0;
      g_serial_line[0] = '\0';
      continue;
    }
    g_serial_line[g_serial_line_len++] = ch;
    g_serial_line[g_serial_line_len] = '\0';
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 + MCP2515 mixed BLDC controller starting...");
  Serial.printf("fixed bitrate=1 Mbps oscillator=8 MHz configured_motors=%u\n",
                static_cast<unsigned>(kMotorConfigCount));

  initializeMotorStates();

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

  if (!configureController()) {
    Serial.println("initial MCP2515 configuration failed");
  }

  printHelp();
}

void loop() {
  processSerialInput();
  delay(5);
}
