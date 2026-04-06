#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#include "lktech_motor_config.h"

// ESP32 + MCP2515 LKTech controller with software zero-offset logic for
// multiple motors on the same CAN bus.
//
// Fixed assumptions from working hardware tests:
// - CAN bitrate: 500 kbps
// - MCP2515 oscillator: 8 MHz
// - LKTech command mode: Multi Loop Angle Control 2 (0xA4)
//
// Control model per motor:
// 1. Physically place the output in the desired home position.
// 2. Send `on <joint>` to enable the motor and capture its current multi-loop
//    motor angle as software zero.
// 3. Command output-side angles with `goto <joint> <deg>`.
// 4. The sketch converts output degrees to motor-side multi-loop degrees using
//    the configured gear ratio.

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;

static constexpr uint8_t CMD_MOTOR_OFF = 0x80;
static constexpr uint8_t CMD_MOTOR_STOP = 0x81;
static constexpr uint8_t CMD_MOTOR_ON = 0x88;
static constexpr uint8_t CMD_READ_MULTI_LOOP_ANGLE = 0x92;
static constexpr uint8_t CMD_MULTI_LOOP_CONTROL_2 = 0xA4;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;

struct MotorRuntimeState {
  bool have_zero_offset;
  double zero_offset_motor_deg;
  double last_motor_angle_deg;
  bool motor_enabled;
  uint16_t motor_speed_dps;
};

MCP2515 g_mcp2515(MCP2515_CS_PIN);
MotorRuntimeState g_motor_states[kMotorConfigCount] = {};
size_t g_active_motor_index = kDefaultActiveMotorIndex;
char g_serial_line[160] = {};
size_t g_serial_line_len = 0;

const LKTechMotorConfig &motorConfig(size_t index) {
  return kMotorConfigs[index];
}

MotorRuntimeState &motorState(size_t index) {
  return g_motor_states[index];
}

uint32_t lktechRequestId(size_t index) {
  return kLKTechStdIdBase + motorConfig(index).motor_id;
}

double motorToOutputDegrees(size_t index, double motor_deg) {
  return (motor_deg - motorState(index).zero_offset_motor_deg) / motorConfig(index).gear_ratio;
}

double outputToMotorDegrees(size_t index, double output_deg) {
  return motorState(index).zero_offset_motor_deg + output_deg * motorConfig(index).gear_ratio;
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

int64_t readSigned56LE(const uint8_t *data) {
  uint64_t raw = 0;
  for (int i = 0; i < 7; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (8 * i);
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
    if (tolower(static_cast<unsigned char>(*left)) != tolower(static_cast<unsigned char>(*right))) {
      return false;
    }
    ++left;
    ++right;
  }

  return *left == '\0' && *right == '\0';
}

void initializeMotorStates() {
  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    MotorRuntimeState &state = motorState(index);
    state.have_zero_offset = false;
    state.zero_offset_motor_deg = 0.0;
    state.last_motor_angle_deg = 0.0;
    state.motor_enabled = false;
    state.motor_speed_dps = motorConfig(index).default_motor_speed_dps;
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
  for (uint8_t i = 0; i < frame.can_dlc; ++i) {
    Serial.printf("%02X", frame.data[i]);
    if (i + 1 < frame.can_dlc) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void printMotorStatus(size_t index) {
  const LKTechMotorConfig &config = motorConfig(index);
  const MotorRuntimeState &state = motorState(index);

  Serial.printf("%s name=%s motor_id=%u req_id=0x%03lX ratio=%.2f speed_dps_motor=%u speed_dps_output=%.2f enabled=%s zero=%s last_motor_deg=%.2f",
                index == g_active_motor_index ? "*" : " ",
                config.name,
                static_cast<unsigned>(config.motor_id),
                static_cast<unsigned long>(lktechRequestId(index)),
                config.gear_ratio,
                static_cast<unsigned>(state.motor_speed_dps),
                static_cast<double>(state.motor_speed_dps) / config.gear_ratio,
                state.motor_enabled ? "yes" : "no",
                state.have_zero_offset ? "yes" : "no",
                state.last_motor_angle_deg);
  if (state.have_zero_offset) {
    Serial.printf(" zero_offset_motor_deg=%.2f last_output_deg=%.2f",
                  state.zero_offset_motor_deg,
                  motorToOutputDegrees(index, state.last_motor_angle_deg));
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
  Serial.println("     list configured motors from lktech_motor_config.h");
  Serial.println("  use JOINT");
  Serial.println("     set the active motor, for example: use joint6 or use 15");
  Serial.println("  status [JOINT|all]");
  Serial.println("     print motor state summary");
  Serial.println("  on [JOINT|all]");
  Serial.println("     motor on + capture software zero from current position");
  Serial.println("  zero [JOINT|all]");
  Serial.println("     capture current motor angle as software zero");
  Serial.println("  pos [JOINT|all]");
  Serial.println("     read current motor position and print motor/output angles");
  Serial.println("  goto DEG");
  Serial.println("  goto JOINT DEG");
  Serial.println("     command output-side angle relative to software zero");
  Serial.println("  motor DEG");
  Serial.println("  motor JOINT DEG");
  Serial.println("     command absolute motor-side multi-loop angle directly");
  Serial.println("  speed N");
  Serial.println("  speed JOINT N");
  Serial.println("     set A4 motor-side speed in dps, for example: speed joint6 3600");
  Serial.println("  off [JOINT|all]");
  Serial.println("     send motor off");
  Serial.println("  stop [JOINT|all]");
  Serial.println("     send stop");
}

bool configureController() {
  g_mcp2515.reset();

  const MCP2515::ERROR bitrate_result = g_mcp2515.setBitrate(kFixedCanSpeed, kFixedCanClock);
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

bool sendFrame(uint32_t arbitration_id, const uint8_t *data, uint8_t dlc = 8) {
  can_frame frame = {};
  frame.can_id = arbitration_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d id=0x%03lX\n",
                  static_cast<int>(result),
                  static_cast<unsigned long>(arbitration_id));
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
      if (arbitration_id == lktechRequestId(index) &&
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

  Serial.printf("timeout waiting for reply cmd=0x%02X on id=0x%03lX (%s)\n",
                static_cast<unsigned>(expected_command),
                static_cast<unsigned long>(lktechRequestId(index)),
                motorConfig(index).name);
  return false;
}

bool sendSimpleCommandAndWait(size_t index, uint8_t command, can_frame *reply_frame = nullptr) {
  const uint8_t payload[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  drainReceiveQueue();
  if (!sendFrame(lktechRequestId(index), payload, sizeof(payload))) {
    return false;
  }
  return waitForReply(index, command, reply_frame);
}

bool readCurrentMotorAngle(size_t index, double *motor_angle_deg) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_READ_MULTI_LOOP_ANGLE, &reply)) {
    return false;
  }

  const int64_t angle_cdeg = readSigned56LE(&reply.data[1]);
  MotorRuntimeState &state = motorState(index);
  state.last_motor_angle_deg = centiDegreesToDegrees(angle_cdeg);
  if (motor_angle_deg != nullptr) {
    *motor_angle_deg = state.last_motor_angle_deg;
  }

  Serial.printf("%s current motor multi-loop angle = %.2f deg\n",
                motorConfig(index).name,
                state.last_motor_angle_deg);
  if (state.have_zero_offset) {
    Serial.printf("%s current output angle = %.2f deg (zero_offset_motor_deg=%.2f)\n",
                  motorConfig(index).name,
                  motorToOutputDegrees(index, state.last_motor_angle_deg),
                  state.zero_offset_motor_deg);
  }
  return true;
}

bool captureZeroOffset(size_t index) {
  double motor_angle_deg = 0.0;
  if (!readCurrentMotorAngle(index, &motor_angle_deg)) {
    return false;
  }

  MotorRuntimeState &state = motorState(index);
  state.zero_offset_motor_deg = motor_angle_deg;
  state.have_zero_offset = true;
  Serial.printf("%s software zero captured at motor_deg=%.2f, output zero is now 0.00 deg\n",
                motorConfig(index).name,
                state.zero_offset_motor_deg);
  return true;
}

bool sendMotorOnAndCaptureZero(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_ON, &reply)) {
    return false;
  }

  motorState(index).motor_enabled = true;
  Serial.printf("%s motor on acknowledged\n", motorConfig(index).name);
  delay(20);
  return captureZeroOffset(index);
}

bool sendMotorOff(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_OFF, &reply)) {
    return false;
  }
  motorState(index).motor_enabled = false;
  Serial.printf("%s motor off acknowledged\n", motorConfig(index).name);
  return true;
}

bool sendMotorStop(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_STOP, &reply)) {
    return false;
  }
  Serial.printf("%s motor stop acknowledged\n", motorConfig(index).name);
  return true;
}

bool commandMotorAngleDegrees(size_t index, double target_motor_deg) {
  MotorRuntimeState &state = motorState(index);
  const int32_t target_cdeg = degreesToCentiDegrees(target_motor_deg);
  const uint8_t payload[8] = {
      CMD_MULTI_LOOP_CONTROL_2,
      0x00,
      static_cast<uint8_t>(state.motor_speed_dps & 0xFF),
      static_cast<uint8_t>((state.motor_speed_dps >> 8) & 0xFF),
      static_cast<uint8_t>(target_cdeg & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 8) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 16) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 24) & 0xFF),
  };

  if (!sendFrame(lktechRequestId(index), payload, sizeof(payload))) {
    return false;
  }

  Serial.printf("%s A4 motor target_deg=%.2f speed_dps_motor=%u speed_dps_output=%.2f\n",
                motorConfig(index).name,
                target_motor_deg,
                static_cast<unsigned>(state.motor_speed_dps),
                static_cast<double>(state.motor_speed_dps) / motorConfig(index).gear_ratio);
  return true;
}

bool commandOutputAngleDegrees(size_t index, double target_output_deg) {
  const MotorRuntimeState &state = motorState(index);
  if (!state.have_zero_offset) {
    Serial.printf("%s software zero not captured yet, send `on %s` or `zero %s` first\n",
                  motorConfig(index).name,
                  motorConfig(index).name,
                  motorConfig(index).name);
    return false;
  }

  const double target_motor_deg = outputToMotorDegrees(index, target_output_deg);
  Serial.printf("%s output target_deg=%.2f -> motor target_deg=%.2f using zero_offset_motor_deg=%.2f ratio=%.2f\n",
                motorConfig(index).name,
                target_output_deg,
                target_motor_deg,
                state.zero_offset_motor_deg,
                motorConfig(index).gear_ratio);
  return commandMotorAngleDegrees(index, target_motor_deg);
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
  for (char *token = strtok(text, " \t"); token != nullptr && count < max_tokens; token = strtok(nullptr, " \t")) {
    tokens[count++] = token;
  }
  return count;
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
      if (motorConfig(index).motor_id == parsed) {
        *index_out = index;
        return true;
      }
    }
  }

  return false;
}

void printUnknownTarget(const char *token) {
  Serial.printf("unknown motor target: %s\n", token == nullptr ? "<null>" : token);
  Serial.println("use one of the names from `list`, for example: joint5 or joint6");
}

void runOnAllMotors(bool (*fn)(size_t)) {
  for (size_t index = 0; index < kMotorConfigCount; ++index) {
    fn(index);
    delay(10);
  }
}

bool onMotor(size_t index) {
  return sendMotorOnAndCaptureZero(index);
}

bool zeroMotor(size_t index) {
  return captureZeroOffset(index);
}

bool posMotor(size_t index) {
  return readCurrentMotorAngle(index, nullptr);
}

bool offMotor(size_t index) {
  return sendMotorOff(index);
}

bool stopMotor(size_t index) {
  return sendMotorStop(index);
}

void handleListCommand() {
  printAllMotorStatus();
}

void handleUseCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: use JOINT");
    return;
  }

  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }

  g_active_motor_index = index;
  Serial.printf("active motor set to %s (id=%u req_id=0x%03lX)\n",
                motorConfig(index).name,
                static_cast<unsigned>(motorConfig(index).motor_id),
                static_cast<unsigned long>(lktechRequestId(index)));
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
    Serial.printf("usage: %s [JOINT|all]\n", command_name);
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
    Serial.println("usage: speed N  or  speed JOINT N");
    return;
  }

  unsigned long parsed = 0;
  if (!parseUnsignedLongToken(value_token, &parsed) || parsed > 65535UL) {
    Serial.println("invalid speed, expected 0..65535 dps");
    return;
  }

  motorState(index).motor_speed_dps = static_cast<uint16_t>(parsed);
  Serial.printf("%s A4 motor speed set to %u dps, output-side equivalent is about %.2f dps\n",
                motorConfig(index).name,
                static_cast<unsigned>(motorState(index).motor_speed_dps),
                static_cast<double>(motorState(index).motor_speed_dps) / motorConfig(index).gear_ratio);
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
    Serial.println("usage: goto DEG  or  goto JOINT DEG");
    return;
  }

  double output_deg = 0.0;
  if (!parseDoubleToken(value_token, &output_deg)) {
    Serial.println("invalid goto angle, expected degrees like: goto joint6 45 or goto -30.5");
    return;
  }

  commandOutputAngleDegrees(index, output_deg);
}

void handleMotorCommand(int argc, char *argv[]) {
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
    Serial.println("usage: motor DEG  or  motor JOINT DEG");
    return;
  }

  double motor_deg = 0.0;
  if (!parseDoubleToken(value_token, &motor_deg)) {
    Serial.println("invalid motor angle, expected degrees like: motor joint6 360 or motor -120.5");
    return;
  }

  commandMotorAngleDegrees(index, motor_deg);
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
  if (equalsIgnoreCase(argv[0], "on")) {
    handleSimpleMotorAction(argc, argv, "on", onMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "zero")) {
    handleSimpleMotorAction(argc, argv, "zero", zeroMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "pos")) {
    handleSimpleMotorAction(argc, argv, "pos", posMotor);
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
  if (equalsIgnoreCase(argv[0], "motor")) {
    handleMotorCommand(argc, argv);
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
  Serial.println("ESP32 + MCP2515 LKTech multi-motor zero-offset controller starting...");
  Serial.printf("fixed bitrate=500 kbps oscillator=8 MHz configured_motors=%u\n",
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
