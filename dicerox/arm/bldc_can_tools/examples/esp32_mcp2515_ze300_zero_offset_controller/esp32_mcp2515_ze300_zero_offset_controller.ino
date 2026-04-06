#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

// ZE300 position controller with software zero-offset logic.
//
// This sketch is intentionally separate from the reader:
// - fixed CAN timing: 1 Mbps, 8 MHz MCP2515 oscillator
// - fixed request mode: 0x100 | Dev_addr
// - fixed default device address: 1
// - fixed physical gear ratio: 1:8
//
// Workflow:
// 1. Physically place the output shaft in the desired "home" pose.
// 2. Send `zero` to read A3 and store the current multi-turn position as software zero.
// 3. Send `speed ...` to configure B2 position-mode max speed.
// 4. Send `goto ...` in output degrees, or `step ...` for relative output motion.
//
// ZE300 protocol notes from the PDF:
// - A3: read single-turn and multi-turn absolute angle in counts
// - B2: set position mode max speed in 0.01 rpm (motor-side)
// - C2: absolute position control in motor counts
// - C3: relative position control in motor counts
// - CF: disable output / free state
//
// Important empirical note from hardware testing:
// - With an assumed 1:8 position scaling, `goto 45` caused about 360 degrees
//   of physical output rotation.
// - That strongly suggests this driver's position units are already referenced
//   to output-side motion, or the driver internally applies the reducer ratio.
// - So this sketch keeps the physical gear ratio documented, but uses a direct
//   1:1 output-to-command position scale unless later tests prove otherwise.

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;

static constexpr CAN_SPEED FIXED_CAN_SPEED = CAN_1000KBPS;
static constexpr CAN_CLOCK FIXED_CAN_CLOCK = MCP_8MHZ;
static constexpr uint16_t ZE300_REQUEST_TAG_MASK = 0x100;
static constexpr uint16_t ZE300_COUNTS_PER_REV = 16384;
static constexpr uint8_t ZE300_DEVICE_ADDRESS = 1;
static constexpr double PHYSICAL_GEAR_RATIO = 8.0;
static constexpr double POSITION_COMMAND_RATIO = 1.0;
static constexpr double SPEED_COMMAND_RATIO = 1.0;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;

static constexpr uint8_t CMD_READ_ABSOLUTE_ANGLES = 0xA3;
static constexpr uint8_t CMD_SET_POSITION_MAX_SPEED = 0xB2;
static constexpr uint8_t CMD_ABSOLUTE_POSITION = 0xC2;
static constexpr uint8_t CMD_RELATIVE_POSITION = 0xC3;
static constexpr uint8_t CMD_DISABLE_OUTPUT = 0xCF;

static constexpr double DEFAULT_OUTPUT_SPEED_DPS = 15.0;

MCP2515 g_mcp2515(MCP2515_CS_PIN);
char g_serial_line[128] = {};
size_t g_serial_line_len = 0;
bool g_have_zero_offset = false;
int32_t g_zero_offset_motor_counts = 0;
int32_t g_last_motor_counts = 0;
uint32_t g_position_limit_centirpm = 0;

uint16_t requestId() {
  return static_cast<uint16_t>(ZE300_REQUEST_TAG_MASK | ZE300_DEVICE_ADDRESS);
}

uint16_t replyId() {
  return ZE300_DEVICE_ADDRESS;
}

int32_t readI32LE(const uint8_t *data, uint8_t offset) {
  uint32_t value = static_cast<uint32_t>(data[offset]) |
                   (static_cast<uint32_t>(data[offset + 1]) << 8) |
                   (static_cast<uint32_t>(data[offset + 2]) << 16) |
                   (static_cast<uint32_t>(data[offset + 3]) << 24);
  return static_cast<int32_t>(value);
}

uint16_t readU16LE(const uint8_t *data, uint8_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}

double countsToMotorDegrees(int32_t counts) {
  return static_cast<double>(counts) * 360.0 / static_cast<double>(ZE300_COUNTS_PER_REV);
}

int32_t motorDegreesToCounts(double motor_deg) {
  const double scaled = motor_deg * static_cast<double>(ZE300_COUNTS_PER_REV) / 360.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

double outputDegreesToMotorDegrees(double output_deg) {
  return output_deg * POSITION_COMMAND_RATIO;
}

double motorDegreesToOutputDegrees(double motor_deg) {
  return motor_deg / POSITION_COMMAND_RATIO;
}

int32_t outputDegreesToMotorCounts(double output_deg) {
  return motorDegreesToCounts(outputDegreesToMotorDegrees(output_deg));
}

double motorCountsToOutputDegrees(int32_t motor_counts) {
  return motorDegreesToOutputDegrees(countsToMotorDegrees(motor_counts));
}

uint32_t outputDpsToMotorCentiRpm(double output_dps) {
  const double motor_rpm = output_dps * SPEED_COMMAND_RATIO / 6.0;
  const double scaled = motor_rpm * 100.0;
  if (scaled <= 0.0) {
    return 0;
  }
  return static_cast<uint32_t>(scaled + 0.5);
}

double motorCentiRpmToOutputDps(uint32_t motor_centirpm) {
  const double motor_rpm = static_cast<double>(motor_centirpm) / 100.0;
  const double motor_dps = motor_rpm * 6.0;
  return motor_dps / SPEED_COMMAND_RATIO;
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

void printStatus() {
  Serial.printf("status device_address=%u request_id=0x%03X reply_id=0x%03X physical_ratio=%.2f position_scale=%.2f speed_scale=%.2f speed_limit_motor=%.2f rpm speed_limit_output=%.2f dps zero=%s last_motor_counts=%ld last_motor_deg=%.2f",
                static_cast<unsigned>(ZE300_DEVICE_ADDRESS),
                requestId(),
                replyId(),
                PHYSICAL_GEAR_RATIO,
                POSITION_COMMAND_RATIO,
                SPEED_COMMAND_RATIO,
                static_cast<double>(g_position_limit_centirpm) / 100.0,
                motorCentiRpmToOutputDps(g_position_limit_centirpm),
                g_have_zero_offset ? "yes" : "no",
                static_cast<long>(g_last_motor_counts),
                countsToMotorDegrees(g_last_motor_counts));
  if (g_have_zero_offset) {
    Serial.printf(" zero_offset_counts=%ld last_output_deg=%.2f",
                  static_cast<long>(g_zero_offset_motor_counts),
                  motorCountsToOutputDegrees(g_last_motor_counts - g_zero_offset_motor_counts));
  }
  Serial.println();
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("     show this help");
  Serial.println("  pos");
  Serial.println("     read current single-turn and multi-turn position");
  Serial.println("  zero");
  Serial.println("     capture current multi-turn position as software zero");
  Serial.println("  speed DPS");
  Serial.println("     set position-mode max speed in output-side deg/s");
  Serial.println("  speedrpm RPM");
  Serial.println("     set position-mode max speed directly in motor rpm");
  Serial.println("  goto DEG");
  Serial.println("     command absolute output-side angle relative to software zero");
  Serial.println("  step DEG");
  Serial.println("     command relative output-side motion");
  Serial.println("  motor DEG");
  Serial.println("     command absolute motor-side angle in degrees");
  Serial.println("  off");
  Serial.println("     disable motor output (0xCF)");
  Serial.println("  status");
  Serial.println("     print controller status");
  Serial.println("  note: this sketch currently uses direct 1:1 command scaling from output angle based on test results");
}

bool configureController() {
  g_mcp2515.reset();

  const MCP2515::ERROR bitrate_result = g_mcp2515.setBitrate(FIXED_CAN_SPEED, FIXED_CAN_CLOCK);
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
  printStatus();
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

bool waitForReply(uint8_t expected_command, can_frame *reply_frame) {
  const unsigned long start_ms = millis();
  can_frame frame = {};

  while (millis() - start_ms < REPLY_TIMEOUT_MS) {
    if (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      printFrame(frame, "RX");
      if ((frame.can_id & CAN_SFF_MASK) == replyId() &&
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

  Serial.printf("timeout waiting for reply cmd=0x%02X on id=0x%03X\n",
                static_cast<unsigned>(expected_command),
                replyId());
  return false;
}

bool sendOneByteCommandAndWait(uint8_t command, can_frame *reply_frame) {
  const uint8_t payload[1] = {command};
  drainReceiveQueue();
  if (!sendFrame(requestId(), payload, sizeof(payload))) {
    return false;
  }
  return waitForReply(command, reply_frame);
}

bool sendFiveByteCommandAndWait(uint8_t command, int32_t value, can_frame *reply_frame) {
  const uint8_t payload[5] = {
      command,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF),
  };
  drainReceiveQueue();
  if (!sendFrame(requestId(), payload, sizeof(payload))) {
    return false;
  }
  return waitForReply(command, reply_frame);
}

bool readCurrentMotorCounts(int32_t *multi_turn_counts_out) {
  can_frame reply = {};
  if (!sendOneByteCommandAndWait(CMD_READ_ABSOLUTE_ANGLES, &reply)) {
    return false;
  }

  if (reply.can_dlc != 7) {
    Serial.printf("unexpected A3 reply length: %u\n", static_cast<unsigned>(reply.can_dlc));
    return false;
  }

  const uint16_t single_turn_counts = readU16LE(reply.data, 1);
  const int32_t multi_turn_counts = readI32LE(reply.data, 3);
  g_last_motor_counts = multi_turn_counts;
  if (multi_turn_counts_out != nullptr) {
    *multi_turn_counts_out = multi_turn_counts;
  }

  Serial.printf("position single=%.2f deg (%u counts) multi=%.2f deg (%ld counts)\n",
                countsToMotorDegrees(single_turn_counts),
                static_cast<unsigned>(single_turn_counts),
                countsToMotorDegrees(multi_turn_counts),
                static_cast<long>(multi_turn_counts));
  if (g_have_zero_offset) {
    Serial.printf("output angle = %.2f deg relative to software zero\n",
                  motorCountsToOutputDegrees(multi_turn_counts - g_zero_offset_motor_counts));
  }
  return true;
}

bool captureZeroOffset() {
  int32_t motor_counts = 0;
  if (!readCurrentMotorCounts(&motor_counts)) {
    return false;
  }

  g_zero_offset_motor_counts = motor_counts;
  g_have_zero_offset = true;
  Serial.printf("software zero captured at motor_counts=%ld, output zero is now 0.00 deg\n",
                static_cast<long>(g_zero_offset_motor_counts));
  return true;
}

bool setPositionSpeedLimitFromMotorCentiRpm(uint32_t motor_centirpm) {
  if (motor_centirpm > 0x7FFFFFFF) {
    Serial.println("speed limit too large for B2 payload");
    return false;
  }

  can_frame reply = {};
  if (!sendFiveByteCommandAndWait(CMD_SET_POSITION_MAX_SPEED, static_cast<int32_t>(motor_centirpm), &reply)) {
    return false;
  }

  if (reply.can_dlc != 5) {
    Serial.printf("unexpected B2 reply length: %u\n", static_cast<unsigned>(reply.can_dlc));
    return false;
  }

  g_position_limit_centirpm = static_cast<uint32_t>(readI32LE(reply.data, 1));
  Serial.printf("position speed limit set: motor=%.2f rpm output=%.2f dps\n",
                static_cast<double>(g_position_limit_centirpm) / 100.0,
                motorCentiRpmToOutputDps(g_position_limit_centirpm));
  return true;
}

bool setPositionSpeedLimitFromOutputDps(double output_dps) {
  return setPositionSpeedLimitFromMotorCentiRpm(outputDpsToMotorCentiRpm(output_dps));
}

bool commandAbsoluteMotorCounts(int32_t motor_counts) {
  can_frame reply = {};
  if (!sendFiveByteCommandAndWait(CMD_ABSOLUTE_POSITION, motor_counts, &reply)) {
    return false;
  }

  if (reply.can_dlc == 7) {
    g_last_motor_counts = readI32LE(reply.data, 3);
    Serial.printf("C2 acknowledged, current multi-turn motor position now %.2f deg (%ld counts)\n",
                  countsToMotorDegrees(g_last_motor_counts),
                  static_cast<long>(g_last_motor_counts));
  }
  return true;
}

bool commandRelativeMotorCounts(int32_t motor_delta_counts) {
  can_frame reply = {};
  if (!sendFiveByteCommandAndWait(CMD_RELATIVE_POSITION, motor_delta_counts, &reply)) {
    return false;
  }

  if (reply.can_dlc == 7) {
    g_last_motor_counts = readI32LE(reply.data, 3);
    Serial.printf("C3 acknowledged, current multi-turn motor position now %.2f deg (%ld counts)\n",
                  countsToMotorDegrees(g_last_motor_counts),
                  static_cast<long>(g_last_motor_counts));
  }
  return true;
}

bool commandAbsoluteMotorDegrees(double motor_deg) {
  const int32_t motor_counts = motorDegreesToCounts(motor_deg);
  Serial.printf("C2 target motor_deg=%.2f -> motor_counts=%ld\n",
                motor_deg,
                static_cast<long>(motor_counts));
  return commandAbsoluteMotorCounts(motor_counts);
}

bool commandAbsoluteOutputDegrees(double output_deg) {
  if (!g_have_zero_offset) {
    Serial.println("software zero not captured yet, send `zero` first");
    return false;
  }

  const int32_t target_motor_counts = g_zero_offset_motor_counts + outputDegreesToMotorCounts(output_deg);
  Serial.printf("goto output_deg=%.2f -> target_motor_counts=%ld target_motor_deg=%.2f using ratio=%.2f\n",
                output_deg,
                static_cast<long>(target_motor_counts),
                countsToMotorDegrees(target_motor_counts),
                POSITION_COMMAND_RATIO);
  return commandAbsoluteMotorCounts(target_motor_counts);
}

bool commandRelativeOutputDegrees(double output_delta_deg) {
  const int32_t motor_delta_counts = outputDegreesToMotorCounts(output_delta_deg);
  Serial.printf("step output_deg=%.2f -> relative_motor_counts=%ld relative_motor_deg=%.2f using ratio=%.2f\n",
                output_delta_deg,
                static_cast<long>(motor_delta_counts),
                countsToMotorDegrees(motor_delta_counts),
                POSITION_COMMAND_RATIO);
  return commandRelativeMotorCounts(motor_delta_counts);
}

bool disableOutput() {
  can_frame reply = {};
  if (!sendOneByteCommandAndWait(CMD_DISABLE_OUTPUT, &reply)) {
    return false;
  }

  Serial.println("motor output disabled (free state)");
  return true;
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

void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  if (strcmp(trimmed, "help") == 0 || strcmp(trimmed, "h") == 0) {
    printHelp();
    return;
  }
  if (strcmp(trimmed, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(trimmed, "pos") == 0) {
    readCurrentMotorCounts(nullptr);
    return;
  }
  if (strcmp(trimmed, "zero") == 0) {
    captureZeroOffset();
    return;
  }
  if (strcmp(trimmed, "off") == 0) {
    disableOutput();
    return;
  }

  if (strncmp(trimmed, "speedrpm", 8) == 0) {
    char *value = trimWhitespace(trimmed + 8);
    double motor_rpm = 0.0;
    if (!parseDoubleToken(value, &motor_rpm)) {
      Serial.println("invalid speedrpm value, expected something like: speedrpm 300");
      return;
    }
    if (motor_rpm < 0.0) {
      Serial.println("speedrpm must be non-negative");
      return;
    }
    setPositionSpeedLimitFromMotorCentiRpm(static_cast<uint32_t>(motor_rpm * 100.0 + 0.5));
    return;
  }

  if (strncmp(trimmed, "speed", 5) == 0) {
    char *value = trimWhitespace(trimmed + 5);
    double output_dps = 0.0;
    if (!parseDoubleToken(value, &output_dps)) {
      Serial.println("invalid speed value, expected something like: speed 180");
      return;
    }
    if (output_dps < 0.0) {
      Serial.println("speed must be non-negative");
      return;
    }
    setPositionSpeedLimitFromOutputDps(output_dps);
    return;
  }

  if (strncmp(trimmed, "goto", 4) == 0) {
    char *value = trimWhitespace(trimmed + 4);
    double output_deg = 0.0;
    if (!parseDoubleToken(value, &output_deg)) {
      Serial.println("invalid goto value, expected something like: goto 45");
      return;
    }
    commandAbsoluteOutputDegrees(output_deg);
    return;
  }

  if (strncmp(trimmed, "step", 4) == 0) {
    char *value = trimWhitespace(trimmed + 4);
    double output_deg = 0.0;
    if (!parseDoubleToken(value, &output_deg)) {
      Serial.println("invalid step value, expected something like: step -10");
      return;
    }
    commandRelativeOutputDegrees(output_deg);
    return;
  }

  if (strncmp(trimmed, "motor", 5) == 0) {
    char *value = trimWhitespace(trimmed + 5);
    double motor_deg = 0.0;
    if (!parseDoubleToken(value, &motor_deg)) {
      Serial.println("invalid motor value, expected something like: motor 720");
      return;
    }
    commandAbsoluteMotorDegrees(motor_deg);
    return;
  }

  Serial.printf("unknown command: %s\n", trimmed);
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
  Serial.println("ESP32 + MCP2515 ZE300 zero-offset controller starting...");

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

  if (!configureController()) {
    Serial.println("initial MCP2515 configuration failed");
  }

  setPositionSpeedLimitFromOutputDps(DEFAULT_OUTPUT_SPEED_DPS);
  printHelp();
}

void loop() {
  processSerialInput();
  delay(5);
}
