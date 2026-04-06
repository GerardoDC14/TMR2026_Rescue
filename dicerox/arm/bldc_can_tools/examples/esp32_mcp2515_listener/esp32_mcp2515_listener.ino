#include <Arduino.h>
#include <ctype.h>
#include <SPI.h>
#include <stdlib.h>
#include <string.h>
#include <mcp2515.h>

// ESP32 + MCP2515 listener / sniffer for LKTech motor debugging.
//
// Library:
//   autowp / arduino-mcp2515
//
// Default wiring:
//   ESP32 GPIO23 -> MCP2515 SI
//   ESP32 GPIO19 -> MCP2515 SO
//   ESP32 GPIO18 -> MCP2515 SCK
//   ESP32 GPIO5  -> MCP2515 CS
//   ESP32 GPIO4  -> MCP2515 INT (optional in this sketch)
//
// Important note:
// - Default mode is NORMAL, not listen-only.
// - If your bus only has the motor and this MCP2515 node, NORMAL mode is often
//   needed so the MCP2515 will ACK frames transmitted by the motor.
// - LISTEN-ONLY is useful only when another node on the bus is already ACKing.

static constexpr uint32_t SERIAL_BAUD = 115200;

static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;
static constexpr uint32_t TEST_ARBITRATION_ID = 0x123;
static constexpr uint8_t TEST_DATA[] = {0xDE, 0xAD, 0xBE, 0xEF};
static constexpr uint16_t LKTECH_STD_ID_BASE = 0x140;
static constexpr uint8_t CMD_MOTOR_OFF = 0x80;
static constexpr uint8_t CMD_MOTOR_STOP = 0x81;
static constexpr uint8_t CMD_MOTOR_ON = 0x88;
static constexpr uint8_t CMD_READ_MULTI_LOOP_ANGLE = 0x92;
static constexpr uint8_t CMD_READ_STATE_1 = 0x9A;
static constexpr uint8_t CMD_READ_STATE_2 = 0x9C;
static constexpr uint8_t CMD_READ_STATE_3 = 0x9D;
static constexpr uint8_t CMD_MULTI_LOOP_CONTROL_2 = 0xA4;

enum SketchMode {
  MODE_NORMAL,
  MODE_LISTEN_ONLY,
  MODE_LOOPBACK,
};

MCP2515 g_mcp2515(MCP2515_CS_PIN);
CAN_SPEED g_can_speed = CAN_500KBPS;
CAN_CLOCK g_can_clock = MCP_8MHZ;
SketchMode g_mode = MODE_NORMAL;
uint8_t g_motor_id = 1;
uint16_t g_a4_speed_dps = 3600;
int32_t g_a4_target_cdeg = 18000;
bool g_have_last_multi_loop_angle = false;
int64_t g_last_multi_loop_angle_cdeg = 0;
char g_serial_line[128] = {};
size_t g_serial_line_len = 0;

const char *modeToString(SketchMode mode) {
  switch (mode) {
    case MODE_NORMAL:
      return "normal";
    case MODE_LISTEN_ONLY:
      return "listen-only";
    case MODE_LOOPBACK:
      return "loopback";
    default:
      return "unknown";
  }
}

const char *speedToString(CAN_SPEED speed) {
  switch (speed) {
    case CAN_500KBPS:
      return "500 kbps";
    case CAN_1000KBPS:
      return "1 Mbps";
    default:
      return "other";
  }
}

const char *clockToString(CAN_CLOCK clock_value) {
  switch (clock_value) {
    case MCP_8MHZ:
      return "8 MHz";
    case MCP_16MHZ:
      return "16 MHz";
    case MCP_20MHZ:
      return "20 MHz";
    default:
      return "unknown";
  }
}

double centiDegreesToDegrees(int32_t centi_degrees) {
  return static_cast<double>(centi_degrees) / 100.0;
}

int32_t degreesToCentiDegrees(double degrees) {
  const double scaled = degrees * 100.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

uint16_t readU16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int16_t readI16LE(const uint8_t *data) {
  return static_cast<int16_t>(readU16LE(data));
}

uint32_t readU32LE(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t readI32LE(const uint8_t *data) {
  return static_cast<int32_t>(readU32LE(data));
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

uint32_t lktechRequestId(uint8_t motor_id) {
  return LKTECH_STD_ID_BASE + motor_id;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h  - help");
  Serial.println("  p  - print current config");
  Serial.println("  r  - reconfigure MCP2515 with current settings");
  Serial.println("  s  - send 123#DEADBEEF immediately");
  Serial.println("  q  - send LKTech read multi-loop angle (0x92)");
  Serial.println("  1  - send LKTech read state 1 (0x9A)");
  Serial.println("  2  - send LKTech read state 2 (0x9C)");
  Serial.println("  3  - send LKTech read state 3 (0x9D)");
  Serial.println("  o  - send LKTech motor on (0x88)");
  Serial.println("  x  - send LKTech motor off (0x80)");
  Serial.println("  t  - send LKTech stop (0x81)");
  Serial.println("  j  - send LKTech A4 +180 deg with current A4 speed");
  Serial.println("  k  - send LKTech A4 -180 deg with current A4 speed");
  Serial.println("  move DEG");
  Serial.println("     send LKTech A4 to absolute multi-loop angle DEG");
  Serial.println("  step DEG");
  Serial.println("     send LKTech A4 to last_measured_angle_deg + DEG");
  Serial.println("  spd N");
  Serial.println("     set LKTech A4 speed in dps, for example: spd 3600");
  Serial.println("  5  - set 500 kbps");
  Serial.println("  m  - set 1 Mbps");
  Serial.println("  8  - set MCP2515 oscillator to 8 MHz");
  Serial.println("  6  - set MCP2515 oscillator to 16 MHz");
  Serial.println("  n  - normal mode");
  Serial.println("  i  - listen-only mode");
  Serial.println("  l  - loopback mode");
  Serial.println("  id N");
  Serial.println("     set LKTech motor ID to N (0..255)");
  Serial.println("  raw XX XX XX XX XX XX XX XX");
  Serial.println("     send arbitrary 8-byte payload to 0x140 + current motor ID");
}

void printConfig() {
  Serial.printf("config bitrate=%s oscillator=%s mode=%s motor_id=%u req_id=0x%03lX a4_speed_dps=%u a4_target_deg=%.2f last_multi_loop=%s%.2f cs=%u int=%u sck=%u miso=%u mosi=%u\n",
                speedToString(g_can_speed),
                clockToString(g_can_clock),
                modeToString(g_mode),
                static_cast<unsigned>(g_motor_id),
                static_cast<unsigned long>(lktechRequestId(g_motor_id)),
                static_cast<unsigned>(g_a4_speed_dps),
                centiDegreesToDegrees(g_a4_target_cdeg),
                g_have_last_multi_loop_angle ? "" : "unknown/",
                g_have_last_multi_loop_angle ? static_cast<double>(g_last_multi_loop_angle_cdeg) / 100.0 : 0.0,
                static_cast<unsigned>(MCP2515_CS_PIN),
                static_cast<unsigned>(MCP2515_INT_PIN),
                static_cast<unsigned>(SPI_SCK_PIN),
                static_cast<unsigned>(SPI_MISO_PIN),
                static_cast<unsigned>(SPI_MOSI_PIN));
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

void sendTestFrame() {
  sendFrame(TEST_ARBITRATION_ID, TEST_DATA, sizeof(TEST_DATA));
}

bool configureController() {
  g_mcp2515.reset();

  const MCP2515::ERROR bitrate_result = g_mcp2515.setBitrate(g_can_speed, g_can_clock);
  if (bitrate_result != MCP2515::ERROR_OK) {
    Serial.printf("setBitrate failed: %d\n", static_cast<int>(bitrate_result));
    return false;
  }

  MCP2515::ERROR mode_result = MCP2515::ERROR_OK;
  switch (g_mode) {
    case MODE_NORMAL:
      mode_result = g_mcp2515.setNormalMode();
      break;
    case MODE_LISTEN_ONLY:
      mode_result = g_mcp2515.setListenOnlyMode();
      break;
    case MODE_LOOPBACK:
      mode_result = g_mcp2515.setLoopbackMode();
      break;
  }

  if (mode_result != MCP2515::ERROR_OK) {
    Serial.printf("setMode failed: %d\n", static_cast<int>(mode_result));
    return false;
  }

  Serial.println("MCP2515 configured successfully.");
  printConfig();
  return true;
}

void printLKTechDecode(const can_frame &frame) {
  const uint32_t arbitration_id = frame.can_id & CAN_SFF_MASK;
  if (arbitration_id < 0x140 || arbitration_id > 0x23F || frame.can_dlc != 8) {
    return;
  }

  const uint8_t motor_id = static_cast<uint8_t>(arbitration_id - 0x140);
  const uint8_t cmd = frame.data[0];
  Serial.printf("  LKTech motor_id=%u cmd=0x%02X", static_cast<unsigned>(motor_id), static_cast<unsigned>(cmd));

  switch (cmd) {
    case 0x92: {
      const int64_t angle_cdeg = readSigned56LE(&frame.data[1]);
      g_have_last_multi_loop_angle = true;
      g_last_multi_loop_angle_cdeg = angle_cdeg;
      Serial.printf(" multi_loop_angle_deg=%.2f", static_cast<double>(angle_cdeg) / 100.0);
      break;
    }
    case 0x94: {
      const uint32_t angle_cdeg = readU32LE(&frame.data[4]);
      Serial.printf(" single_loop_angle_deg=%.2f", static_cast<double>(angle_cdeg) / 100.0);
      break;
    }
    case 0x9A: {
      const int8_t temperature_c = static_cast<int8_t>(frame.data[1]);
      const uint16_t voltage_centi_v = readU16LE(&frame.data[2]);
      const int16_t current_centi_a = readI16LE(&frame.data[4]);
      Serial.printf(" state1 temp_c=%d voltage_v=%.2f current_a=%.2f",
                    static_cast<int>(temperature_c),
                    static_cast<double>(voltage_centi_v) / 100.0,
                    static_cast<double>(current_centi_a) / 100.0);
      break;
    }
    case 0x9C: {
      const int8_t temperature_c = static_cast<int8_t>(frame.data[1]);
      const int16_t iq_raw = readI16LE(&frame.data[2]);
      const int16_t speed_dps = readI16LE(&frame.data[4]);
      const uint16_t encoder = readU16LE(&frame.data[6]);
      Serial.printf(" state2 temp_c=%d iq_raw=%d speed_dps=%d encoder=%u",
                    static_cast<int>(temperature_c),
                    static_cast<int>(iq_raw),
                    static_cast<int>(speed_dps),
                    static_cast<unsigned>(encoder));
      break;
    }
    case 0x9D: {
      const int8_t temperature_c = static_cast<int8_t>(frame.data[1]);
      const int16_t ia = readI16LE(&frame.data[2]);
      const int16_t ib = readI16LE(&frame.data[4]);
      const int16_t ic = readI16LE(&frame.data[6]);
      Serial.printf(" state3 temp_c=%d ia_raw=%d ib_raw=%d ic_raw=%d",
                    static_cast<int>(temperature_c),
                    static_cast<int>(ia),
                    static_cast<int>(ib),
                    static_cast<int>(ic));
      break;
    }
    case 0xA4: {
      const uint16_t speed_dps = readU16LE(&frame.data[2]);
      const int32_t angle_cdeg = readI32LE(&frame.data[4]);
      Serial.printf(" multiloop_control2 speed_dps=%u target_deg=%.2f",
                    static_cast<unsigned>(speed_dps),
                    static_cast<double>(angle_cdeg) / 100.0);
      break;
    }
    case 0xA7: {
      const int32_t increment_cdeg = readI32LE(&frame.data[4]);
      Serial.printf(" incremental_position target_deg=%.2f",
                    static_cast<double>(increment_cdeg) / 100.0);
      break;
    }
    default:
      Serial.print(" raw_or_unhandled");
      break;
  }
  Serial.println();
}

void sendSimpleLKTechCommand(uint8_t command) {
  const uint8_t payload[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendFrame(lktechRequestId(g_motor_id), payload, sizeof(payload));
}

void sendLKTechA4AbsoluteDegrees(double target_degrees) {
  const int32_t target_cdeg = degreesToCentiDegrees(target_degrees);
  const uint8_t payload[8] = {
      CMD_MULTI_LOOP_CONTROL_2,
      0x00,
      static_cast<uint8_t>(g_a4_speed_dps & 0xFF),
      static_cast<uint8_t>((g_a4_speed_dps >> 8) & 0xFF),
      static_cast<uint8_t>(target_cdeg & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 8) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 16) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 24) & 0xFF),
  };

  if (sendFrame(lktechRequestId(g_motor_id), payload, sizeof(payload))) {
    g_a4_target_cdeg = target_cdeg;
    Serial.printf("A4 command target_deg=%.2f speed_dps=%u\n",
                  target_degrees,
                  static_cast<unsigned>(g_a4_speed_dps));
  }
}

void sendLKTechA4AbsoluteCentiDegrees(int32_t target_cdeg) {
  sendLKTechA4AbsoluteDegrees(centiDegreesToDegrees(target_cdeg));
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

bool parseByteToken(const char *token, uint8_t *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }

  char *end = nullptr;
  const long parsed = strtol(token, &end, 0);
  if (end == token || *end != '\0' || parsed < 0 || parsed > 0xFF) {
    return false;
  }

  *value = static_cast<uint8_t>(parsed);
  return true;
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

bool parseRawPayloadCommand(char *line, uint8_t payload[8]) {
  char *trimmed = trimWhitespace(line);
  if (strncmp(trimmed, "raw", 3) != 0) {
    return false;
  }

  char *cursor = trimmed + 3;
  if (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  int count = 0;
  for (char *token = strtok(cursor, " ,\t"); token != nullptr; token = strtok(nullptr, " ,\t")) {
    if (count >= 8 || !parseByteToken(token, &payload[count])) {
      return false;
    }
    ++count;
  }
  return count == 8;
}

bool handleIdCommand(char *trimmed) {
  if (strncmp(trimmed, "id", 2) != 0) {
    return false;
  }

  char *cursor = trimmed + 2;
  if (*cursor == '\0' || !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  unsigned long parsed = 0;
  if (!parseUnsignedLongToken(trimWhitespace(cursor), &parsed) || parsed > 255) {
    Serial.println("invalid motor ID, expected 0..255");
    return true;
  }

  g_motor_id = static_cast<uint8_t>(parsed);
  Serial.printf("motor_id set to %u req_id=0x%03lX\n",
                static_cast<unsigned>(g_motor_id),
                static_cast<unsigned long>(lktechRequestId(g_motor_id)));
  return true;
}

bool handleSpeedCommand(char *trimmed) {
  if (strncmp(trimmed, "spd", 3) != 0) {
    return false;
  }

  char *cursor = trimmed + 3;
  if (*cursor == '\0' || !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  unsigned long parsed = 0;
  if (!parseUnsignedLongToken(trimWhitespace(cursor), &parsed) || parsed > 65535UL) {
    Serial.println("invalid A4 speed, expected 0..65535 dps");
    return true;
  }

  g_a4_speed_dps = static_cast<uint16_t>(parsed);
  Serial.printf("A4 speed set to %u dps\n", static_cast<unsigned>(g_a4_speed_dps));
  return true;
}

bool handleMoveCommand(char *trimmed) {
  if (strncmp(trimmed, "move", 4) != 0) {
    return false;
  }

  char *cursor = trimmed + 4;
  if (*cursor == '\0' || !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  double target_degrees = 0.0;
  if (!parseDoubleToken(trimWhitespace(cursor), &target_degrees)) {
    Serial.println("invalid move target, expected degrees like: move 180 or move -45.5");
    return true;
  }

  sendLKTechA4AbsoluteDegrees(target_degrees);
  return true;
}

bool handleStepCommand(char *trimmed) {
  if (strncmp(trimmed, "step", 4) != 0) {
    return false;
  }

  char *cursor = trimmed + 4;
  if (*cursor == '\0' || !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  if (!g_have_last_multi_loop_angle) {
    Serial.println("no multi-loop angle cached yet, send q first");
    return true;
  }

  double delta_degrees = 0.0;
  if (!parseDoubleToken(trimWhitespace(cursor), &delta_degrees)) {
    Serial.println("invalid step target, expected degrees like: step 10 or step -5.5");
    return true;
  }

  const int64_t delta_cdeg = static_cast<int64_t>(degreesToCentiDegrees(delta_degrees));
  const int64_t target_cdeg_64 = g_last_multi_loop_angle_cdeg + delta_cdeg;
  if (target_cdeg_64 < INT32_MIN || target_cdeg_64 > INT32_MAX) {
    Serial.println("step target is out of A4 int32 range");
    return true;
  }

  Serial.printf("A4 step current_deg=%.2f delta_deg=%.2f target_deg=%.2f speed_dps=%u\n",
                static_cast<double>(g_last_multi_loop_angle_cdeg) / 100.0,
                delta_degrees,
                static_cast<double>(target_cdeg_64) / 100.0,
                static_cast<unsigned>(g_a4_speed_dps));
  sendLKTechA4AbsoluteCentiDegrees(static_cast<int32_t>(target_cdeg_64));
  return true;
}

void printReceivedFrame(const can_frame &frame) {
  const uint32_t arbitration_id = frame.can_id & CAN_SFF_MASK;
  printFrame(frame, "RX");
  if (arbitration_id == TEST_ARBITRATION_ID &&
      frame.can_dlc == sizeof(TEST_DATA) &&
      memcmp(frame.data, TEST_DATA, sizeof(TEST_DATA)) == 0) {
    Serial.println("MATCH received expected 123#DEADBEEF");
  }
  printLKTechDecode(frame);
}

void pollReceive() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printReceivedFrame(frame);
  }
}

void handleSerialCommand(char command) {
  switch (command) {
    case 'h':
    case '?':
      printHelp();
      break;
    case 'p':
      printConfig();
      break;
    case 'r':
      configureController();
      break;
    case 's':
      sendTestFrame();
      break;
    case 'q':
      sendSimpleLKTechCommand(CMD_READ_MULTI_LOOP_ANGLE);
      break;
    case '1':
      sendSimpleLKTechCommand(CMD_READ_STATE_1);
      break;
    case '2':
      sendSimpleLKTechCommand(CMD_READ_STATE_2);
      break;
    case '3':
      sendSimpleLKTechCommand(CMD_READ_STATE_3);
      break;
    case 'o':
      sendSimpleLKTechCommand(CMD_MOTOR_ON);
      break;
    case 'x':
      sendSimpleLKTechCommand(CMD_MOTOR_OFF);
      break;
    case 't':
      sendSimpleLKTechCommand(CMD_MOTOR_STOP);
      break;
    case 'j':
      sendLKTechA4AbsoluteDegrees(180.0);
      break;
    case 'k':
      sendLKTechA4AbsoluteDegrees(-180.0);
      break;
    case '5':
      g_can_speed = CAN_500KBPS;
      configureController();
      break;
    case 'm':
      g_can_speed = CAN_1000KBPS;
      configureController();
      break;
    case '8':
      g_can_clock = MCP_8MHZ;
      configureController();
      break;
    case '6':
      g_can_clock = MCP_16MHZ;
      configureController();
      break;
    case 'n':
      g_mode = MODE_NORMAL;
      configureController();
      break;
    case 'i':
      g_mode = MODE_LISTEN_ONLY;
      configureController();
      break;
    case 'l':
      g_mode = MODE_LOOPBACK;
      configureController();
      break;
    case '\n':
    case '\r':
      break;
    default:
      Serial.printf("unknown command '%c'\n", command);
      printHelp();
      break;
  }
}

void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  if (trimmed[1] == '\0') {
    handleSerialCommand(trimmed[0]);
    return;
  }

  uint8_t payload[8] = {};
  if (parseRawPayloadCommand(trimmed, payload)) {
    sendFrame(lktechRequestId(g_motor_id), payload, sizeof(payload));
    return;
  }

  if (handleIdCommand(trimmed)) {
    return;
  }

  if (handleSpeedCommand(trimmed)) {
    return;
  }

  if (handleMoveCommand(trimmed)) {
    return;
  }

  if (handleStepCommand(trimmed)) {
    return;
  }

  Serial.printf("unknown line command: %s\n", trimmed);
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
  Serial.println("ESP32 + MCP2515 listener starting...");

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

  if (!configureController()) {
    Serial.println("Initial MCP2515 configuration failed.");
  }

  printHelp();
}

void loop() {
  processSerialInput();

  pollReceive();
  delay(5);
}
