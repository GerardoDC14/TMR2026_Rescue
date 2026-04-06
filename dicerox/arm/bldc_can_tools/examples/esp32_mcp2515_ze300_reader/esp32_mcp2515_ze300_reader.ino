#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

// Basic ZE300 CAN reader for ESP32 + MCP2515.
//
// Protocol basis:
// - Standard 11-bit CAN
// - Default bitrate 1 Mbps
// - Request can be sent to (0x100 | Dev_addr)
// - Reply comes back on Dev_addr
//
// This sketch is intentionally read-focused for bring-up on a mixed bus.

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
static constexpr uint8_t CMD_A0 = 0xA0;
static constexpr uint8_t CMD_A3 = 0xA3;
static constexpr uint8_t CMD_A4 = 0xA4;
static constexpr uint8_t CMD_AE = 0xAE;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;
static constexpr unsigned long SCAN_TIMEOUT_MS = 800;

MCP2515 g_mcp2515(MCP2515_CS_PIN);
char g_serial_line[96] = {};
size_t g_serial_line_len = 0;
uint8_t g_device_address = 1;
bool g_tagged_request = true;

uint16_t requestId() {
  return g_tagged_request ? static_cast<uint16_t>(ZE300_REQUEST_TAG_MASK | g_device_address)
                          : g_device_address;
}

uint16_t replyId() {
  return g_device_address;
}

double countsToDeg(int32_t counts) {
  return counts * (360.0 / ZE300_COUNTS_PER_REV);
}

uint16_t readU16LE(const uint8_t *data, uint8_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}

int16_t readI16LE(const uint8_t *data, uint8_t offset) {
  return static_cast<int16_t>(readU16LE(data, offset));
}

int32_t readI32LE(const uint8_t *data, uint8_t offset) {
  uint32_t value = static_cast<uint32_t>(data[offset]) |
                   (static_cast<uint32_t>(data[offset + 1]) << 8) |
                   (static_cast<uint32_t>(data[offset + 2]) << 16) |
                   (static_cast<uint32_t>(data[offset + 3]) << 24);
  return static_cast<int32_t>(value);
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

void printConfig() {
  Serial.printf("config bitrate=1 Mbps oscillator=8 MHz device_address=%u request_id=0x%03X reply_id=0x%03X request_mode=%s cs=%u int=%u sck=%u miso=%u mosi=%u\n",
                static_cast<unsigned>(g_device_address),
                requestId(),
                replyId(),
                g_tagged_request ? "0x100|addr" : "addr",
                static_cast<unsigned>(MCP2515_CS_PIN),
                static_cast<unsigned>(MCP2515_INT_PIN),
                static_cast<unsigned>(SPI_SCK_PIN),
                static_cast<unsigned>(SPI_MISO_PIN),
                static_cast<unsigned>(SPI_MOSI_PIN));
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  a0");
  Serial.println("     read boot/app/hardware/CAN protocol versions");
  Serial.println("  a3");
  Serial.println("     read single-turn and multi-turn absolute angles");
  Serial.println("  a4");
  Serial.println("     read temp, q-current, speed, single-turn angle");
  Serial.println("  ae");
  Serial.println("     read bus voltage/current, temp, mode, and faults");
  Serial.println("  all");
  Serial.println("     run a0, a3, a4, ae in sequence");
  Serial.println("  probe");
  Serial.println("     try A0 once with tag on and once with tag off");
  Serial.println("  scan");
  Serial.println("     send A0 to common address 0xFF and print all version replies");
  Serial.println("  addr N");
  Serial.println("     set ZE300 device address (1..254)");
  Serial.println("  tag on");
  Serial.println("  tag off");
  Serial.println("     choose request ID as 0x100|addr or direct addr");
  Serial.println("  status");
  Serial.println("     print current reader configuration");
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
  printConfig();
  return true;
}

void drainReceiveQueue() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printFrame(frame, "RX(stale)");
  }
}

bool sendCommand(uint8_t command) {
  can_frame frame = {};
  frame.can_id = requestId();
  frame.can_dlc = 1;
  frame.data[0] = command;

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d\n", static_cast<int>(result));
    return false;
  }

  printFrame(frame, "TX");
  return true;
}

void decodeReply(const can_frame &frame) {
  if (frame.can_dlc == 0) {
    return;
  }

  switch (frame.data[0]) {
    case CMD_A0:
      if (frame.can_dlc == 8) {
        Serial.printf("  A0 versions: boot=0x%04X app=0x%04X hw=0x%04X can_proto=0x%02X\n",
                      readU16LE(frame.data, 1),
                      readU16LE(frame.data, 3),
                      readU16LE(frame.data, 5),
                      static_cast<unsigned>(frame.data[7]));
      }
      break;
    case CMD_A3:
      if (frame.can_dlc == 7) {
        const uint16_t single_counts = readU16LE(frame.data, 1);
        const int32_t multi_counts = readI32LE(frame.data, 3);
        Serial.printf("  A3 angles: single=%.2f deg (%u counts) multi=%.2f deg (%ld counts)\n",
                      countsToDeg(single_counts),
                      static_cast<unsigned>(single_counts),
                      countsToDeg(multi_counts),
                      static_cast<long>(multi_counts));
      }
      break;
    case CMD_A4:
      if (frame.can_dlc == 8) {
        const int16_t q_current_ma = readI16LE(frame.data, 2);
        const int16_t speed_centirpm = readI16LE(frame.data, 4);
        const uint16_t single_counts = readU16LE(frame.data, 6);
        Serial.printf("  A4 realtime: temp=%u C q_current=%.3f A speed=%.2f rpm single=%.2f deg\n",
                      static_cast<unsigned>(frame.data[1]),
                      static_cast<double>(q_current_ma) / 1000.0,
                      static_cast<double>(speed_centirpm) / 100.0,
                      countsToDeg(single_counts));
      }
      break;
    case CMD_AE:
      if (frame.can_dlc == 8) {
        Serial.printf("  AE state: bus_voltage=%.2f V bus_current=%.2f A temp=%u C mode=%u fault=0x%02X\n",
                      static_cast<double>(readU16LE(frame.data, 1)) / 100.0,
                      static_cast<double>(readU16LE(frame.data, 3)) / 100.0,
                      static_cast<unsigned>(frame.data[5]),
                      static_cast<unsigned>(frame.data[6]),
                      static_cast<unsigned>(frame.data[7]));
      }
      break;
    default:
      Serial.println("  unhandled reply");
      break;
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
        decodeReply(frame);
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

void runRead(uint8_t command) {
  can_frame reply = {};
  drainReceiveQueue();
  if (!sendCommand(command)) {
    return;
  }
  waitForReply(command, &reply);
}

void sendRawOneByteCommand(uint16_t arbitration_id, uint8_t command) {
  can_frame frame = {};
  frame.can_id = arbitration_id;
  frame.can_dlc = 1;
  frame.data[0] = command;

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d\n", static_cast<int>(result));
    return;
  }

  printFrame(frame, "TX");
}

void runProbe() {
  const bool original_tag = g_tagged_request;
  const bool modes[2] = {true, false};

  for (bool tagged : modes) {
    g_tagged_request = tagged;
    Serial.printf("probe: trying A0 with request mode %s\n",
                  g_tagged_request ? "0x100|addr" : "addr");
    runRead(CMD_A0);
  }

  g_tagged_request = original_tag;
  printConfig();
}

void runScan() {
  drainReceiveQueue();
  Serial.println("scan: sending A0 to common address 0x0FF and listening for version replies...");
  sendRawOneByteCommand(0x0FF, CMD_A0);

  const unsigned long start_ms = millis();
  can_frame frame = {};
  bool any_reply = false;

  while (millis() - start_ms < SCAN_TIMEOUT_MS) {
    if (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      printFrame(frame, "RX");
      if ((frame.can_id & CAN_SFF_MASK) <= 0x0FF &&
          frame.can_dlc >= 1 &&
          frame.data[0] == CMD_A0) {
        any_reply = true;
        decodeReply(frame);
        Serial.printf("  scan hit: device_address=%lu\n",
                      static_cast<unsigned long>(frame.can_id & CAN_SFF_MASK));
      }
    }
    delay(2);
  }

  if (!any_reply) {
    Serial.println("scan: no ZE300 version replies received.");
  }
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
    printConfig();
    return;
  }
  if (strcmp(trimmed, "a0") == 0) {
    runRead(CMD_A0);
    return;
  }
  if (strcmp(trimmed, "a3") == 0) {
    runRead(CMD_A3);
    return;
  }
  if (strcmp(trimmed, "a4") == 0) {
    runRead(CMD_A4);
    return;
  }
  if (strcmp(trimmed, "ae") == 0) {
    runRead(CMD_AE);
    return;
  }
  if (strcmp(trimmed, "all") == 0) {
    runRead(CMD_A0);
    runRead(CMD_A3);
    runRead(CMD_A4);
    runRead(CMD_AE);
    return;
  }
  if (strcmp(trimmed, "probe") == 0) {
    runProbe();
    return;
  }
  if (strcmp(trimmed, "scan") == 0) {
    runScan();
    return;
  }
  if (strncmp(trimmed, "addr", 4) == 0) {
    char *value = trimWhitespace(trimmed + 4);
    unsigned long parsed = 0;
    if (!parseUnsignedLongToken(value, &parsed) || parsed == 0 || parsed > 254) {
      Serial.println("invalid ZE300 device address, expected 1..254");
      return;
    }
    g_device_address = static_cast<uint8_t>(parsed);
    printConfig();
    return;
  }
  if (strcmp(trimmed, "tag on") == 0) {
    g_tagged_request = true;
    printConfig();
    return;
  }
  if (strcmp(trimmed, "tag off") == 0) {
    g_tagged_request = false;
    printConfig();
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
  Serial.println("ESP32 + MCP2515 ZE300 reader starting...");

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
