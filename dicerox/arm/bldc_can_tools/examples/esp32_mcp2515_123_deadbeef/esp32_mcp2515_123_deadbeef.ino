#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// Minimal ESP32 + MCP2515 sanity-test example using the common
// "autowp/arduino-mcp2515" library.
//
// Install:
//   Library Manager -> search for "mcp2515" by autowp
//
// Default wiring for a typical ESP32 DevKit:
//   ESP32 GPIO23 -> MCP2515 SI
//   ESP32 GPIO19 -> MCP2515 SO
//   ESP32 GPIO18 -> MCP2515 SCK
//   ESP32 GPIO5  -> MCP2515 CS
//   ESP32 GPIO4  -> MCP2515 INT (optional for this sketch)
//   ESP32 3V3/5V -> module VCC  (depends on your module)
//   ESP32 GND    -> module GND
//   MCP2515 CANH -> CAN bus H
//   MCP2515 CANL -> CAN bus L
//
// Test frame:
//   123#DEADBEEF
//
// Serial commands at 115200:
//   h  - help
//   s  - send immediately
//   p  - print current config
//   5  - set 500 kbps
//   m  - set 1 Mbps
//   8  - set MCP2515 oscillator to 8 MHz
//   6  - set MCP2515 oscillator to 16 MHz
//   n  - normal mode
//   l  - loopback mode
//   i  - listen-only mode

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t SEND_PERIOD_MS = 1000;

static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;

static constexpr uint32_t TEST_ARBITRATION_ID = 0x123;
static constexpr uint8_t TEST_DATA[] = {0xDE, 0xAD, 0xBE, 0xEF};

enum SketchMode {
  MODE_NORMAL,
  MODE_LOOPBACK,
  MODE_LISTEN_ONLY,
};

MCP2515 g_mcp2515(MCP2515_CS_PIN);
CAN_SPEED g_can_speed = CAN_500KBPS;
CAN_CLOCK g_can_clock = MCP_8MHZ;
SketchMode g_mode = MODE_NORMAL;
uint32_t g_last_send_ms = 0;

const char *modeToString(SketchMode mode) {
  switch (mode) {
    case MODE_NORMAL:
      return "normal";
    case MODE_LOOPBACK:
      return "loopback";
    case MODE_LISTEN_ONLY:
      return "listen-only";
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

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h  - help");
  Serial.println("  s  - send 123#DEADBEEF immediately");
  Serial.println("  p  - print current config");
  Serial.println("  5  - set 500 kbps");
  Serial.println("  m  - set 1 Mbps");
  Serial.println("  8  - set MCP2515 oscillator to 8 MHz");
  Serial.println("  6  - set MCP2515 oscillator to 16 MHz");
  Serial.println("  n  - normal mode");
  Serial.println("  l  - loopback mode");
  Serial.println("  i  - listen-only mode");
}

void printConfig() {
  Serial.printf(
      "config id=0x%03lX data=DE AD BE EF bitrate=%s oscillator=%s mode=%s cs=%u int=%u sck=%u miso=%u mosi=%u\n",
      static_cast<unsigned long>(TEST_ARBITRATION_ID),
      speedToString(g_can_speed),
      clockToString(g_can_clock),
      modeToString(g_mode),
      static_cast<unsigned>(MCP2515_CS_PIN),
      static_cast<unsigned>(MCP2515_INT_PIN),
      static_cast<unsigned>(SPI_SCK_PIN),
      static_cast<unsigned>(SPI_MISO_PIN),
      static_cast<unsigned>(SPI_MOSI_PIN));
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
    case MODE_LOOPBACK:
      mode_result = g_mcp2515.setLoopbackMode();
      break;
    case MODE_LISTEN_ONLY:
      mode_result = g_mcp2515.setListenOnlyMode();
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

void printFrame(const can_frame &frame, const char *prefix) {
  Serial.printf("%s id=0x%03lX dlc=%u data=",
                prefix,
                static_cast<unsigned long>(frame.can_id & CAN_SFF_MASK),
                static_cast<unsigned>(frame.can_dlc));
  for (uint8_t i = 0; i < frame.can_dlc; ++i) {
    Serial.printf("%02X", frame.data[i]);
    if (i + 1 < frame.can_dlc) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void sendTestFrame() {
  can_frame frame = {};
  frame.can_id = TEST_ARBITRATION_ID;
  frame.can_dlc = sizeof(TEST_DATA);
  memcpy(frame.data, TEST_DATA, sizeof(TEST_DATA));

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d\n", static_cast<int>(result));
    return;
  }

  printFrame(frame, "TX");
}

void pollReceive() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printFrame(frame, "RX");
    if ((frame.can_id & CAN_SFF_MASK) == TEST_ARBITRATION_ID &&
        frame.can_dlc == sizeof(TEST_DATA) &&
        memcmp(frame.data, TEST_DATA, sizeof(TEST_DATA)) == 0) {
      Serial.println("MATCH received expected 123#DEADBEEF");
    }
  }
}

void handleSerialCommand(char command) {
  switch (command) {
    case 'h':
    case '?':
      printHelp();
      break;
    case 's':
      sendTestFrame();
      break;
    case 'p':
      printConfig();
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
    case 'l':
      g_mode = MODE_LOOPBACK;
      configureController();
      break;
    case 'i':
      g_mode = MODE_LISTEN_ONLY;
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 + MCP2515 123#DEADBEEF test starting...");

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

  if (!configureController()) {
    Serial.println("Initial MCP2515 configuration failed.");
  }

  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleSerialCommand(static_cast<char>(Serial.read()));
  }

  pollReceive();

  const uint32_t now = millis();
  if (g_mode != MODE_LISTEN_ONLY && now - g_last_send_ms >= SEND_PERIOD_MS) {
    g_last_send_ms = now;
    sendTestFrame();
  }

  delay(10);
}
