#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "driver/twai.h"

// ESP32 built-in TWAI controller + external CAN transceiver.
// Wiring for your stated setup:
//   GPIO22 (CTX) -> transceiver TXD
//   GPIO21 (CRX) -> transceiver RXD
//   transceiver CANH -> motor CANH
//   transceiver CANL -> motor CANL
//   transceiver GND -> ESP32 GND
//
// This sketch is intentionally conservative:
// - It starts at 500 kbps
// - It accepts all CAN frames
// - It prints TWAI alerts, status, and received frames
// - It provides mostly read-only LKTech probe commands over Serial
//
// The ESP-IDF TWAI driver API and default config macros are documented by
// Espressif here:
// https://docs.espressif.com/projects/esp-idf/en/v4.2.3/esp32/api-reference/peripherals/twai.html
//
// LKTech CAN protocol notes used here come from the V2.36 CAN manuals the user
// provided. Important corrections from those manuals:
// - Single-motor command ID = 0x140 + ID
// - Single-motor reply ID   = 0x140 + ID   (same arbitration ID)
// - Broadcast mode uses IDs like 0x280/0x281/0x282/0x288 and is separate.

static constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_22;
static constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_21;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t STATUS_PERIOD_MS = 1000;
static constexpr uint8_t DEFAULT_LKTECH_MOTOR_ID = 1;
static constexpr uint16_t LKTECH_SINGLE_MOTOR_ID_BASE = 0x140;

static constexpr uint8_t CMD_READ_STATE_1 = 0x9A;
static constexpr uint8_t CMD_READ_STATE_2 = 0x9C;
static constexpr uint8_t CMD_READ_STATE_3 = 0x9D;
static constexpr uint8_t CMD_MOTOR_OFF = 0x80;
static constexpr uint8_t CMD_MOTOR_ON = 0x88;
static constexpr uint8_t CMD_MOTOR_STOP = 0x81;
static constexpr uint8_t CMD_READ_MULTI_LOOP_ANGLE = 0x92;
static constexpr uint8_t CMD_READ_SINGLE_LOOP_ANGLE = 0x94;
static constexpr uint8_t CMD_MULTI_LOOP_CONTROL_2 = 0xA4;
static constexpr uint8_t CMD_POSITION_CONTROL_5 = 0xA7;

static uint32_t g_last_status_ms = 0;
static uint32_t g_current_bitrate = 500000;
static uint8_t g_motor_id = DEFAULT_LKTECH_MOTOR_ID;
static uint16_t g_demo_a7_ctl_value_cdeg = 10 * 100;
static char g_serial_line[128] = {};
static size_t g_serial_line_len = 0;

uint32_t lktechRequestId(uint8_t motor_id) {
  return LKTECH_SINGLE_MOTOR_ID_BASE + motor_id;
}

bool isSingleMotorId(uint32_t arbitration_id) {
  return arbitration_id >= LKTECH_SINGLE_MOTOR_ID_BASE &&
         arbitration_id <= (LKTECH_SINGLE_MOTOR_ID_BASE + 0xFF);
}

uint8_t extractMotorId(uint32_t arbitration_id) {
  return static_cast<uint8_t>(arbitration_id - LKTECH_SINGLE_MOTOR_ID_BASE);
}

int16_t readI16LE(const uint8_t *data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
}

uint16_t readU16LE(const uint8_t *data) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
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

void printLKTechDecode(const twai_message_t &message) {
  if (message.extd || message.rtr || message.data_length_code != 8) {
    return;
  }
  if (!isSingleMotorId(message.identifier)) {
    return;
  }

  const uint8_t motor_id = extractMotorId(message.identifier);
  const uint8_t command = message.data[0];
  Serial.printf("  LKTech motor_id=%u cmd=0x%02X", motor_id, command);

  switch (command) {
    case CMD_READ_MULTI_LOOP_ANGLE: {
      const int64_t angle_cdeg = readSigned56LE(&message.data[1]);
      const double angle_deg = static_cast<double>(angle_cdeg) / 100.0;
      Serial.printf(" multi_loop_angle_deg=%.2f", angle_deg);
      break;
    }
    case CMD_READ_SINGLE_LOOP_ANGLE: {
      const uint32_t angle_cdeg = readU32LE(&message.data[4]);
      const double angle_deg = static_cast<double>(angle_cdeg) / 100.0;
      Serial.printf(" single_loop_angle_deg=%.2f", angle_deg);
      break;
    }
    case CMD_MULTI_LOOP_CONTROL_2: {
      const uint16_t speed_dps = readU16LE(&message.data[2]);
      const int32_t angle_cdeg = readI32LE(&message.data[4]);
      const double angle_deg = static_cast<double>(angle_cdeg) / 100.0;
      Serial.printf(" multiloop_control2 speed_dps=%u target_deg=%.2f",
                    static_cast<unsigned>(speed_dps),
                    angle_deg);
      break;
    }
    case CMD_POSITION_CONTROL_5: {
      const uint16_t request_cdeg = readU16LE(&message.data[4]);
      const uint16_t encoder_raw = readU16LE(&message.data[6]);
      Serial.printf(" position_control5 demo_request_cdeg=%u demo_request_deg=%.2f encoder_raw=%u",
                    static_cast<unsigned>(request_cdeg),
                    static_cast<double>(request_cdeg) / 100.0,
                    static_cast<unsigned>(encoder_raw));
      break;
    }
    case CMD_READ_STATE_1: {
      const int8_t temperature_c = static_cast<int8_t>(message.data[1]);
      const uint16_t voltage_centi_v = readU16LE(&message.data[2]);
      const int16_t current_centi_a = readI16LE(&message.data[4]);
      const uint8_t motor_state = message.data[6];
      const uint8_t error_state = message.data[7];
      Serial.printf(
          " state1 temp_c=%d voltage_v=%.2f current_a=%.2f motor_state=0x%02X error_state=0x%02X",
          static_cast<int>(temperature_c),
          static_cast<double>(voltage_centi_v) / 100.0,
          static_cast<double>(current_centi_a) / 100.0,
          motor_state,
          error_state);
      break;
    }
    case CMD_READ_STATE_2: {
      const int8_t temperature_c = static_cast<int8_t>(message.data[1]);
      const int16_t iq_raw = readI16LE(&message.data[2]);
      const int16_t speed_dps = readI16LE(&message.data[4]);
      const uint16_t encoder = readU16LE(&message.data[6]);
      Serial.printf(
          " state2 temp_c=%d iq_or_power_raw=%d speed_dps=%d encoder=%u",
          static_cast<int>(temperature_c),
          static_cast<int>(iq_raw),
          static_cast<int>(speed_dps),
          static_cast<unsigned>(encoder));
      break;
    }
    case CMD_READ_STATE_3: {
      const int8_t temperature_c = static_cast<int8_t>(message.data[1]);
      const int16_t ia = readI16LE(&message.data[2]);
      const int16_t ib = readI16LE(&message.data[4]);
      const int16_t ic = readI16LE(&message.data[6]);
      Serial.printf(
          " state3 temp_c=%d ia_raw=%d ib_raw=%d ic_raw=%d",
          static_cast<int>(temperature_c),
          static_cast<int>(ia),
          static_cast<int>(ib),
          static_cast<int>(ic));
      break;
    }
    case CMD_MOTOR_OFF:
      Serial.print(" motor_off_echo");
      break;
    case CMD_MOTOR_ON:
      Serial.print(" motor_on_echo");
      break;
    case CMD_MOTOR_STOP:
      Serial.print(" motor_stop_echo");
      break;
    default:
      Serial.print(" unknown_or_unhandled_reply");
      break;
  }
  Serial.println();
}

void printFrame(const twai_message_t &message) {
  Serial.printf(
      "RX id=0x%03lX ext=%d rtr=%d dlc=%d data=",
      static_cast<unsigned long>(message.identifier),
      message.extd ? 1 : 0,
      message.rtr ? 1 : 0,
      message.data_length_code);

  for (int i = 0; i < message.data_length_code; ++i) {
    Serial.printf("%02X", message.data[i]);
    if (i + 1 < message.data_length_code) {
      Serial.print(" ");
    }
  }
  Serial.println();
  printLKTechDecode(message);
}

void printStatus() {
  twai_status_info_t status_info;
  if (twai_get_status_info(&status_info) != ESP_OK) {
    Serial.println("status: failed to read TWAI status");
    return;
  }

  Serial.printf(
      "status bitrate=%lu state=%d tx_err=%lu rx_err=%lu msgs_to_tx=%lu msgs_to_rx=%lu tx_failed=%lu bus_error_count=%lu arb_lost=%lu\n",
      static_cast<unsigned long>(g_current_bitrate),
      static_cast<int>(status_info.state),
      static_cast<unsigned long>(status_info.tx_error_counter),
      static_cast<unsigned long>(status_info.rx_error_counter),
      static_cast<unsigned long>(status_info.msgs_to_tx),
      static_cast<unsigned long>(status_info.msgs_to_rx),
      static_cast<unsigned long>(status_info.tx_failed_count),
      static_cast<unsigned long>(status_info.bus_error_count),
      static_cast<unsigned long>(status_info.arb_lost_count));
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h  - help");
  Serial.println("  r  - send LKTech read multi-loop angle request (0x92)");
  Serial.println("  a  - send LKTech read single-loop angle request (0x94)");
  Serial.println("  1  - send LKTech read state 1 request (0x9A)");
  Serial.println("  s  - send LKTech read state 2 request (0x9C)");
  Serial.println("  3  - send LKTech read state 3 request (0x9D)");
  Serial.println("  o  - send LKTech motor on command (0x88)");
  Serial.println("  x  - send LKTech motor off command (0x80)");
  Serial.println("  t  - send LKTech stop command (0x81)");
  Serial.println("  7  - send STM32 demo A7 command with current demo target");
  Serial.println("  j  - send known A4 example: +180 deg at 120 dps");
  Serial.println("  k  - send known A4 example: -180 deg at 120 dps");
  Serial.println("  5  - switch TWAI bitrate to 500 kbps");
  Serial.println("  m  - switch TWAI bitrate to 1 Mbps");
  Serial.println("  p  - print current TWAI status immediately");
  Serial.println("  b  - initiate TWAI bus recovery");
  Serial.println("  id N");
  Serial.println("     set current motor ID to N (0..255)");
  Serial.println("  a7deg N");
  Serial.println("     set demo A7 target in degrees for command 7");
  Serial.println("  raw XX XX XX XX XX XX XX XX");
  Serial.println("     send arbitrary 8-byte payload to current 0x140+ID request ID");
}

bool configureTWAI(uint32_t bitrate) {
  twai_stop();
  twai_driver_uninstall();

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t t_config;

  switch (bitrate) {
    case 250000: 
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break; 
    case 500000:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000000:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      Serial.printf("unsupported bitrate=%lu\n", static_cast<unsigned long>(bitrate));
      return false;
  }

  g_config.tx_queue_len = 8;
  g_config.rx_queue_len = 32;
  g_config.alerts_enabled =
      TWAI_ALERT_RX_DATA |
      TWAI_ALERT_TX_SUCCESS |
      TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_ERR_PASS |
      TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_ARB_LOST;

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed err=0x%X for bitrate=%lu\n",
                  static_cast<unsigned>(err),
                  static_cast<unsigned long>(bitrate));
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed err=0x%X for bitrate=%lu\n",
                  static_cast<unsigned>(err),
                  static_cast<unsigned long>(bitrate));
    return false;
  }

  g_current_bitrate = bitrate;
  g_last_status_ms = millis();
  Serial.printf("TWAI reconfigured to bitrate=%lu motor_id=%u req_id=0x%03lX\n",
                static_cast<unsigned long>(g_current_bitrate),
                static_cast<unsigned>(g_motor_id),
                static_cast<unsigned long>(lktechRequestId(g_motor_id)));
  printStatus();
  return true;
}

bool sendStandardFrame(uint32_t arbitration_id, const uint8_t data[8]) {
  twai_message_t message = {};
  message.identifier = arbitration_id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  memcpy(message.data, data, 8);

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    Serial.printf("TX failed id=0x%03lX err=0x%X\n",
                  static_cast<unsigned long>(arbitration_id),
                  static_cast<unsigned>(err));
    return false;
  }

  Serial.printf("TX id=0x%03lX data=", static_cast<unsigned long>(arbitration_id));
  for (int i = 0; i < 8; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < 8) {
      Serial.print(" ");
    }
  }
  Serial.println();
  return true;
}

void sendReadMultiLoopAngle() {
  const uint8_t payload[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendReadSingleLoopAngle() {
  const uint8_t payload[8] = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendReadState2() {
  const uint8_t payload[8] = {CMD_READ_STATE_2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendReadState1() {
  const uint8_t payload[8] = {CMD_READ_STATE_1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendReadState3() {
  const uint8_t payload[8] = {CMD_READ_STATE_3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendMotorOn() {
  const uint8_t payload[8] = {CMD_MOTOR_ON, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendMotorOff() {
  const uint8_t payload[8] = {CMD_MOTOR_OFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendMotorStop() {
  const uint8_t payload[8] = {CMD_MOTOR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendKnownA4Positive180() {
  const uint8_t payload[8] = {CMD_MULTI_LOOP_CONTROL_2, 0x00, 0x78, 0x00, 0x50, 0x46, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendKnownA4Negative180() {
  const uint8_t payload[8] = {CMD_MULTI_LOOP_CONTROL_2, 0x00, 0x78, 0x00, 0xB0, 0xB9, 0xFF, 0xFF};
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
}

void sendDemoA7Command() {
  const uint8_t payload[8] = {
      CMD_POSITION_CONTROL_5,
      0x00,
      0x00,
      0x00,
      static_cast<uint8_t>(g_demo_a7_ctl_value_cdeg & 0xFF),
      static_cast<uint8_t>((g_demo_a7_ctl_value_cdeg >> 8) & 0xFF),
      0x00,
      0x00,
  };
  sendStandardFrame(lktechRequestId(g_motor_id), payload);
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

bool parseFloatToken(const char *token, float *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }

  char *end = nullptr;
  const float parsed = strtof(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }

  *value = parsed;
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

void handleSingleCharCommand(char command) {
  switch (command) {
    case 'h':
    case '?':
      printHelp();
      break;
    case 'r':
      sendReadMultiLoopAngle();
      break;
    case 'a':
      sendReadSingleLoopAngle();
      break;
    case '1':
      sendReadState1();
      break;
    case 's':
      sendReadState2();
      break;
    case '3':
      sendReadState3();
      break;
    case 'o':
      sendMotorOn();
      break;
    case 'x':
      sendMotorOff();
      break;
    case 't':
      sendMotorStop();
      break;
    case '7':
      sendDemoA7Command();
      break;
    case 'j':
      sendKnownA4Positive180();
      break;
    case 'k':
      sendKnownA4Negative180();
      break;
    case '2':
      configureTWAI(250000);
      break;
    case '5':
      configureTWAI(500000);
      break;
    case 'm':
      configureTWAI(1000000);
      break;
    case 'p':
      printStatus();
      break;
    case 'b': {
      esp_err_t err = twai_initiate_recovery();
      Serial.printf("bus recovery requested err=0x%X\n", static_cast<unsigned>(err));
      break;
    }
    case '\n':
    case '\r':
      break;
    default:
      Serial.printf("unknown command '%c'\n", command);
      printHelp();
      break;
  }
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

bool handleA7DegCommand(char *trimmed) {
  if (strncmp(trimmed, "a7deg", 5) != 0) {
    return false;
  }

  char *cursor = trimmed + 5;
  if (*cursor == '\0' || !isspace(static_cast<unsigned char>(*cursor))) {
    return false;
  }

  float degrees = 0.0f;
  if (!parseFloatToken(trimWhitespace(cursor), &degrees)) {
    Serial.println("invalid a7deg value");
    return true;
  }

  if (degrees < 0.0f || degrees > 655.35f) {
    Serial.println("a7deg out of range, expected 0.0..655.35");
    return true;
  }

  g_demo_a7_ctl_value_cdeg = static_cast<uint16_t>(degrees * 100.0f + 0.5f);
  Serial.printf("A7 demo target set to %.2f deg (%u cdeg)\n",
                static_cast<double>(g_demo_a7_ctl_value_cdeg) / 100.0,
                static_cast<unsigned>(g_demo_a7_ctl_value_cdeg));
  return true;
}

void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  if (trimmed[1] == '\0') {
    handleSingleCharCommand(trimmed[0]);
    return;
  }

  uint8_t payload[8] = {};
  if (parseRawPayloadCommand(trimmed, payload)) {
    sendStandardFrame(lktechRequestId(g_motor_id), payload);
    return;
  }

  if (handleIdCommand(trimmed)) {
    return;
  }

  if (handleA7DegCommand(trimmed)) {
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

void handleAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0) {
    return;
  }

  Serial.printf("alert flags=0x%08lX", static_cast<unsigned long>(alerts));
  if (alerts & TWAI_ALERT_RX_DATA) Serial.print(" RX_DATA");
  if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.print(" TX_SUCCESS");
  if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
  if (alerts & TWAI_ALERT_ERR_PASS) Serial.print(" ERR_PASS");
  if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
  if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
  if (alerts & TWAI_ALERT_BUS_RECOVERED) Serial.print(" BUS_RECOVERED");
  if (alerts & TWAI_ALERT_ARB_LOST) Serial.print(" ARB_LOST");
  Serial.println();
}

void handleReceive() {
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    printFrame(message);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 TWAI LKTech probe starting...");
  Serial.printf("TWAI TX pin=%d RX pin=%d bitrate=%lu motor_id=%u req_id=0x%03lX\n",
                static_cast<int>(TWAI_TX_PIN),
                static_cast<int>(TWAI_RX_PIN),
                static_cast<unsigned long>(g_current_bitrate),
                static_cast<unsigned>(g_motor_id),
                static_cast<unsigned long>(lktechRequestId(g_motor_id)));

  if (!configureTWAI(g_current_bitrate)) {
    return;
  }

  Serial.println("TWAI started.");
  printHelp();
  printStatus();
}

void loop() {
  processSerialInput();

  handleAlerts();
  handleReceive();

  const uint32_t now = millis();
  if (now - g_last_status_ms >= STATUS_PERIOD_MS) {
    g_last_status_ms = now;
    printStatus();
  }

  delay(10);
}
