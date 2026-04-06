#include <Arduino.h>
#include "driver/twai.h"

// Minimal ESP32 TWAI sanity-test sender.
//
// Default behavior:
// - TX pin: GPIO22
// - RX pin: GPIO21
// - Standard CAN ID: 0x123
// - Payload: DE AD BE EF
// - Sends once per second
//
// Serial commands:
//   h  - help
//   s  - send immediately
//   p  - print TWAI status
//   5  - switch to 500 kbps
//   m  - switch to 1 Mbps
//   b  - request TWAI bus recovery

static constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_22;
static constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_21;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t SEND_PERIOD_MS = 1000;
static constexpr uint32_t TEST_ARBITRATION_ID = 0x123;
static constexpr uint8_t TEST_DATA[] = {0xDE, 0xAD, 0xBE, 0xEF};

static uint32_t g_current_bitrate = 500000;
static uint32_t g_last_send_ms = 0;

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h  - help");
  Serial.println("  s  - send 123#DEADBEEF immediately");
  Serial.println("  p  - print TWAI status");
  Serial.println("  5  - switch TWAI bitrate to 500 kbps");
  Serial.println("  m  - switch TWAI bitrate to 1 Mbps");
  Serial.println("  b  - request TWAI bus recovery");
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

bool configureTWAI(uint32_t bitrate) {
  twai_stop();
  twai_driver_uninstall();

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t t_config;

  switch (bitrate) {
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
  g_config.rx_queue_len = 16;
  g_config.alerts_enabled =
      TWAI_ALERT_TX_SUCCESS |
      TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_ERR_PASS |
      TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_ARB_LOST;

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed err=0x%X\n", static_cast<unsigned>(err));
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed err=0x%X\n", static_cast<unsigned>(err));
    return false;
  }

  g_current_bitrate = bitrate;
  g_last_send_ms = millis();
  Serial.printf("TWAI configured bitrate=%lu sending 123#DEADBEEF\n",
                static_cast<unsigned long>(g_current_bitrate));
  printStatus();
  return true;
}

void sendTestFrame() {
  twai_message_t message = {};
  message.identifier = TEST_ARBITRATION_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = sizeof(TEST_DATA);
  memcpy(message.data, TEST_DATA, sizeof(TEST_DATA));

  const esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    Serial.printf("TX failed id=0x%03lX err=0x%X\n",
                  static_cast<unsigned long>(TEST_ARBITRATION_ID),
                  static_cast<unsigned>(err));
    return;
  }

  Serial.println("TX id=0x123 data=DE AD BE EF");
}

void handleAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0) {
    return;
  }

  Serial.printf("alert flags=0x%08lX", static_cast<unsigned long>(alerts));
  if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.print(" TX_SUCCESS");
  if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
  if (alerts & TWAI_ALERT_ERR_PASS) Serial.print(" ERR_PASS");
  if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
  if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
  if (alerts & TWAI_ALERT_BUS_RECOVERED) Serial.print(" BUS_RECOVERED");
  if (alerts & TWAI_ALERT_ARB_LOST) Serial.print(" ARB_LOST");
  Serial.println();
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
      printStatus();
      break;
    case '5':
      configureTWAI(500000);
      break;
    case 'm':
      configureTWAI(1000000);
      break;
    case 'b': {
      const esp_err_t err = twai_initiate_recovery();
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 TWAI 123#DEADBEEF sender starting...");
  Serial.printf("TWAI TX pin=%d RX pin=%d\n",
                static_cast<int>(TWAI_TX_PIN),
                static_cast<int>(TWAI_RX_PIN));

  if (!configureTWAI(g_current_bitrate)) {
    return;
  }

  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleSerialCommand(static_cast<char>(Serial.read()));
  }

  handleAlerts();

  const uint32_t now = millis();
  if (now - g_last_send_ms >= SEND_PERIOD_MS) {
    g_last_send_ms = now;
    sendTestFrame();
  }

  delay(10);
}
