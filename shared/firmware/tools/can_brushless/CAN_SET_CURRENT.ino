// Standalone VESC current command test — uses ESP32 TWAI (built-in CAN)
#include "driver/twai.h"

#define TWAI_TX GPIO_NUM_4
#define TWAI_RX GPIO_NUM_5

static const uint8_t VESC_ID = 10;
static const uint8_t CMD_SET_CURRENT = 1;

static inline uint32_t vescEID(uint8_t unit_id, uint8_t cmd) {
  return ((uint32_t)cmd << 8) | unit_id;
}

static inline void putInt32BE(uint8_t *d, int32_t v) {
  d[0] = (v >> 24) & 0xFF;
  d[1] = (v >> 16) & 0xFF;
  d[2] = (v >>  8) & 0xFF;
  d[3] = (v >>  0) & 0xFF;
}

void sendVescCurrent(float currentA) {
  int32_t val = (int32_t)(currentA * 1000.0f);
  twai_message_t msg = {};
  msg.extd       = 1;
  msg.identifier = vescEID(VESC_ID, CMD_SET_CURRENT);
  msg.data_length_code = 4;
  putInt32BE(msg.data, val);
  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
  if (err != ESP_OK) {
    Serial.print("send err="); Serial.println(err);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  Serial.println("SEND CURRENT start (TWAI)");
}

void loop() {
  // 2s push (torque)
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    sendVescCurrent(3.0f);
    delay(20);
  }

  // 2s stop
  t0 = millis();
  while (millis() - t0 < 2000) {
    sendVescCurrent(0.0f);
    delay(20);
  }
}
