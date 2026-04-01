// Standalone PPM → VESC current test — uses ESP32 TWAI (built-in CAN)
#include "driver/twai.h"

// -------------------- PPM CONFIG --------------------
#define PPM_PIN            4
#define CHANNELS           6
#define SYNC_PULSE_US      2100
#define BAUD_RATE          115200

// Deadband around 1500 us
#define PPM_DEADBAND_LOW   1480
#define PPM_DEADBAND_HIGH  1520

// Clamp valid PPM range (safety)
#define PPM_MIN_US         1000
#define PPM_MAX_US         2000

// PPM read variables
volatile uint32_t ppmValues[CHANNELS] = {0};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPulse = 0;
volatile bool frameReady = false;

volatile uint32_t lastFrameMicros = 0;

// ISR for PPM
void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint32_t pulseWidth = now - lastPulse;
  lastPulse = now;

  if (pulseWidth > SYNC_PULSE_US) {
    currentChannel = 0;
    frameReady = true;
    lastFrameMicros = now;
  } else {
    if (currentChannel < CHANNELS) {
      ppmValues[currentChannel] = pulseWidth;
      currentChannel++;
    }
  }
}

// -------------------- CAN / VESC CONFIG --------------------
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
    Serial.print("CAN send err=");
    Serial.println(err);
  }
}

// -------------------- CONTROL TUNING --------------------
static const float I_MAX_A = 5.0f;
static const uint32_t CAN_PERIOD_MS = 20;   // 50 Hz
static const uint32_t FAILSAFE_MS = 200;

static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float ppmToNormalized(int ppm) {
  ppm = clampInt(ppm, PPM_MIN_US, PPM_MAX_US);

  if (ppm >= PPM_DEADBAND_LOW && ppm <= PPM_DEADBAND_HIGH) {
    return 0.0f;
  }

  if (ppm > PPM_DEADBAND_HIGH) {
    return (float)(ppm - PPM_DEADBAND_HIGH) / (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
  } else {
    return -(float)(PPM_DEADBAND_LOW - ppm) / (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  // PPM setup
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
  Serial.println("PPM + CAN control start");

  // TWAI CAN setup
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
  Serial.println("CAN init done (TWAI)");
}

void loop() {
  static uint32_t lastCanMs = 0;
  uint32_t nowMs = millis();

  if (frameReady) {
    frameReady = false;
    Serial.print("CH2=");
    Serial.println(ppmValues[1]);
  }

  if (nowMs - lastCanMs >= CAN_PERIOD_MS) {
    lastCanMs = nowMs;

    uint32_t ageMs = (micros() - lastFrameMicros) / 1000;
    if (ageMs > FAILSAFE_MS) {
      sendVescCurrent(0.0f);
      Serial.println("FAILSAFE -> 0A");
      return;
    }

    int ch2 = (int)ppmValues[1];
    float x = ppmToNormalized(ch2);
    float currentCmd = x * I_MAX_A;
    sendVescCurrent(currentCmd);
  }
}
