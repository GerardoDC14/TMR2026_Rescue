#include <SPI.h>
#include <mcp2515.h>

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
static const uint8_t PIN_CS  = 5;   // MCP2515 CS
static const uint8_t VESC_ID = 10;  // tu VESC ID (según screenshot)
static const uint8_t CMD_SET_CURRENT = 1; // VESC: SET_CURRENT

MCP2515 mcp2515(PIN_CS);
struct can_frame tx;

static inline uint32_t vescEID(uint8_t unit_id, uint8_t cmd) {
  return ((uint32_t)cmd << 8) | unit_id;
}

static inline void putInt32BE(uint8_t *d, int32_t v) {
  d[0] = (v >> 24) & 0xFF;
  d[1] = (v >> 16) & 0xFF;
  d[2] = (v >>  8) & 0xFF;
  d[3] = (v >>  0) & 0xFF;
}

// Send current command (A). VESC expects current * 1000 as int32.
void sendVescCurrent(float currentA) {
  int32_t val = (int32_t)(currentA * 1000.0f); // A -> mA units
  tx.can_id  = vescEID(VESC_ID, CMD_SET_CURRENT) | CAN_EFF_FLAG;
  tx.can_dlc = 4;
  putInt32BE(tx.data, val);

  auto err = mcp2515.sendMessage(&tx);
  if (err != MCP2515::ERROR_OK) {
    Serial.print("CAN send err=");
    Serial.println((int)err);
  }
}

// -------------------- CONTROL TUNING --------------------
// Max motor current commanded by RC (choose safe value)
static const float I_MAX_A = 5.0f;      // ajusta (ej. 3A, 5A, 10A) según tu setup
static const uint32_t CAN_PERIOD_MS = 20;  // 50 Hz
static const uint32_t FAILSAFE_MS = 200;   // si no hay frame PPM en 200ms -> 0A

// Helper: clamp
static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Map PPM to [-1, 1] with deadband and linear scaling
float ppmToNormalized(int ppm) {
  ppm = clampInt(ppm, PPM_MIN_US, PPM_MAX_US);

  if (ppm >= PPM_DEADBAND_LOW && ppm <= PPM_DEADBAND_HIGH) {
    return 0.0f;
  }

  if (ppm > PPM_DEADBAND_HIGH) {
    // Map [1520..2000] -> [0..1]
    return (float)(ppm - PPM_DEADBAND_HIGH) / (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
  } else {
    // Map [1480..1000] -> [0..-1]
    return -(float)(PPM_DEADBAND_LOW - ppm) / (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  // PPM setup
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
  Serial.println("PPM + CAN control start");

  // CAN setup (VSPI explicit)
  SPI.begin(18, 19, 23, PIN_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("CAN init done");
}

void loop() {
  static uint32_t lastCanMs = 0;
  uint32_t nowMs = millis();

  // Optionally print PPM frames (debug)
  if (frameReady) {
    frameReady = false;
    // Debug: print channel 2
    Serial.print("CH2=");
    Serial.println(ppmValues[1]);
  }

  // Run control loop at 50Hz
  if (nowMs - lastCanMs >= CAN_PERIOD_MS) {
    lastCanMs = nowMs;

    // Failsafe: if no PPM frame recently -> stop
    uint32_t ageMs = (micros() - lastFrameMicros) / 1000;
    if (ageMs > FAILSAFE_MS) {
      sendVescCurrent(0.0f);
      Serial.println("FAILSAFE -> 0A");
      return;
    }

    // Read channel 2 safely (copy volatile)
    int ch2 = (int)ppmValues[1];

    float x = ppmToNormalized(ch2); // -1..1
    float currentCmd = x * I_MAX_A;

    sendVescCurrent(currentCmd);
  }
}
