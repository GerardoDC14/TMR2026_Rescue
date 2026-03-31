#include <SPI.h>
#include <mcp2515.h>

static const uint8_t PIN_CS = 5;
MCP2515 mcp2515(PIN_CS);
struct can_frame tx;

static const uint8_t VESC_ID = 10;
static const uint8_t CMD_SET_CURRENT = 1; // SET_CURRENT

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
  int32_t val = (int32_t)(currentA * 1000.0f); // A -> mA units per VESC CAN
  tx.can_id  = vescEID(VESC_ID, CMD_SET_CURRENT) | CAN_EFF_FLAG;
  tx.can_dlc = 4;
  putInt32BE(tx.data, val);
  auto err = mcp2515.sendMessage(&tx);
  if (err != MCP2515::ERROR_OK) {
    Serial.print("send err="); Serial.println((int)err);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  SPI.begin(18, 19, 23, PIN_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("SEND CURRENT start");
}

void loop() {
  // 2s empuje (torque)
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    sendVescCurrent(3.0f);  // prueba 3A (sube a 5A si no vence)
    delay(20);
  }

  // 2s stop
  t0 = millis();
  while (millis() - t0 < 2000) {
    sendVescCurrent(0.0f);
    delay(20);
  }
}
