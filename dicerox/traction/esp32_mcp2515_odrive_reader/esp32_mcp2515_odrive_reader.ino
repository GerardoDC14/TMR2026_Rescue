#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif

#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000UL
#endif

#ifndef CAN_SFF_MASK
#define CAN_SFF_MASK 0x7FFUL
#endif

// Minimal ESP32 + MCP2515 reader for one ODrive CAN node.
//
// Your current ODrive settings:
//   axis0.config.can.node_id = 0x02
//   can.config.baud_rate     = 1000000
//
// Wiring used here matches the MCP2515 examples already present in this repo:
//   ESP32 GPIO23 -> MCP2515 SI
//   ESP32 GPIO19 -> MCP2515 SO
//   ESP32 GPIO18 -> MCP2515 SCK
//   ESP32 GPIO5  -> MCP2515 CS
//   ESP32 GPIO4  -> MCP2515 INT
//
// Important:
// - Leave the MCP2515 in NORMAL mode if this is the only CAN receiver on the bus,
//   otherwise the ODrive frames may not get ACKed.
// - Many clone MCP2515 boards use an 8 MHz crystal. If you see no traffic,
//   try changing MCP_CLOCK below to MCP_16MHZ.

static constexpr uint32_t SERIAL_BAUD = 115200;

static constexpr uint8_t PIN_CAN_SCK  = 18;
static constexpr uint8_t PIN_CAN_MISO = 19;
static constexpr uint8_t PIN_CAN_MOSI = 23;
static constexpr uint8_t PIN_CAN_CS   = 5;
static constexpr uint8_t PIN_CAN_INT  = 4;

static constexpr uint8_t ODRIVE_NODE_ID = 0x02;
static constexpr CAN_SPEED MCP_SPEED = CAN_1000KBPS;
static constexpr CAN_CLOCK MCP_CLOCK = MCP_8MHZ;

static constexpr uint32_t ENCODER_REQUEST_PERIOD_MS = 100;
static constexpr uint32_t BUS_REQUEST_PERIOD_MS = 250;
static constexpr uint32_t ERROR_REQUEST_PERIOD_MS = 1000;
static constexpr uint32_t SUMMARY_PRINT_PERIOD_MS = 500;

static constexpr uint8_t CMD_HEARTBEAT = 0x01;
static constexpr uint8_t CMD_GET_ERROR = 0x03;
static constexpr uint8_t CMD_GET_ENCODER_ESTIMATES = 0x09;
static constexpr uint8_t CMD_GET_BUS_VOLTAGE_CURRENT = 0x17;

MCP2515 g_can(PIN_CAN_CS);

struct ODriveState {
  bool haveHeartbeat = false;
  bool haveEncoder = false;
  bool haveBus = false;
  bool haveError = false;

  uint32_t heartbeatActiveErrors = 0;
  uint8_t axisState = 0;

  uint32_t activeErrors = 0;
  uint32_t disarmReason = 0;

  float posTurns = 0.0f;
  float velTurnsPerSecond = 0.0f;
  float busVoltage = 0.0f;
  float busCurrent = 0.0f;

  unsigned long lastHeartbeatMs = 0;
  unsigned long lastEncoderMs = 0;
  unsigned long lastBusMs = 0;
  unsigned long lastErrorMs = 0;
};

ODriveState g_state;

static inline uint32_t makeCanId(uint8_t nodeId, uint8_t cmdId) {
  return (static_cast<uint32_t>(nodeId) << 5) | (cmdId & 0x1F);
}

static inline uint32_t getStandardId(const can_frame &frame) {
  return frame.can_id & CAN_SFF_MASK;
}

static inline bool isExtendedFrame(const can_frame &frame) {
  return (frame.can_id & CAN_EFF_FLAG) != 0;
}

static inline bool isRemoteFrame(const can_frame &frame) {
  return (frame.can_id & CAN_RTR_FLAG) != 0;
}

static inline uint8_t extractNodeId(uint32_t standardId) {
  return static_cast<uint8_t>((standardId >> 5) & 0x3F);
}

static inline uint8_t extractCommandId(uint32_t standardId) {
  return static_cast<uint8_t>(standardId & 0x1F);
}

static inline uint32_t bytesToU32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

static inline float bytesToFloat(const uint8_t *data) {
  float value = 0.0f;
  memcpy(&value, data, sizeof(value));
  return value;
}

const char *axisStateName(uint8_t state) {
  switch (state) {
    case 0:  return "UNDEFINED";
    case 1:  return "IDLE";
    case 2:  return "STARTUP_SEQUENCE";
    case 3:  return "FULL_CALIBRATION_SEQUENCE";
    case 4:  return "MOTOR_CALIBRATION";
    case 5:  return "ENCODER_INDEX_SEARCH";
    case 6:  return "ENCODER_OFFSET_CALIBRATION";
    case 7:  return "CLOSED_LOOP_CONTROL_OLD";
    case 8:  return "CLOSED_LOOP_CONTROL";
    case 9:  return "LOCKIN_SPIN";
    case 10: return "ENCODER_DIR_FIND";
    case 11: return "HOMING";
    case 12: return "ENCODER_HALL_POLARITY_CALIBRATION";
    case 13: return "ENCODER_HALL_PHASE_CALIBRATION";
    default: return "UNKNOWN";
  }
}

void printFrame(const can_frame &frame, const char *prefix) {
  const uint32_t standardId = getStandardId(frame);
  Serial.printf("%s id=0x%03lX ext=%d rtr=%d dlc=%u data=",
                prefix,
                static_cast<unsigned long>(standardId),
                isExtendedFrame(frame) ? 1 : 0,
                isRemoteFrame(frame) ? 1 : 0,
                static_cast<unsigned>(frame.can_dlc));

  if (frame.can_dlc == 0) {
    Serial.print("(empty)");
  } else {
    for (uint8_t i = 0; i < frame.can_dlc; ++i) {
      Serial.printf("%02X", frame.data[i]);
      if (i + 1 < frame.can_dlc) {
        Serial.print(" ");
      }
    }
  }

  Serial.println();
}

bool configureCan() {
  g_can.reset();

  const MCP2515::ERROR bitrateResult = g_can.setBitrate(MCP_SPEED, MCP_CLOCK);
  if (bitrateResult != MCP2515::ERROR_OK) {
    Serial.printf("setBitrate failed: %d\n", static_cast<int>(bitrateResult));
    return false;
  }

  const MCP2515::ERROR modeResult = g_can.setNormalMode();
  if (modeResult != MCP2515::ERROR_OK) {
    Serial.printf("setNormalMode failed: %d\n", static_cast<int>(modeResult));
    return false;
  }

  Serial.println("MCP2515 configured in NORMAL mode.");
  return true;
}

bool sendRemoteRequest(uint8_t cmdId, uint8_t expectedLen) {
  can_frame frame = {};
  frame.can_id = makeCanId(ODRIVE_NODE_ID, cmdId) | CAN_RTR_FLAG;
  frame.can_dlc = expectedLen;

  const MCP2515::ERROR result = g_can.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d for RTR id=0x%03lX\n",
                  static_cast<int>(result),
                  static_cast<unsigned long>(frame.can_id & CAN_SFF_MASK));
    return false;
  }

  return true;
}

void decodeFrame(const can_frame &frame) {
  if (isExtendedFrame(frame) || isRemoteFrame(frame)) {
    return;
  }

  const uint32_t standardId = getStandardId(frame);
  const uint8_t nodeId = extractNodeId(standardId);
  const uint8_t cmdId = extractCommandId(standardId);

  if (nodeId != ODRIVE_NODE_ID) {
    return;
  }

  switch (cmdId) {
    case CMD_HEARTBEAT:
      if (frame.can_dlc >= 5) {
        g_state.haveHeartbeat = true;
        g_state.heartbeatActiveErrors = bytesToU32(frame.data);
        g_state.axisState = frame.data[4];
        g_state.lastHeartbeatMs = millis();
      }
      break;

    case CMD_GET_ERROR:
      if (frame.can_dlc >= 8) {
        g_state.haveError = true;
        g_state.activeErrors = bytesToU32(frame.data);
        g_state.disarmReason = bytesToU32(frame.data + 4);
        g_state.lastErrorMs = millis();
      }
      break;

    case CMD_GET_ENCODER_ESTIMATES:
      if (frame.can_dlc >= 8) {
        g_state.haveEncoder = true;
        g_state.posTurns = bytesToFloat(frame.data);
        g_state.velTurnsPerSecond = bytesToFloat(frame.data + 4);
        g_state.lastEncoderMs = millis();
      }
      break;

    case CMD_GET_BUS_VOLTAGE_CURRENT:
      if (frame.can_dlc >= 8) {
        g_state.haveBus = true;
        g_state.busVoltage = bytesToFloat(frame.data);
        g_state.busCurrent = bytesToFloat(frame.data + 4);
        g_state.lastBusMs = millis();
      }
      break;

    default:
      printFrame(frame, "RX unknown");
      break;
  }
}

void processIncomingFrames() {
  can_frame frame = {};

  while (digitalRead(PIN_CAN_INT) == LOW) {
    if (g_can.readMessage(&frame) != MCP2515::ERROR_OK) {
      break;
    }
    decodeFrame(frame);
  }
}

void printSummary() {
  Serial.print("node=0x");
  if (ODRIVE_NODE_ID < 0x10) {
    Serial.print('0');
  }
  Serial.print(ODRIVE_NODE_ID, HEX);

  Serial.print(" heartbeat=");
  Serial.print(g_state.haveHeartbeat ? "yes" : "no");
  Serial.print(" state=");
  Serial.print(axisStateName(g_state.axisState));
  Serial.print(" hb_err=0x");
  Serial.print(g_state.heartbeatActiveErrors, HEX);

  Serial.print(" | encoder=");
  Serial.print(g_state.haveEncoder ? "yes" : "no");
  Serial.print(" pos_turns=");
  Serial.print(g_state.posTurns, 6);
  Serial.print(" vel_turns_s=");
  Serial.print(g_state.velTurnsPerSecond, 6);

  Serial.print(" | bus=");
  Serial.print(g_state.haveBus ? "yes" : "no");
  Serial.print(" V=");
  Serial.print(g_state.busVoltage, 3);
  Serial.print(" I=");
  Serial.print(g_state.busCurrent, 3);

  Serial.print(" | error=");
  Serial.print(g_state.haveError ? "yes" : "no");
  Serial.print(" active=0x");
  Serial.print(g_state.activeErrors, HEX);
  Serial.print(" disarm=0x");
  Serial.println(g_state.disarmReason, HEX);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  pinMode(PIN_CAN_INT, INPUT);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

  Serial.println();
  Serial.println("ESP32 + MCP2515 ODrive reader");
  Serial.printf("ODrive node=0x%02X bitrate=1 Mbps MCP clock=%s\n",
                static_cast<unsigned>(ODRIVE_NODE_ID),
                MCP_CLOCK == MCP_8MHZ ? "8 MHz" : "16/20 MHz");
  Serial.printf("Expected IDs: heartbeat=0x%03lX error=0x%03lX encoder=0x%03lX bus=0x%03lX\n",
                static_cast<unsigned long>(makeCanId(ODRIVE_NODE_ID, CMD_HEARTBEAT)),
                static_cast<unsigned long>(makeCanId(ODRIVE_NODE_ID, CMD_GET_ERROR)),
                static_cast<unsigned long>(makeCanId(ODRIVE_NODE_ID, CMD_GET_ENCODER_ESTIMATES)),
                static_cast<unsigned long>(makeCanId(ODRIVE_NODE_ID, CMD_GET_BUS_VOLTAGE_CURRENT)));

  if (!configureCan()) {
    Serial.println("CAN configuration failed. Check MCP2515 crystal (8 vs 16 MHz), wiring, and power.");
  }
}

void loop() {
  static unsigned long lastEncoderRequestMs = 0;
  static unsigned long lastBusRequestMs = 0;
  static unsigned long lastErrorRequestMs = 0;
  static unsigned long lastSummaryPrintMs = 0;

  processIncomingFrames();

  const unsigned long now = millis();

  if (now - lastEncoderRequestMs >= ENCODER_REQUEST_PERIOD_MS) {
    lastEncoderRequestMs = now;
    sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8);
  }

  if (now - lastBusRequestMs >= BUS_REQUEST_PERIOD_MS) {
    lastBusRequestMs = now;
    sendRemoteRequest(CMD_GET_BUS_VOLTAGE_CURRENT, 8);
  }

  if (now - lastErrorRequestMs >= ERROR_REQUEST_PERIOD_MS) {
    lastErrorRequestMs = now;
    sendRemoteRequest(CMD_GET_ERROR, 8);
  }

  if (now - lastSummaryPrintMs >= SUMMARY_PRINT_PERIOD_MS) {
    lastSummaryPrintMs = now;
    printSummary();
  }
}
