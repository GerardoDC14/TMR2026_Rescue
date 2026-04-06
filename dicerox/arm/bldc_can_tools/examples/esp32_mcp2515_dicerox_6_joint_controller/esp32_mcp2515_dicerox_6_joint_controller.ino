#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <math.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif
#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000UL
#endif
#ifndef CAN_SFF_MASK
#define CAN_SFF_MASK 0x7FFU
#endif

#include "../esp32_mcp2515_odrive_sniffer/odrive_can_support.h"
#include "../esp32_mcp2515_odrive_sniffer/odrive_can_state.h"
#include "dicerox_mixed_motor_config.h"
#include "dicerox_joint_control.h"   // pulls in can_bus + odrive + mixed
#include "dicerox_serial_ui.h"

using namespace odrive_can_sniffer;

// ── Global state ─────────────────────────────────────────────────────────────
MCP2515 g_mcp2515(PIN_CAN_CS);
struct can_frame g_rx_frame;

CanMode   g_mode               = CanMode::Normal;
Oscillator g_oscillator        = Oscillator::MHz8;
bool      g_show_all_frames    = false;
bool      g_print_odrive_telemetry = false;
bool      g_stream_odrive_targets  = true;
size_t    g_active_target_index    = kDefaultActiveTargetIndex;
unsigned long g_pause_odrive_stream_until_ms = 0;

NodeRuntimeState       g_odrive_states[SUPPORTED_NODE_COUNT];
MixedMotorRuntimeState g_mixed_states[kMixedMotorConfigCount] = {};
char   g_serial_line[SERIAL_COMMAND_BUFFER_SIZE] = {};
size_t g_serial_line_len = 0;

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(UNIFIED_SERIAL_BAUD);
  delay(1000);

  initializeOdriveStates();
  initializeMixedStates();

  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

  Serial.println();
  Serial.println("ESP32 + MCP2515 Dicerox 6-joint controller starting...");
  Serial.println("Shared CAN bus:");
  Serial.println("  joint1-3 = ODrive CANSimple (0x10, 0x11, 0x12)");
  Serial.println("  joint4   = ZE300 (device 1)");
  Serial.println("  joint5-6 = LKTech (IDs 14, 15)");

  if (!configureCan()) {
    Serial.println("initial CAN configuration failed");
  }

  runAutomaticOdriveBringupAllNodes();
  printConfig();
  printAllStatus();
  printHelp();
}

void loop() {
  processSerialInput();
  processIncomingFrames();
  serviceOdriveTargetStreaming();
  delay(1);
}
