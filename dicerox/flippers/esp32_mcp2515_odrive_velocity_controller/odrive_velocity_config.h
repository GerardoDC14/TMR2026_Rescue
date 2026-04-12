#pragma once

#include <Arduino.h>
#include <mcp2515.h>

namespace odrive_velocity_config {

// Serial
constexpr uint32_t kSerialBaud = 115200;
constexpr size_t kSerialLineBufferSize = 96;

// ESP32 <-> MCP2515 wiring
constexpr uint8_t kPinCanSck = 18;
constexpr uint8_t kPinCanMiso = 19;
constexpr uint8_t kPinCanMosi = 23;
constexpr uint8_t kPinCanCs = 5;
constexpr uint8_t kPinCanInt = 4;

// ODrive bus configuration
constexpr uint8_t kOdriveNodeId = 0x02;
constexpr CAN_SPEED kMcpSpeed = CAN_1000KBPS;
constexpr CAN_CLOCK kMcpClock = MCP_8MHZ;

// Motion/control configuration
constexpr uint32_t kClosedLoopConfirmTimeoutMs = 800;
constexpr uint32_t kClosedLoopRetryIntervalMs = 100;
constexpr uint32_t kHeartbeatStaleMs = 500;

// Telemetry polling
constexpr uint32_t kEncoderRequestPeriodMs = 100;
constexpr uint32_t kBusRequestPeriodMs = 250;
constexpr uint32_t kErrorRequestPeriodMs = 1000;
constexpr uint32_t kTelemetryPrintPeriodMs = 500;

}  // namespace odrive_velocity_config
