#pragma once

#include <Arduino.h>
#include <mcp2515.h>

struct LKTechMotorConfig {
  const char *name;
  uint8_t motor_id;
  double gear_ratio;
  uint16_t default_motor_speed_dps;
};

static constexpr CAN_SPEED kFixedCanSpeed = CAN_500KBPS;
static constexpr CAN_CLOCK kFixedCanClock = MCP_8MHZ;
static constexpr uint16_t kLKTechStdIdBase = 0x140;

static constexpr LKTechMotorConfig kMotorConfigs[] = {
    {"joint5", 14, 10.0, 3600},
    {"joint6", 15, 10.0, 3600},
};

static constexpr size_t kMotorConfigCount = sizeof(kMotorConfigs) / sizeof(kMotorConfigs[0]);
static constexpr size_t kDefaultActiveMotorIndex = 1;  // joint6
