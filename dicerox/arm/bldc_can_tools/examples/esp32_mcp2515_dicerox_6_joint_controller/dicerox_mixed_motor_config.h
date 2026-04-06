#pragma once

#include <Arduino.h>
#include <mcp2515.h>

enum MixedDriverType {
  DRIVER_LKTECH = 0,
  DRIVER_ZE300 = 1,
};

struct MixedMotorConfig {
  const char *joint_name;
  const char *alias_name;
  MixedDriverType driver_type;
  uint8_t device_id;
  double physical_gear_ratio;
  double position_command_ratio;
  double speed_command_ratio;
  double default_output_speed_dps;
  bool tagged_requests;
};

static constexpr CAN_SPEED kMixedCanSpeed = CAN_1000KBPS;
static constexpr CAN_CLOCK kMixedCanClock = MCP_8MHZ;

static constexpr uint16_t kLKTechStdIdBase = 0x140;
static constexpr uint16_t kZE300TaggedRequestMask = 0x100;
static constexpr uint16_t kZE300CountsPerRev = 16384;

static constexpr MixedMotorConfig kMixedMotorConfigs[] = {
    {"joint4", "ze300", DRIVER_ZE300, 1, 8.0, 1.0, 1.0, 100.0, true},
    {"joint5", "lktech14", DRIVER_LKTECH, 14, 10.0, 10.0, 10.0, 360.0, true},
    {"joint6", "lktech15", DRIVER_LKTECH, 15, 10.0, 10.0, 10.0, 360.0, true},
};

static constexpr size_t kMixedMotorConfigCount =
    sizeof(kMixedMotorConfigs) / sizeof(kMixedMotorConfigs[0]);
