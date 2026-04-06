#pragma once
#include <Arduino.h>
#include <mcp2515.h>
#include "../esp32_mcp2515_odrive_sniffer/odrive_can_support.h"
#include "../esp32_mcp2515_odrive_sniffer/odrive_can_state.h"
#include "dicerox_mixed_motor_config.h"

// ── Compile-time constants ────────────────────────────────────────────────────
constexpr unsigned long UNIFIED_SERIAL_BAUD         = 115200;
constexpr unsigned long REPLY_TIMEOUT_MS            = 500;
constexpr unsigned long ODRIVE_HEARTBEAT_STALE_MS   = 2000;
constexpr unsigned long ODRIVE_ENCODER_STALE_MS     = 1000;
constexpr uint8_t       SEND_RETRY_COUNT            = 6;
constexpr unsigned long SEND_RETRY_DELAY_MS         = 2;

constexpr uint8_t LK_CMD_MOTOR_OFF              = 0x80;
constexpr uint8_t LK_CMD_MOTOR_STOP             = 0x81;
constexpr uint8_t LK_CMD_MOTOR_ON               = 0x88;
constexpr uint8_t LK_CMD_READ_MULTI_LOOP_ANGLE  = 0x92;
constexpr uint8_t LK_CMD_MULTI_LOOP_CONTROL_2   = 0xA4;

constexpr uint8_t ZE_CMD_READ_ABSOLUTE_ANGLES   = 0xA3;
constexpr uint8_t ZE_CMD_SET_POSITION_MAX_SPEED = 0xB2;
constexpr uint8_t ZE_CMD_ABSOLUTE_POSITION      = 0xC2;
constexpr uint8_t ZE_CMD_RELATIVE_POSITION      = 0xC3;
constexpr uint8_t ZE_CMD_DISABLE_OUTPUT         = 0xCF;

// ── Types defined here so all headers can use them ────────────────────────────
enum UnifiedDriverType {
  UNIFIED_DRIVER_ODRIVE = 0,
  UNIFIED_DRIVER_LKTECH = 1,
  UNIFIED_DRIVER_ZE300  = 2,
};

struct UnifiedMotorTarget {
  const char      *name;
  const char      *alias;
  UnifiedDriverType driver_type;
  uint8_t          odrive_node_id;
  size_t           mixed_index;
};

struct MixedMotorRuntimeState {
  bool   have_zero_offset;
  bool   enabled;
  bool   ze300_speed_applied;
  double zero_offset_command_deg;
  double last_command_deg;
  double output_speed_dps;
  int32_t last_native_counts;
};

constexpr size_t kUnifiedMotorCount         = 6;
constexpr size_t kDefaultActiveTargetIndex  = 0;
constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 192;

// ── Forward-declarations of globals (defined in the .ino) ────────────────────
extern MCP2515                  g_mcp2515;
extern struct can_frame         g_rx_frame;

extern odrive_can_sniffer::CanMode    g_mode;
extern odrive_can_sniffer::Oscillator g_oscillator;
extern bool                     g_show_all_frames;
extern bool                     g_print_odrive_telemetry;
extern bool                     g_stream_odrive_targets;
extern size_t                   g_active_target_index;
extern unsigned long            g_pause_odrive_stream_until_ms;

extern odrive_can_sniffer::NodeRuntimeState g_odrive_states[odrive_can_sniffer::SUPPORTED_NODE_COUNT];
extern MixedMotorRuntimeState   g_mixed_states[kMixedMotorConfigCount];
extern char                     g_serial_line[SERIAL_COMMAND_BUFFER_SIZE];
extern size_t                   g_serial_line_len;

// ── kUnifiedTargets – defined once in dicerox_joint_control.h (inline array) ──
// Forward-declared here so other headers can reference it.
extern const UnifiedMotorTarget kUnifiedTargets[kUnifiedMotorCount];
