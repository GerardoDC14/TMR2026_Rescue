#pragma once
#include "dicerox_globals.h"

using namespace odrive_can_sniffer;

// Forward declaration so drainReceiveQueue (defined here) can call it.
// handleReceivedCanFrame is defined in dicerox_odrive_control.h.
inline void handleReceivedCanFrame(const can_frame &frame, bool already_printed);

inline void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

inline void printCanFrame(const can_frame &frame, const char *prefix) {
  const uint32_t arbitration_id = frame.can_id & CAN_SFF_MASK;
  const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  const bool remote   = (frame.can_id & CAN_RTR_FLAG) != 0;

  Serial.printf("%s id=0x%0*lX ext=%d rtr=%d dlc=%u data=",
                prefix,
                extended ? 8 : 3,
                static_cast<unsigned long>(arbitration_id),
                extended ? 1 : 0,
                remote   ? 1 : 0,
                static_cast<unsigned>(frame.can_dlc));
  for (uint8_t index = 0; index < frame.can_dlc; ++index) {
    Serial.printf("%02X", frame.data[index]);
    if (index + 1 < frame.can_dlc) {
      Serial.print(" ");
    }
  }

  if (!extended) {
    const uint8_t node_id = extractNodeId(arbitration_id);
    NodeRuntimeState *state = findNodeState(g_odrive_states, SUPPORTED_NODE_COUNT, node_id);
    if (state != nullptr) {
      Serial.print(" odrive_node=0x");
      printHexByte(node_id);
      Serial.print(" cmd=0x");
      printHexByte(extractCommandId(arbitration_id));
      Serial.print("(");
      Serial.print(commandName(extractCommandId(arbitration_id)));
      Serial.print(")");
    }
  }

  Serial.println();
}

inline bool applyCanMode() {
  switch (g_mode) {
    case CanMode::Normal:
      return g_mcp2515.setNormalMode() == MCP2515::ERROR_OK;
    case CanMode::ListenOnly:
      return g_mcp2515.setListenOnlyMode() == MCP2515::ERROR_OK;
    case CanMode::Loopback:
      return g_mcp2515.setLoopbackMode() == MCP2515::ERROR_OK;
    default:
      return false;
  }
}

inline bool configureCan() {
  g_mcp2515.reset();

  const auto osc = g_oscillator == Oscillator::MHz8 ? MCP_8MHZ : MCP_16MHZ;
  if (g_mcp2515.setBitrate(CAN_1000KBPS, osc) != MCP2515::ERROR_OK) {
    Serial.println("ERROR: failed to set MCP2515 bitrate");
    return false;
  }
  if (!applyCanMode()) {
    Serial.println("ERROR: failed to set MCP2515 mode");
    return false;
  }
  Serial.println("MCP2515 configured OK");
  return true;
}

inline bool ensureCanNormalMode(const char *context) {
  if (g_mode == CanMode::Normal) {
    return true;
  }

  g_mode = CanMode::Normal;
  if (!configureCan()) {
    Serial.print("Failed to switch CAN mode to normal");
    if (context != nullptr) {
      Serial.print(" for ");
      Serial.print(context);
    }
    Serial.println(".");
    return false;
  }

  if (context != nullptr) {
    Serial.print("CAN mode switched to normal for ");
    Serial.println(context);
  }
  return true;
}

inline bool sendCanFrame(const can_frame &frame, const char *message = nullptr, bool log_frame = true) {
  can_frame tx = frame;
  if (log_frame) {
    printCanFrame(tx, "TX");
  }

  MCP2515::ERROR last_error = MCP2515::ERROR_OK;
  for (uint8_t attempt = 0; attempt < SEND_RETRY_COUNT; ++attempt) {
    last_error = g_mcp2515.sendMessage(&tx);
    if (last_error == MCP2515::ERROR_OK) {
      if (message != nullptr) {
        Serial.println(message);
      }
      return true;
    }

    if (attempt + 1 < SEND_RETRY_COUNT) {
      delay(SEND_RETRY_DELAY_MS);
    }
  }

  if (log_frame || message != nullptr) {
    Serial.printf("ERROR: send failed code=%d id=0x%03lX\n",
                  static_cast<int>(last_error),
                  static_cast<unsigned long>(tx.can_id & CAN_SFF_MASK));
  }
  return false;
}

inline bool sendStandardFrame(uint16_t arbitration_id, const uint8_t *data, uint8_t dlc, bool log_frame = true) {
  can_frame frame = {};
  frame.can_id  = arbitration_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);
  return sendCanFrame(frame, nullptr, log_frame);
}

inline void drainReceiveQueue() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    if (g_show_all_frames) {
      printCanFrame(frame, "RX(stale)");
    }
    handleReceivedCanFrame(frame, true);
  }
}
