#pragma once

#include "odrive_can_support.h"

namespace odrive_can_sniffer {

inline bool isSupportedNodeId(uint8_t nodeId) {
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    if (SUPPORTED_NODE_IDS[i] == nodeId) {
      return true;
    }
  }
  return false;
}

inline NodeRuntimeState *findNodeState(NodeRuntimeState *states, size_t stateCount, uint8_t nodeId) {
  for (size_t i = 0; i < stateCount; ++i) {
    if (states[i].nodeId == nodeId) {
      return &states[i];
    }
  }
  return nullptr;
}

inline const NodeRuntimeState *findNodeState(const NodeRuntimeState *states, size_t stateCount, uint8_t nodeId) {
  for (size_t i = 0; i < stateCount; ++i) {
    if (states[i].nodeId == nodeId) {
      return &states[i];
    }
  }
  return nullptr;
}

inline NodeRuntimeState &activeNodeState(NodeRuntimeState *states, size_t stateCount, uint8_t activeNodeId) {
  NodeRuntimeState *state = findNodeState(states, stateCount, activeNodeId);
  return state != nullptr ? *state : states[0];
}

inline void resetNodeState(NodeRuntimeState &state) {
  state.haveLatestEncoderEstimate = false;
  state.haveZeroReference = false;
  state.haveActiveTarget = false;
  state.haveHeartbeat = false;
  state.haveErrorStatus = false;
  state.lastEncoderPosTurns = 0.0f;
  state.lastEncoderVelTurnsPerSecond = 0.0f;
  state.zeroReferenceTurns = 0.0f;
  state.lastRelativeCommandTurns = 0.0f;
  state.lastRelativeCommandDegrees = 0.0f;
  state.lastAbsoluteCommandTurns = 0.0f;
  state.lastAxisState = AXIS_STATE_IDLE;
  state.lastHeartbeatActiveErrors = 0;
  state.lastErrorActiveErrors = 0;
  state.lastDisarmReason = 0;
  state.lastTargetStreamMs = 0;
  state.lastHeartbeatMs = 0;
  state.lastErrorMs = 0;
  state.lastEncoderEstimateMs = 0;
  state.lastEncoderTelemetryPrintMs = 0;
}

inline void initializeNodeStates(NodeRuntimeState *states, size_t stateCount) {
  const size_t count = stateCount < SUPPORTED_NODE_COUNT ? stateCount : SUPPORTED_NODE_COUNT;
  for (size_t i = 0; i < count; ++i) {
    states[i].nodeId = SUPPORTED_NODE_IDS[i];
    resetNodeState(states[i]);
  }
}

}  // namespace odrive_can_sniffer
