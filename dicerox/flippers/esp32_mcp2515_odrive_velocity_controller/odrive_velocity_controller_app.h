#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "odrive_velocity_config.h"
#include "odrive_velocity_protocol.h"

namespace odrive_velocity {

namespace cfg = odrive_velocity_config;
namespace proto = odrive_velocity_protocol;

class ControllerApp {
public:
  ControllerApp() : canController_(cfg::kPinCanCs) {}

  void begin() {
    Serial.begin(cfg::kSerialBaud);
    delay(1000);

    pinMode(cfg::kPinCanInt, INPUT);
    SPI.begin(cfg::kPinCanSck, cfg::kPinCanMiso, cfg::kPinCanMosi, cfg::kPinCanCs);

    Serial.println();
    Serial.println("ESP32 + MCP2515 ODrive velocity controller");
    Serial.printf("Node=0x%02X bitrate=1 Mbps MCP clock=%s\n",
                  static_cast<unsigned>(cfg::kOdriveNodeId),
                  cfg::kMcpClock == MCP_8MHZ ? "8 MHz" : "16/20 MHz");
    Serial.printf("IDs: heartbeat=0x%03lX axis_state=0x%03lX set_mode=0x%03lX set_vel=0x%03lX\n",
                  static_cast<unsigned long>(proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdHeartbeat)),
                  static_cast<unsigned long>(proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetAxisState)),
                  static_cast<unsigned long>(proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetControllerMode)),
                  static_cast<unsigned long>(proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetInputVel)));

    if (!configureCan()) {
      Serial.println("CAN configuration failed. Check MCP2515 crystal, wiring, CANH/CANL, and bus power.");
    }

    printHelp();
  }

  void update() {
    handleSerialInput();
    processIncomingFrames();
    serviceTelemetryPolling();
  }

private:
  MCP2515 canController_;
  proto::ODriveState state_;
  bool printRawFrames_ = false;
  bool printTelemetry_ = false;
  char serialLine_[cfg::kSerialLineBufferSize] = {};
  size_t serialLineLen_ = 0;

  void printFrame(const can_frame &frame, const char *prefix) {
    Serial.printf("%s id=0x%03lX ext=%d rtr=%d dlc=%u data=",
                  prefix,
                  static_cast<unsigned long>(proto::getStandardId(frame)),
                  proto::isExtendedFrame(frame) ? 1 : 0,
                  proto::isRemoteFrame(frame) ? 1 : 0,
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
    canController_.reset();

    const MCP2515::ERROR bitrateResult = canController_.setBitrate(cfg::kMcpSpeed, cfg::kMcpClock);
    if (bitrateResult != MCP2515::ERROR_OK) {
      Serial.printf("setBitrate failed: %d\n", static_cast<int>(bitrateResult));
      return false;
    }

    const MCP2515::ERROR modeResult = canController_.setNormalMode();
    if (modeResult != MCP2515::ERROR_OK) {
      Serial.printf("setNormalMode failed: %d\n", static_cast<int>(modeResult));
      return false;
    }

    Serial.println("MCP2515 configured in NORMAL mode.");
    return true;
  }

  bool sendFrame(const can_frame &frame, const char *message = nullptr, bool logFrame = false) {
    can_frame tx = frame;

    if (printRawFrames_ || logFrame) {
      printFrame(tx, "TX");
    }

    const MCP2515::ERROR result = canController_.sendMessage(&tx);
    if (result != MCP2515::ERROR_OK) {
      Serial.printf("sendMessage failed: %d\n", static_cast<int>(result));
      return false;
    }

    if (message != nullptr && message[0] != '\0') {
      Serial.println(message);
    }

    return true;
  }

  bool sendRemoteRequest(uint8_t cmdId, uint8_t expectedLen, const char *message = nullptr, bool logFrame = false) {
    can_frame frame = {};
    frame.can_id = proto::makeCanId(cfg::kOdriveNodeId, cmdId) | CAN_RTR_FLAG;
    frame.can_dlc = expectedLen;
    return sendFrame(frame, message, logFrame);
  }

  bool sendAxisStateRequest(uint32_t axisState, const char *message = nullptr, bool logFrame = false) {
    can_frame frame = {};
    frame.can_id = proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetAxisState);
    frame.can_dlc = 4;
    proto::writeU32ToBytes(axisState, frame.data);
    return sendFrame(frame, message, logFrame);
  }

  bool sendControllerModeVelocityVelRamp(const char *message = nullptr, bool logFrame = false) {
    can_frame frame = {};
    frame.can_id = proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetControllerMode);
    frame.can_dlc = 8;
    proto::writeU32ToBytes(proto::kControlModeVelocityControl, &frame.data[0]);
    proto::writeU32ToBytes(proto::kInputModeVelRamp, &frame.data[4]);
    return sendFrame(frame, message, logFrame);
  }

  bool sendInputVelocity(float velTurnsPerSecond,
                         float torqueFeedforwardNm = 0.0f,
                         const char *message = nullptr,
                         bool logFrame = false) {
    can_frame frame = {};
    frame.can_id = proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdSetInputVel);
    frame.can_dlc = 8;
    proto::writeFloatToBytes(velTurnsPerSecond, &frame.data[0]);
    proto::writeFloatToBytes(torqueFeedforwardNm, &frame.data[4]);
    if (sendFrame(frame, message, logFrame)) {
      state_.lastCommandedVelTurnsPerSecond = velTurnsPerSecond;
      return true;
    }
    return false;
  }

  bool sendClearErrors(bool identify = false, const char *message = nullptr, bool logFrame = false) {
    can_frame frame = {};
    frame.can_id = proto::makeCanId(cfg::kOdriveNodeId, proto::kCmdClearErrors);
    frame.can_dlc = 1;
    frame.data[0] = identify ? 1 : 0;
    return sendFrame(frame, message, logFrame);
  }

  bool heartbeatFresh() const {
    return state_.haveHeartbeat && millis() - state_.lastHeartbeatMs <= cfg::kHeartbeatStaleMs;
  }

  bool isClosedLoopActive() const {
    return heartbeatFresh() &&
           state_.axisState == proto::kAxisStateClosedLoopControl &&
           state_.heartbeatActiveErrors == 0;
  }

  void decodeFrame(const can_frame &frame) {
    if (proto::isExtendedFrame(frame)) {
      return;
    }

    if (printRawFrames_) {
      printFrame(frame, "RX");
    }

    if (proto::isRemoteFrame(frame)) {
      return;
    }

    const uint32_t standardId = proto::getStandardId(frame);
    if (proto::extractNodeId(standardId) != cfg::kOdriveNodeId) {
      return;
    }

    switch (proto::extractCommandId(standardId)) {
      case proto::kCmdHeartbeat:
        if (frame.can_dlc >= 5) {
          const uint32_t activeErrors = proto::bytesToU32(frame.data);
          const uint8_t axisState = frame.data[4];
          const bool changed = !state_.haveHeartbeat ||
                               activeErrors != state_.heartbeatActiveErrors ||
                               axisState != state_.axisState;

          state_.haveHeartbeat = true;
          state_.heartbeatActiveErrors = activeErrors;
          state_.axisState = axisState;
          state_.lastHeartbeatMs = millis();

          if (changed) {
            Serial.printf("Heartbeat state=%s active_errors=0x%lX\n",
                          proto::axisStateName(axisState),
                          static_cast<unsigned long>(activeErrors));
          }
        }
        break;

      case proto::kCmdGetError:
        if (frame.can_dlc >= 8) {
          state_.haveError = true;
          state_.activeErrors = proto::bytesToU32(frame.data);
          state_.disarmReason = proto::bytesToU32(frame.data + 4);
          state_.lastErrorMs = millis();
        }
        break;

      case proto::kCmdGetEncoderEstimates:
        if (frame.can_dlc >= 8) {
          state_.haveEncoder = true;
          state_.posTurns = proto::bytesToFloat(frame.data);
          state_.velTurnsPerSecond = proto::bytesToFloat(frame.data + 4);
          state_.lastEncoderMs = millis();
        }
        break;

      case proto::kCmdGetBusVoltageCurrent:
        if (frame.can_dlc >= 8) {
          state_.haveBus = true;
          state_.busVoltage = proto::bytesToFloat(frame.data);
          state_.busCurrent = proto::bytesToFloat(frame.data + 4);
          state_.lastBusMs = millis();
        }
        break;

      default:
        break;
    }
  }

  void processIncomingFrames() {
    can_frame frame = {};

    while (digitalRead(cfg::kPinCanInt) == LOW) {
      if (canController_.readMessage(&frame) != MCP2515::ERROR_OK) {
        break;
      }
      decodeFrame(frame);
    }
  }

  void printStatus() const {
    Serial.print("node=0x");
    if (cfg::kOdriveNodeId < 0x10) {
      Serial.print('0');
    }
    Serial.print(cfg::kOdriveNodeId, HEX);

    Serial.print(" state=");
    Serial.print(proto::axisStateName(state_.axisState));
    Serial.print(" hb_ok=");
    Serial.print(heartbeatFresh() ? "yes" : "no");
    Serial.print(" hb_err=0x");
    Serial.print(state_.heartbeatActiveErrors, HEX);

    Serial.print(" | cmd_vel=");
    Serial.print(state_.lastCommandedVelTurnsPerSecond, 3);
    Serial.print(" turns/s");

    Serial.print(" | enc=");
    Serial.print(state_.haveEncoder ? "yes" : "no");
    Serial.print(" pos=");
    Serial.print(state_.posTurns, 4);
    Serial.print(" turns");
    Serial.print(" vel=");
    Serial.print(state_.velTurnsPerSecond, 4);
    Serial.print(" turns/s");

    Serial.print(" | bus=");
    Serial.print(state_.haveBus ? "yes" : "no");
    Serial.print(" V=");
    Serial.print(state_.busVoltage, 2);
    Serial.print(" I=");
    Serial.print(state_.busCurrent, 2);

    Serial.print(" | err=");
    Serial.print(state_.haveError ? "yes" : "no");
    Serial.print(" active=0x");
    Serial.print(state_.activeErrors, HEX);
    Serial.print(" disarm=0x");
    Serial.println(state_.disarmReason, HEX);
  }

  void printHelp() const {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  help              print this help");
    Serial.println("  status            print cached ODrive state");
    Serial.println("  on                clear errors, set velocity mode, request closed loop");
    Serial.println("  mode              send VELOCITY_CONTROL + VEL_RAMP");
    Serial.println("  vel <turns/s>     send Set_Input_Vel, for example: vel -5");
    Serial.println("  stop              send Set_Input_Vel = 0");
    Serial.println("  idle              request AXIS_STATE_IDLE");
    Serial.println("  clear             send Clear_Errors");
    Serial.println("  enc               request Get_Encoder_Estimates");
    Serial.println("  bus               request Get_Bus_Voltage_Current");
    Serial.println("  err               request Get_Error");
    Serial.println("  raw on|off        toggle CAN frame logging");
    Serial.println("  telemetry on|off  toggle periodic status print");
    Serial.println();
  }

  bool requestClosedLoopAndConfirm(uint32_t timeoutMs) {
    if (isClosedLoopActive()) {
      Serial.println("Closed loop already active.");
      return true;
    }

    sendAxisStateRequest(proto::kAxisStateClosedLoopControl, "Requested CLOSED_LOOP_CONTROL", true);

    const unsigned long startMs = millis();
    unsigned long lastRetryMs = startMs;

    while (millis() - startMs < timeoutMs) {
      processIncomingFrames();

      if (isClosedLoopActive() && state_.lastHeartbeatMs >= startMs) {
        Serial.println("Closed loop confirmed.");
        return true;
      }

      if (millis() - lastRetryMs >= cfg::kClosedLoopRetryIntervalMs) {
        lastRetryMs = millis();
        sendAxisStateRequest(proto::kAxisStateClosedLoopControl, nullptr, false);
      }

      delay(1);
    }

    Serial.println("Closed loop not confirmed.");
    sendRemoteRequest(proto::kCmdGetError, 8, "Requested Get_Error after closed-loop timeout", true);
    const unsigned long waitStart = millis();
    while (millis() - waitStart < 120) {
      processIncomingFrames();
      delay(1);
    }
    printStatus();
    return false;
  }

  bool startVelocityControl() {
    if (!sendClearErrors(false, "Sent Clear_Errors", true)) {
      return false;
    }

    if (!sendControllerModeVelocityVelRamp("Set controller mode to VELOCITY_CONTROL + VEL_RAMP", true)) {
      return false;
    }

    return requestClosedLoopAndConfirm(cfg::kClosedLoopConfirmTimeoutMs);
  }

  bool ensureClosedLoopForVelocity() {
    if (isClosedLoopActive()) {
      return true;
    }
    Serial.println("Closed loop is not active, starting velocity control sequence.");
    return startVelocityControl();
  }

  void handleSimpleCommand(const char *line) {
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0 || strcmp(line, "h") == 0) {
      printHelp();
      return;
    }

    if (strcmp(line, "status") == 0 || strcmp(line, "p") == 0) {
      printStatus();
      return;
    }

    if (strcmp(line, "on") == 0) {
      (void)startVelocityControl();
      return;
    }

    if (strcmp(line, "mode") == 0) {
      (void)sendControllerModeVelocityVelRamp("Set controller mode to VELOCITY_CONTROL + VEL_RAMP", true);
      return;
    }

    if (strcmp(line, "stop") == 0) {
      if (!ensureClosedLoopForVelocity()) {
        return;
      }
      (void)sendInputVelocity(0.0f, 0.0f, "Sent Set_Input_Vel = 0 turns/s", true);
      return;
    }

    if (strcmp(line, "idle") == 0) {
      (void)sendAxisStateRequest(proto::kAxisStateIdle, "Requested IDLE", true);
      return;
    }

    if (strcmp(line, "clear") == 0) {
      (void)sendClearErrors(false, "Sent Clear_Errors", true);
      return;
    }

    if (strcmp(line, "enc") == 0) {
      (void)sendRemoteRequest(proto::kCmdGetEncoderEstimates, 8, "Requested Get_Encoder_Estimates", true);
      return;
    }

    if (strcmp(line, "bus") == 0) {
      (void)sendRemoteRequest(proto::kCmdGetBusVoltageCurrent, 8, "Requested Get_Bus_Voltage_Current", true);
      return;
    }

    if (strcmp(line, "err") == 0) {
      (void)sendRemoteRequest(proto::kCmdGetError, 8, "Requested Get_Error", true);
      return;
    }

    if (strncmp(line, "vel ", 4) == 0) {
      char *endPtr = nullptr;
      const float velTurnsPerSecond = strtof(line + 4, &endPtr);
      if (endPtr == line + 4) {
        Serial.println("Invalid velocity. Example: vel -5");
        return;
      }

      while (*endPtr == ' ' || *endPtr == '\t') {
        ++endPtr;
      }
      if (*endPtr != '\0') {
        Serial.println("Unexpected extra text after velocity.");
        return;
      }

      if (!ensureClosedLoopForVelocity()) {
        return;
      }

      char message[64];
      snprintf(message, sizeof(message), "Sent Set_Input_Vel = %.3f turns/s", velTurnsPerSecond);
      (void)sendInputVelocity(velTurnsPerSecond, 0.0f, message, true);
      return;
    }

    if (strcmp(line, "raw on") == 0) {
      printRawFrames_ = true;
      Serial.println("raw_frames = ON");
      return;
    }

    if (strcmp(line, "raw off") == 0) {
      printRawFrames_ = false;
      Serial.println("raw_frames = OFF");
      return;
    }

    if (strcmp(line, "telemetry on") == 0) {
      printTelemetry_ = true;
      Serial.println("telemetry = ON");
      return;
    }

    if (strcmp(line, "telemetry off") == 0) {
      printTelemetry_ = false;
      Serial.println("telemetry = OFF");
      return;
    }

    Serial.print("Unknown command: ");
    Serial.println(line);
    printHelp();
  }

  void handleSerialInput() {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());

      if (c == '\r' || c == '\n') {
        if (serialLineLen_ > 0) {
          serialLine_[serialLineLen_] = '\0';
          handleSimpleCommand(serialLine_);
          serialLineLen_ = 0;
        }
        continue;
      }

      if (serialLineLen_ + 1 < cfg::kSerialLineBufferSize) {
        serialLine_[serialLineLen_++] = c;
      } else {
        Serial.println("Command too long, clearing input buffer.");
        serialLineLen_ = 0;
      }
    }
  }

  void serviceTelemetryPolling() {
    static unsigned long lastEncoderRequestMs = 0;
    static unsigned long lastBusRequestMs = 0;
    static unsigned long lastErrorRequestMs = 0;
    static unsigned long lastTelemetryPrintMs = 0;

    const unsigned long now = millis();

    if (now - lastEncoderRequestMs >= cfg::kEncoderRequestPeriodMs) {
      lastEncoderRequestMs = now;
      sendRemoteRequest(proto::kCmdGetEncoderEstimates, 8, nullptr, false);
    }

    if (now - lastBusRequestMs >= cfg::kBusRequestPeriodMs) {
      lastBusRequestMs = now;
      sendRemoteRequest(proto::kCmdGetBusVoltageCurrent, 8, nullptr, false);
    }

    if (now - lastErrorRequestMs >= cfg::kErrorRequestPeriodMs) {
      lastErrorRequestMs = now;
      sendRemoteRequest(proto::kCmdGetError, 8, nullptr, false);
    }

    if (printTelemetry_ && now - lastTelemetryPrintMs >= cfg::kTelemetryPrintPeriodMs) {
      lastTelemetryPrintMs = now;
      printStatus();
    }
  }
};

}  // namespace odrive_velocity
