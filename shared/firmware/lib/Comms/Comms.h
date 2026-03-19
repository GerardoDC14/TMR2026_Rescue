#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── Comms ────────────────────────────────────────────────────────────────────
// Binary framed UART protocol between the ESP32 and the Asus NUC mini PC.
//
// Frame layout:
//   [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
//   CRC = XOR of TYPE, LEN_H, LEN_L, and all PAYLOAD bytes.
//
// The protocol is symmetric; either side may initiate any message type.
// All public methods are safe to call from any task; internal TX is queued.
//
// Incoming message callbacks are dispatched from within tick(), which must be
// called continuously from the comms task.

using ArmJointsCallback    = void(*)(const ArmJointsPayload&);
using SensorEnableCallback = void(*)(uint8_t mask);
using EstopCallback        = void(*)(bool active);

class Comms {
public:
    static void begin();

    // Called continuously from the comms task (Core 0).
    // Drains UART RX, parses frames, and flushes the TX queue.
    static void tick();

    // ── Outgoing messages (enqueue for next tick) ──────────────────────────────
    static void sendTelemetry(const TelemetryPayload& payload);
    static void sendMagData(const MagData& mag);
    static void sendThermalData(const ThermalData& thermal);
    static void sendGasData(const GasData& gas);
    static void sendStatus(const SystemStatus& status);

    // ── Callback registration ─────────────────────────────────────────────────
    static void onArmJoints(ArmJointsCallback cb)       { s_cb_arm    = cb; }
    static void onSensorEnable(SensorEnableCallback cb)  { s_cb_sensor = cb; }
    static void onEstop(EstopCallback cb)                { s_cb_estop  = cb; }

    // True if a full valid frame has been received within the last second.
    static bool isConnected();

private:
    // TX helper
    static void sendFrame(uint8_t type, const uint8_t* payload, uint16_t len);
    static uint8_t computeCRC(uint8_t type, uint16_t len, const uint8_t* payload);

    // RX state machine
    enum class RxState : uint8_t {
        SOF0, SOF1, TYPE, LEN_H, LEN_L, PAYLOAD, CRC
    };

    static void processFrame(uint8_t type, const uint8_t* buf, uint16_t len);

    static RxState  s_rx_state;
    static uint8_t  s_rx_type;
    static uint16_t s_rx_len;
    static uint16_t s_rx_idx;
    static uint8_t  s_rx_buf[PROTO_MAX_PAYLOAD];
    static uint8_t  s_rx_crc;

    static uint32_t s_last_rx_ms;

    static ArmJointsCallback    s_cb_arm;
    static SensorEnableCallback s_cb_sensor;
    static EstopCallback        s_cb_estop;
};
