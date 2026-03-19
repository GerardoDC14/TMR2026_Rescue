#include "Comms.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// ─── Static member definitions ───────────────────────────────────────────────
Comms::RxState  Comms::s_rx_state = Comms::RxState::SOF0;
uint8_t         Comms::s_rx_type   = 0;
uint16_t        Comms::s_rx_len    = 0;
uint16_t        Comms::s_rx_idx    = 0;
uint8_t         Comms::s_rx_buf[PROTO_MAX_PAYLOAD];
uint8_t         Comms::s_rx_crc    = 0;
uint32_t        Comms::s_last_rx_ms = 0;

ArmJointsCallback    Comms::s_cb_arm    = nullptr;
SensorEnableCallback Comms::s_cb_sensor = nullptr;
EstopCallback        Comms::s_cb_estop  = nullptr;

static HardwareSerial& s_uart = Serial;   // UART0 — shared with USB cable

// ─── Initialisation ───────────────────────────────────────────────────────────
void Comms::begin() {
    s_uart.begin(MINIPC_BAUD);   // UART0 pins (GPIO1/3) are fixed; no pin args needed
}

// ─── Main loop ────────────────────────────────────────────────────────────────
void Comms::tick() {
    // ── RX ───────────────────────────────────────────────────────────────────
    while (s_uart.available()) {
        uint8_t b = s_uart.read();

        switch (s_rx_state) {
            case RxState::SOF0:
                if (b == PROTO_SOF_0) s_rx_state = RxState::SOF1;
                break;

            case RxState::SOF1:
                s_rx_state = (b == PROTO_SOF_1) ? RxState::TYPE : RxState::SOF0;
                break;

            case RxState::TYPE:
                s_rx_type  = b;
                s_rx_crc   = b;   // CRC starts here
                s_rx_state = RxState::LEN_H;
                break;

            case RxState::LEN_H:
                s_rx_len   = (uint16_t)b << 8;
                s_rx_crc  ^= b;
                s_rx_state = RxState::LEN_L;
                break;

            case RxState::LEN_L:
                s_rx_len  |= b;
                s_rx_crc  ^= b;
                s_rx_idx   = 0;
                if (s_rx_len == 0) {
                    // Zero-length payload: go straight to CRC
                    s_rx_state = RxState::CRC;
                } else if (s_rx_len > PROTO_MAX_PAYLOAD) {
                    // Reject oversized frame
                    s_rx_state = RxState::SOF0;
                } else {
                    s_rx_state = RxState::PAYLOAD;
                }
                break;

            case RxState::PAYLOAD:
                s_rx_buf[s_rx_idx++] = b;
                s_rx_crc ^= b;
                if (s_rx_idx >= s_rx_len) s_rx_state = RxState::CRC;
                break;

            case RxState::CRC:
                if (b == s_rx_crc) {
                    s_last_rx_ms = millis();
                    processFrame(s_rx_type, s_rx_buf, s_rx_len);
                }
                // Always return to idle after CRC byte, valid or not
                s_rx_state = RxState::SOF0;
                break;
        }
    }
}

// ─── Incoming frame dispatch ──────────────────────────────────────────────────
void Comms::processFrame(uint8_t type, const uint8_t* buf, uint16_t len) {
    switch (type) {
        case MSG_ARM_JOINTS:
            if (len == sizeof(ArmJointsPayload) && s_cb_arm) {
                ArmJointsPayload p;
                memcpy(&p, buf, sizeof(p));
                s_cb_arm(p);
            }
            break;

        case MSG_SENSOR_ENABLE:
            if (len == sizeof(SensorEnablePayload) && s_cb_sensor) {
                s_cb_sensor(buf[0]);
            }
            break;

        case MSG_ESTOP:
            if (s_cb_estop) s_cb_estop(true);
            break;

        case MSG_ESTOP_CLEAR:
            if (s_cb_estop) s_cb_estop(false);
            break;

        default:
            // Unknown type — silently discard
            break;
    }
}

// ─── TX helpers ───────────────────────────────────────────────────────────────
uint8_t Comms::computeCRC(uint8_t type, uint16_t len, const uint8_t* payload) {
    uint8_t crc = type ^ (uint8_t)(len >> 8) ^ (uint8_t)(len & 0xFF);
    for (uint16_t i = 0; i < len; i++) crc ^= payload[i];
    return crc;
}

void Comms::sendFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
    uint8_t header[5] = {
        PROTO_SOF_0,
        PROTO_SOF_1,
        type,
        (uint8_t)(len >> 8),
        (uint8_t)(len & 0xFF)
    };
    uint8_t crc = computeCRC(type, len, payload);

    s_uart.write(header, 5);
    if (len > 0) s_uart.write(payload, len);
    s_uart.write(&crc, 1);
}

// ─── Outgoing message builders ────────────────────────────────────────────────
void Comms::sendTelemetry(const TelemetryPayload& p) {
    sendFrame(MSG_TELEMETRY,
              reinterpret_cast<const uint8_t*>(&p),
              sizeof(p));
}

void Comms::sendMagData(const MagData& mag) {
    MagPayload p;
    p.x_uT100      = static_cast<int16_t>(mag.x_uT * 100.0f);
    p.y_uT100      = static_cast<int16_t>(mag.y_uT * 100.0f);
    p.z_uT100      = static_cast<int16_t>(mag.z_uT * 100.0f);
    p.heading_deg10 = static_cast<int16_t>(mag.heading_deg * 10.0f);
    sendFrame(MSG_SENSOR_MAG,
              reinterpret_cast<const uint8_t*>(&p),
              sizeof(p));
}

void Comms::sendThermalData(const ThermalData& thermal) {
    ThermalPayload p;
    for (int i = 0; i < 32 * 24; i++) {
        p.pixels[i] = static_cast<int16_t>(thermal.pixels[i] * 10.0f);
    }
    p.ambient_C10 = static_cast<int16_t>(thermal.ambient_temp_C * 10.0f);
    sendFrame(MSG_SENSOR_THERMAL,
              reinterpret_cast<const uint8_t*>(&p),
              sizeof(p));
}

void Comms::sendGasData(const GasData& gas) {
    GasPayload p;
    p.rs_ro_100 = static_cast<int16_t>(gas.rs_ro_ratio * 100.0f);
    p.ppm_lpg   = static_cast<int16_t>(gas.ppm_lpg);
    p.ppm_co    = static_cast<int16_t>(gas.ppm_co);
    p.ppm_smoke = static_cast<int16_t>(gas.ppm_smoke);
    sendFrame(MSG_SENSOR_GAS,
              reinterpret_cast<const uint8_t*>(&p),
              sizeof(p));
}

void Comms::sendStatus(const SystemStatus& s) {
    uint8_t buf[4];
    buf[0] = static_cast<uint8_t>(s.mode);
    buf[1] = (s.ppm_connected   ? 0x01 : 0)
           | (s.minipc_connected ? 0x02 : 0)
           | (s.can_ok           ? 0x04 : 0)
           | (s.estop            ? 0x08 : 0);
    buf[2] = s.sensor_mask;
    buf[3] = 0;   // padding
    sendFrame(MSG_STATUS, buf, sizeof(buf));
}

bool Comms::isConnected() {
    return (millis() - s_last_rx_ms) < 1000;
}
