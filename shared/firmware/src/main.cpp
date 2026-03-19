#include <Arduino.h>
#include "config.h"
#include "robot_types.h"

#include "RC.h"
#include "Encoders.h"
#include "Locomotion.h"
#include "Sensors.h"
#include "CANInterface.h"
#include "Comms.h"
#include "Control.h"

// ─── Task handles ─────────────────────────────────────────────────────────────
static TaskHandle_t h_control = nullptr;
static TaskHandle_t h_comms   = nullptr;
static TaskHandle_t h_can     = nullptr;
static TaskHandle_t h_sensors = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
//  Core 1 — Control task
//  Runs at PRIO_CONTROL.  Reads PPM + encoders and drives the state machine.
// ─────────────────────────────────────────────────────────────────────────────
static void controlTask(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);
    TickType_t       last   = xTaskGetTickCount();

    for (;;) {
        // Update encoder-derived values (speed + flipper angle)
        Encoders::updateDerivedValues();

        // Run the high-level control state machine
        Control::tick();

        // Strict period — vTaskDelayUntil guarantees no drift
        vTaskDelayUntil(&last, period);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 0 — Comms task
//  Handles UART ↔ mini PC and sends periodic telemetry.
// ─────────────────────────────────────────────────────────────────────────────
static void commsTask(void* /*arg*/) {
    const TickType_t telem_period = pdMS_TO_TICKS(20);   // 50 Hz telemetry
    TickType_t       last         = xTaskGetTickCount();

    for (;;) {
        // Drive the UART RX parser and flush TX
        Comms::tick();

        // Build and send telemetry at 50 Hz
        if (xTaskGetTickCount() - last >= telem_period) {
            last = xTaskGetTickCount();

            EncoderState enc;
            Encoders::getState(enc);

            SystemStatus status;
            Control::getSystemStatus(status);

            TelemetryPayload telem;
            telem.mode  = static_cast<uint8_t>(status.mode);
            telem.flags = (status.ppm_connected    ? 0x01 : 0)
                        | (status.sensor_mask       ? 0x02 : 0)
                        | (status.can_ok            ? 0x04 : 0)
                        | (status.estop             ? 0x08 : 0);

            PPMFrame ppm;
            RC::getFrame(ppm);   // returns last valid frame
            for (int i = 0; i < PPM_CHANNELS; i++) telem.ppm[i] = ppm.ch[i];

            telem.speed_left    = static_cast<int16_t>(enc.speed_left_rpm  * 10.0f);
            telem.speed_right   = static_cast<int16_t>(enc.speed_right_rpm * 10.0f);
            telem.flipper_angle = static_cast<int16_t>(enc.flipper_angle_deg * 10.0f);
            telem.uptime_ms     = status.uptime_ms;

            Comms::sendTelemetry(telem);
        }

        // Yield briefly so other Core 0 tasks get time
        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 0 — CAN task
//  Polls the TWAI driver for incoming frames (arm status, faults).
// ─────────────────────────────────────────────────────────────────────────────
static void canTask(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(5);   // 200 Hz poll
    TickType_t       last   = xTaskGetTickCount();

    for (;;) {
        CANInterface::poll();
        vTaskDelayUntil(&last, period);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 1 — Sensor task  (low priority, background)
//  Runs sensor state machines and sends data to mini PC when new data arrives.
// ─────────────────────────────────────────────────────────────────────────────
static void sensorTask(void* /*arg*/) {
    for (;;) {
        Sensors::runOnce();

        uint8_t mask = Sensors::getEnabledMask();

        if (mask & SENSOR_BIT_MAG) {
            MagData mag;
            Sensors::getMag(mag);
            if (mag.valid) Comms::sendMagData(mag);
        }

        if (mask & SENSOR_BIT_THERMAL) {
            ThermalData thermal;
            Sensors::getThermal(thermal);
            if (thermal.valid) Comms::sendThermalData(thermal);
        }

        if (mask & SENSOR_BIT_GAS) {
            GasData gas;
            Sensors::getGas(gas);
            if (gas.valid) Comms::sendGasData(gas);
        }

        // Yield for at least one tick between sensor sweeps
        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup() — hardware init + task creation
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    // Initialise all subsystems
    Comms::begin();

    RC::begin(PIN_PPM);
    Encoders::begin();
    Locomotion::begin();
    Sensors::begin();
    CANInterface::begin();   // non-fatal if CAN module is absent at startup

    // Must be last: registers callbacks into Comms
    Control::begin();

    //Serial.println("whatup bitches");

    // ── Core 0: protocol tasks ────────────────────────────────────────────────
    xTaskCreatePinnedToCore(commsTask, "Comms",   STACK_COMMS,   nullptr,
                            PRIO_COMMS,   &h_comms, TASK_CORE_COMMS);

    xTaskCreatePinnedToCore(canTask,   "CAN",     STACK_CAN,     nullptr,
                            PRIO_CAN,     &h_can,   TASK_CORE_CAN);

    // ── Core 1: control + sensor tasks ───────────────────────────────────────
    xTaskCreatePinnedToCore(controlTask, "Control", STACK_CONTROL, nullptr,
                            PRIO_CONTROL, &h_control, TASK_CORE_CONTROL);

    xTaskCreatePinnedToCore(sensorTask,  "Sensors", STACK_SENSORS, nullptr,
                            PRIO_SENSORS, &h_sensors, TASK_CORE_SENSORS);
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop() — not used; all work is done in pinned RTOS tasks
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    vTaskDelay(portMAX_DELAY);
}