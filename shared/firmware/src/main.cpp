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
#include "Debug.h"

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
#ifdef ENABLE_COMMS
        // Drive the UART RX parser and flush TX
        Comms::tick();
#endif

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
            RC::peekFrame(ppm);  // read without consuming the fresh flag (control task owns it)
            for (int i = 0; i < PPM_CHANNELS; i++) telem.ppm[i] = ppm.ch[i];

            telem.speed_left    = static_cast<int16_t>(enc.speed_left_rpm  * 10.0f);
            telem.speed_right   = static_cast<int16_t>(enc.speed_right_rpm * 10.0f);
            telem.flipper_angle = static_cast<int16_t>(enc.flipper_angle_deg * 10.0f);
            telem.uptime_ms     = status.uptime_ms;

#ifdef ENABLE_COMMS
            Comms::sendTelemetry(telem);

  #ifdef ROBOT_SECONDARY
            // Send independent flipper angles at the same 50 Hz rate
            Comms::sendEncoderExt(enc);
  #endif

  #ifdef ROBOT_MAIN
            // Send commanded duty cycles for both track motors and the flipper
            MainMotorPayload motors;
            motors.duty_left_1000    = static_cast<int16_t>(Locomotion::getTrackLeft()     * 1000.0f);
            motors.duty_right_1000   = static_cast<int16_t>(Locomotion::getTrackRight()    * 1000.0f);
            motors.duty_flipper_1000 = static_cast<int16_t>(Locomotion::getFlipperEffort() * 1000.0f);
            Comms::sendMainMotorStatus(motors);
  #endif
#endif
        }

        // Yield briefly so other Core 0 tasks get time
        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 0 — CAN task
//  Polls the TWAI driver for incoming frames and forwards VESC feedback.
// ─────────────────────────────────────────────────────────────────────────────
static void canTask(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(5);   // 200 Hz poll
    TickType_t       last   = xTaskGetTickCount();

    for (;;) {
        CANInterface::poll();

        // Forward any fresh ODrive arm feedback to mini PC (both robots)
        for (uint8_t j = 0; j < ODRIVE_MAX_JOINTS; j++) {
            OdriveStatusPayload o;
            if (CANInterface::getOdriveStatus(j, o)) {
                Comms::sendOdriveStatus(o);
            }
        }

#if ODRIVE_ENABLE_ERROR_POLL
        // Forward ODrive error snapshots (arm + traction, all variants)
        for (uint8_t n = 0; n < CANInterface::odriveNodeCount(); n++) {
            OdriveErrorPayload e;
            if (CANInterface::getOdriveError(n, e)) {
                Comms::sendOdriveError(e);
            }
        }
#endif

#ifdef ROBOT_SECONDARY
        // Forward any fresh traction ODrive feedback
        for (uint8_t m = 0; m < TRACTION_ODRIVE_NUM; m++) {
            OdriveStatusPayload o;
            if (CANInterface::getTractionStatus(m, o)) {
                Comms::sendOdriveStatus(o);
            }
        }

        // Forward any fresh VESC flipper feedback to mini PC
        for (uint8_t id = 1; id <= VESC_MAX_CONTROLLERS; id++) {
            VescStatusPayload v;
            if (CANInterface::getVescStatus(id, v)) {
                Comms::sendVescStatus(v);
            }
        }

        // Forward any fresh LKTech feedback (J5, J6)
        for (uint8_t j = 0; j < LKTECH_MAX_JOINTS; j++) {
            LktechStatusPayload l;
            if (CANInterface::getLktechStatus(j, l)) {
                Comms::sendLktechStatus(l);
            }
        }

        // Forward any fresh ZE300 (J4) feedback
        {
            Ze300StatusPayload z;
            if (CANInterface::getZe300Status(z)) {
                Comms::sendZe300Status(z);
            }
        }
#endif

        vTaskDelayUntil(&last, period);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 1 — Fast sensor task  (mag / gas / imu)
//  Rate-limited internally by Sensors::runOnce(); never blocks.
// ─────────────────────────────────────────────────────────────────────────────
static void sensorTask(void* /*arg*/) {
    for (;;) {
        uint8_t fresh = Sensors::runOnce();   // only reads sensors whose interval elapsed

        if (fresh & SENSOR_BIT_MAG) {
            MagData mag;
            Sensors::getMag(mag);
            if (mag.valid) Comms::sendMagData(mag);
        }

        if (fresh & SENSOR_BIT_GAS) {
            GasData gas;
            Sensors::getGas(gas);
            if (gas.valid) Comms::sendGasData(gas);
        }

        if (fresh & SENSOR_BIT_IMU) {
            ImuData imu;
            Sensors::getImu(imu);
            if (imu.valid) Comms::sendImuData(imu);
        }


        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core 1 — Thermal task  (lowest priority, blocks on MLX90640 getFrame)
//  Runs independently so the ~250 ms blocking read doesn't stall fast sensors.
// ─────────────────────────────────────────────────────────────────────────────
static void thermalTask(void* /*arg*/) {
    for (;;) {
        Sensors::runThermalOnce();   // blocks until frame ready (~250 ms @ 4 Hz)

        if (Sensors::getEnabledMask() & SENSOR_BIT_THERMAL) {
            ThermalData thermal;
            Sensors::getThermal(thermal);
            if (thermal.valid) Comms::sendThermalData(thermal);
        }

        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup() — hardware init + task creation
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    // Initialise all subsystems
    Comms::begin();
    Debug::begin(Dbg::ALL);   // enable all debug topics (no-op when ENABLE_COMMS defined)

    RC::begin(PIN_PPM);
    Encoders::begin();
    Locomotion::begin();
    Sensors::begin();
    CANInterface::begin();   // non-fatal if CAN module is absent at startup

    // Must be last: registers callbacks into Comms
    Control::begin();

    // ── Core 0: protocol tasks ────────────────────────────────────────────────
    xTaskCreatePinnedToCore(commsTask, "Comms",   STACK_COMMS,   nullptr,
                            PRIO_COMMS,   nullptr, TASK_CORE_COMMS);
                            
    xTaskCreatePinnedToCore(canTask,   "CAN",     STACK_CAN,     nullptr,
                            PRIO_CAN,     nullptr, TASK_CORE_CAN);

    // ── Core 1: control + sensor tasks ───────────────────────────────────────
    xTaskCreatePinnedToCore(controlTask,  "Control",  STACK_CONTROL, nullptr,
                            PRIO_CONTROL, nullptr, TASK_CORE_CONTROL);
                            
    xTaskCreatePinnedToCore(sensorTask,   "Sensors",  STACK_SENSORS, nullptr,
                            PRIO_SENSORS, nullptr, TASK_CORE_SENSORS);
    xTaskCreatePinnedToCore(thermalTask,  "Thermal",  STACK_THERMAL, nullptr,
                            PRIO_THERMAL, nullptr, TASK_CORE_THERMAL);
                            
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop() — not used; all work is done in pinned RTOS tasks
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    vTaskDelay(portMAX_DELAY);
}