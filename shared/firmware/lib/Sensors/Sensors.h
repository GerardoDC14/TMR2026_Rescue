#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Sensors ──────────────────────────────────────────────────────────────────
// Manages LIS3MDL (magnetometer), MLX90640 (thermal camera), and MQ2 (gas).
//
// All sensors remain in their lowest-power / idle state until enabled via
// setEnabledMask().  Once a sensor bit is set, its reading loop runs inside
// the dedicated sensor task (Core 1, low priority) and never blocks the
// control loop.
//
// Call begin() once from setup, then runOnce() repeatedly from the sensor task.
// Retrieve the latest data with getMag() / getThermal() / getGas().

class Sensors {
public:
    static bool begin();          // returns false if any mandatory init fails

    // Called from the sensor FreeRTOS task.  Non-blocking: each invocation
    // advances state machines for whichever sensors are active.
    static void runOnce();

    // Enable/disable sensors at runtime (called by Comms on MSG_SENSOR_ENABLE).
    static void setEnabledMask(uint8_t mask);
    static uint8_t getEnabledMask();

    // Thread-safe data accessors
    static void getMag(MagData& out);
    static void getThermal(ThermalData& out);
    static void getGas(GasData& out);

private:
    static void readMag();
    static void readThermal();
    static void readGas();

    // Heading from raw magnetometer XYZ
    static float computeHeading(float x, float y);

    // MQ2 resistance → estimated gas concentration
    static float mq2Curve(float ratio, float a, float b);

    static uint8_t        s_mask;
    // s_mux lives as a file-level static in Sensors.cpp (avoids FreeRTOS header in .h)
    static MagData        s_mag;
    static ThermalData    s_thermal;
    static GasData        s_gas;
    static bool           s_lis_ok;
    static bool           s_mlx_ok;
};
