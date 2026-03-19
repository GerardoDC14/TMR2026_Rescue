#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MLX90640.h>
#include <math.h>

// ─── Static member definitions ───────────────────────────────────────────────
uint8_t      Sensors::s_mask    = 0;
MagData      Sensors::s_mag     = {};
ThermalData  Sensors::s_thermal = {};
GasData      Sensors::s_gas     = {};
bool         Sensors::s_lis_ok  = false;
bool         Sensors::s_mlx_ok  = false;

// File-level mutex: keeps FreeRTOS types out of the class header
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_LIS3MDL  s_lis;
static Adafruit_MLX90640 s_mlx;

// Thermal pixel buffer for Adafruit_MLX90640::getFrame()
static float s_mlx_pixels[32 * 24];

// ─── Initialisation ───────────────────────────────────────────────────────────
bool Sensors::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);   // 400 kHz fast-mode

    // LIS3MDL ─────────────────────────────────────────────────────────────────
    if (s_lis.begin_I2C()) {
        s_lis.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        s_lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        s_lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
        s_lis.setRange(LIS3MDL_RANGE_4_GAUSS);
        s_lis_ok = true;
    }

    // MLX90640 ────────────────────────────────────────────────────────────────
    // begin() reads EEPROM and extracts calibration internally
    if (s_mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
        s_mlx.setMode(MLX90640_CHESS);           // chess-board sub-frame pattern
        s_mlx.setResolution(MLX90640_ADC_18BIT);
        s_mlx.setRefreshRate(MLX90640_4_HZ);     // 4 Hz; adjust via MLX90640_REFRESH_HZ
        s_mlx_ok = true;
    }

    return true;   // non-fatal: caller can check s_lis_ok / s_mlx_ok separately
}

// ─── Sensor task entry point ──────────────────────────────────────────────────
void Sensors::runOnce() {
    if (s_mask & SENSOR_BIT_MAG)     readMag();
    if (s_mask & SENSOR_BIT_THERMAL) readThermal();
    if (s_mask & SENSOR_BIT_GAS)     readGas();
}

// ─── Mask control ─────────────────────────────────────────────────────────────
void Sensors::setEnabledMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_mask = mask;
    portEXIT_CRITICAL(&s_mux);
}

uint8_t Sensors::getEnabledMask() {
    portENTER_CRITICAL(&s_mux);
    uint8_t m = s_mask;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

// ─── Thread-safe accessors ────────────────────────────────────────────────────
void Sensors::getMag(MagData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_mag;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::getThermal(ThermalData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_thermal;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::getGas(GasData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_gas;
    portEXIT_CRITICAL(&s_mux);
}

// ─── Private sensor reads ─────────────────────────────────────────────────────
void Sensors::readMag() {
    if (!s_lis_ok) return;
    // LIS3MDL is in continuous mode at 155 Hz; just read the latest sample
    sensors_event_t event;
    s_lis.getEvent(&event);

    MagData d;
    d.x_uT       = event.magnetic.x;
    d.y_uT       = event.magnetic.y;
    d.z_uT       = event.magnetic.z;
    d.heading_deg = computeHeading(d.x_uT, d.y_uT);
    d.valid       = true;

    portENTER_CRITICAL(&s_mux);
    s_mag = d;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readThermal() {
    if (!s_mlx_ok) return;

    // getFrame() handles the two-sub-page protocol and calibration internally.
    // It blocks until a complete frame is ready (up to 1/refresh_rate seconds).
    if (s_mlx.getFrame(s_mlx_pixels) != 0) return;

    ThermalData d;
    d.ambient_temp_C = 0.0f;   // Adafruit API does not expose ambient separately
    float max_t = -300.0f;
    for (int i = 0; i < 32 * 24; i++) {
        d.pixels[i] = s_mlx_pixels[i];
        if (s_mlx_pixels[i] > max_t) max_t = s_mlx_pixels[i];
    }
    d.max_temp_C = max_t;
    d.valid      = true;

    portENTER_CRITICAL(&s_mux);
    s_thermal = d;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readGas() {
    // Average MQ2_SAMPLE_COUNT ADC readings to reduce noise
    uint32_t sum = 0;
    for (int i = 0; i < MQ2_SAMPLE_COUNT; i++) {
        sum += analogRead(PIN_MQ2);
        delayMicroseconds(200);
    }
    float adc_avg = sum / static_cast<float>(MQ2_SAMPLE_COUNT);

    // ESP32 ADC: 12-bit (0–4095), 3.3 V reference
    float v_out = (adc_avg / 4095.0f) * 3.3f;
    if (v_out < 0.001f) v_out = 0.001f;   // avoid division by zero

    // Sensor resistance: Rs = (Vcc/Vout - 1) × RL
    float rs = ((3.3f / v_out) - 1.0f) * MQ2_RL_KOHM;
    float ratio = rs / MQ2_RO_KOHM;

    GasData d;
    d.rs_ro_ratio = ratio;
    // Empirical curves from MQ2 datasheet (log-log linear approximation)
    d.ppm_lpg   = mq2Curve(ratio, 44.947f, -3.445f);
    d.ppm_co    = mq2Curve(ratio, 18.446f, -2.800f);
    d.ppm_smoke = mq2Curve(ratio, 22.098f, -2.676f);
    d.valid      = true;

    portENTER_CRITICAL(&s_mux);
    s_gas = d;
    portEXIT_CRITICAL(&s_mux);
}

// ─── Helpers ─────────────────────────────────────────────────────────────────
float Sensors::computeHeading(float x, float y) {
    float h = atan2f(y, x) * 180.0f / M_PI;
    if (h < 0.0f) h += 360.0f;
    return h;
}

// Log-log linear curve: PPM = a × (Rs/Ro)^b
float Sensors::mq2Curve(float ratio, float a, float b) {
    return a * powf(ratio, b);
}
