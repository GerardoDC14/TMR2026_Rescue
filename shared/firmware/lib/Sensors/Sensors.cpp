#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_BNO055.h>

// ─── Static member definitions ───────────────────────────────────────────────
uint8_t      Sensors::s_mask    = 0;
MagData      Sensors::s_mag     = {};
ThermalData  Sensors::s_thermal = {};
GasData      Sensors::s_gas     = {};
ImuData      Sensors::s_imu     = {};
bool         Sensors::s_lis_ok  = false;
bool         Sensors::s_mlx_ok  = false;
bool         Sensors::s_bno_ok  = false;

// File-level mutex: keeps FreeRTOS types out of the class header
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static Adafruit_LIS3MDL  s_lis;
static Adafruit_MLX90640 s_mlx;
static Adafruit_BNO055   s_bno(55, BNO055_I2C_ADDR);

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
    if (s_mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
        s_mlx.setMode(MLX90640_CHESS);
        s_mlx.setResolution(MLX90640_ADC_18BIT);
        s_mlx.setRefreshRate(MLX90640_4_HZ);
        s_mlx_ok = true;
    }

    // BNO055 ──────────────────────────────────────────────────────────────────
    // NDOF fusion mode: uses accel + gyro + mag for full orientation.
    if (s_bno.begin()) {
        s_bno.setExtCrystalUse(true);   // use external 32.768 kHz crystal for accuracy
        s_bno_ok = true;
    }

    return true;   // non-fatal: callers can check s_*_ok separately
}

// ─── Sensor task entry point ──────────────────────────────────────────────────
void Sensors::runOnce() {
    if (s_mask & SENSOR_BIT_MAG)     readMag();
    if (s_mask & SENSOR_BIT_THERMAL) readThermal();
    if (s_mask & SENSOR_BIT_GAS)     readGas();
    if (s_mask & SENSOR_BIT_IMU)     readImu();
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

void Sensors::getImu(ImuData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_imu;
    portEXIT_CRITICAL(&s_mux);
}

// ─── Private sensor reads ─────────────────────────────────────────────────────
void Sensors::readMag() {
    if (!s_lis_ok) return;
    // LIS3MDL is in continuous mode at 155 Hz; just read the latest sample
    sensors_event_t event;
    s_lis.getEvent(&event);

    MagData d;
    d.x_uT  = event.magnetic.x;
    d.y_uT  = event.magnetic.y;
    d.z_uT  = event.magnetic.z;
    d.valid = true;

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
    for (int i = 0; i < 32 * 24; i++) {
        d.pixels[i] = s_mlx_pixels[i];
    }
    d.valid = true;

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
    d.valid       = true;

    portENTER_CRITICAL(&s_mux);
    s_gas = d;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readImu() {
    if (!s_bno_ok) return;

    imu::Vector<3> euler  = s_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> laccel = s_bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro   = s_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
    s_bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    ImuData d;
    // BNO055 Euler: x=heading/yaw, y=roll, z=pitch in degrees
    d.yaw_deg   = euler.x();
    d.pitch_deg = euler.z();
    d.roll_deg  = euler.y();
    d.accel_x = laccel.x(); d.accel_y = laccel.y(); d.accel_z = laccel.z();
    d.gyro_x  = gyro.x();   d.gyro_y  = gyro.y();   d.gyro_z  = gyro.z();
    d.calib = (uint8_t)((cal_sys << 6) | (cal_gyro << 4) | (cal_accel << 2) | cal_mag);
    d.valid = true;

    portENTER_CRITICAL(&s_mux);
    s_imu = d;
    portEXIT_CRITICAL(&s_mux);
}

