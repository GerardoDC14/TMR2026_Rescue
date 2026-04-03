#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_BNO055.h>

// ─── Static member definitions ───────────────────────────────────────────────
uint8_t      Sensors::s_mask    = 0;
MagData      Sensors::s_mag     = {};
ThermalData  Sensors::s_thermal = {};
GasData      Sensors::s_gas     = {};
ImuData      Sensors::s_imu     = {};
bool         Sensors::s_mag_ok  = false;
bool         Sensors::s_mlx_ok  = false;
bool         Sensors::s_bno_ok  = false;

// File-level spinlock for data fields (short critical sections only)
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// I2C bus mutex — serializes access across sensorTask and thermalTask.
// Must be a FreeRTOS mutex (not portMUX) because I2C transactions block.
static SemaphoreHandle_t s_i2c_mutex = nullptr;

static QMC5883LCompass   s_qmc;
static Adafruit_MLX90640 s_mlx;
static Adafruit_BNO055   s_bno(55, BNO055_I2C_ADDR);

// Thermal pixel buffer for Adafruit_MLX90640::getFrame()
static float s_mlx_pixels[32 * 24];

// ─── Initialisation ───────────────────────────────────────────────────────────
bool Sensors::begin() {
    s_i2c_mutex = xSemaphoreCreateMutex();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);   // 400 kHz fast-mode

    // QMC5883L (GY-271) ──────────────────────────────────────────────────────
    Wire.beginTransmission(0x0D);
    if (Wire.endTransmission() == 0) {
        s_qmc.init();
        //s_qmc.setMode(0x01, 0x04, 0x10, 0x00);  // continuous, 50 Hz ODR, 8G range, 512 OSR
        s_mag_ok = true;
    }

    // MLX90640 ────────────────────────────────────────────────────────────────
    if (s_mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
        s_mlx.setMode(MLX90640_CHESS);
        s_mlx.setResolution(MLX90640_ADC_18BIT);
        s_mlx.setRefreshRate(MLX90640_8_HZ);
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

// ─── Per-sensor rate tracking ────────────────────────────────────────────────
static uint32_t s_last_mag_ms  = 0;
static uint32_t s_last_imu_ms  = 0;
static uint32_t s_last_gas_ms  = 0;

// ─── Fast sensor task entry point (mag / gas / imu) ──────────────────────────
uint8_t Sensors::runOnce() {
    uint32_t now = millis();
    uint8_t read = 0;

    if ((s_mask & SENSOR_BIT_MAG) &&
        (now - s_last_mag_ms >= 1000 / SENSOR_MAG_HZ)) {
        s_last_mag_ms = now;
        readMag();
        read |= SENSOR_BIT_MAG;
    }

    if ((s_mask & SENSOR_BIT_GAS) &&
        (now - s_last_gas_ms >= 1000 / SENSOR_GAS_HZ)) {
        s_last_gas_ms = now;
        readGas();
        read |= SENSOR_BIT_GAS;
    }

    if ((s_mask & SENSOR_BIT_IMU) &&
        (now - s_last_imu_ms >= 1000 / SENSOR_IMU_HZ)) {
        s_last_imu_ms = now;
        readImu();
        read |= SENSOR_BIT_IMU;
    }

    return read;
}

// ─── Thermal task entry point (separate task, blocks on getFrame) ────────────
void Sensors::runThermalOnce() {
    if (s_mask & SENSOR_BIT_THERMAL) readThermal();
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
    if (!s_mag_ok) return;
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    s_qmc.read(); 
    
    int x_raw = s_qmc.getX();
    int y_raw = s_qmc.getY();
    int z_raw = s_qmc.getZ();

    xSemaphoreGive(s_i2c_mutex);

    //constexpr float SCALE_UT = 100.0f / 3000.0f;
    constexpr float SCALE_UT = 1;

    constexpr float offset_x_uT = 0.0f; 
    constexpr float offset_y_uT = 0.0f;
    constexpr float offset_z_uT = 0.0f;

    MagData d;
    
    if (x_raw == 0 && y_raw == 0 && z_raw == 0) {
        d.valid = true; 
        d.x_uT = 0.0f; d.y_uT = 0.0f; d.z_uT = 0.0f; 
    }   
    else {
        d.x_uT  = (x_raw * SCALE_UT) - offset_x_uT;
        d.y_uT  = (y_raw * SCALE_UT) - offset_y_uT;
        d.z_uT  = (z_raw * SCALE_UT) - offset_z_uT;
        d.valid = true;
    }
    
    portENTER_CRITICAL(&s_mux);
    s_mag = d;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readThermal() {
    if (!s_mlx_ok) return;
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return;

    // getFrame() handles the two-sub-page protocol and calibration internally.
    // It blocks until a complete frame is ready (up to 1/refresh_rate seconds).
    int rc = s_mlx.getFrame(s_mlx_pixels);

    xSemaphoreGive(s_i2c_mutex);

    if (rc != 0) return;

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
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    imu::Quaternion quat = s_bno.getQuat();
    imu::Vector<3> laccel = s_bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro   = s_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
    s_bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    xSemaphoreGive(s_i2c_mutex);

    imu::Vector<3> euler = quat.toEuler(); 

    ImuData d;
    d.yaw_deg   = euler.x() * (180.0 / M_PI);
    d.pitch_deg = euler.y() * (180.0 / M_PI);
    d.roll_deg  = euler.z() * (180.0 / M_PI);

    d.accel_x = laccel.x(); 
    d.accel_y = laccel.y(); 
    d.accel_z = laccel.z();
    
    d.gyro_x  = gyro.x();   
    d.gyro_y  = gyro.y();   
    d.gyro_z  = gyro.z();
    
    d.calib = (uint8_t)((cal_sys << 6) | (cal_gyro << 4) | (cal_accel << 2) | cal_mag);
    d.valid = true;

    portENTER_CRITICAL(&s_mux);
    s_imu = d;
    portEXIT_CRITICAL(&s_mux);
}
