/*
 * firmware_dicerox — ESP32 main application
 *
 * Integrates:
 *   - I2C sensors (AMG8833 thermal, LIS3MDL mag, MQ-2 gas)
 *   - 6-channel RC PPM input
 *   - Tank-drive + flipper control via CAN (VESCs + ODrives through MCP2515)
 *   - Binary UART framing compatible with esp32_bridge.py
 *
 * All Serial I/O goes through the binary protocol; no text telemetry.
 *
 * Serial protocol: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD][CRC:1]
 *   CRC = XOR of TYPE ^ LEN_H ^ LEN_L ^ payload bytes.
 *
 * Sensor enable mask (PC → ESP32, MSG_SENSOR_ENABLE): bit0=mag, bit1=thermal,
 * bit2=gas. Starts at 0x00 (all off).
 */

#include <Arduino.h>
#include <Wire.h>
/*
#include <SPI.h>
#include <mcp2515.h>
#include <ODrive.hpp>
*/
#include <Adafruit_MLX90640.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

// ─── Pin map ────────────────────────────────────────────────────────────────
#define PIN_MQ2        12
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22

/*
#define PIN_PPM         4
#define PIN_CAN_CS      5
#define PIN_CAN_SCK    18
#define PIN_CAN_MISO   19
#define PIN_CAN_MOSI   23

// ─── PPM config ─────────────────────────────────────────────────────────────
static const uint8_t  PPM_CHANNELS      = 6;
static const uint32_t PPM_SYNC_US       = 2100;
static const int      PPM_MIN_US        = 1000;
static const int      PPM_MAX_US        = 2000;
static const int      PPM_DEAD_LOW      = 1480;
static const int      PPM_DEAD_HIGH     = 1520;

static const uint8_t CH_STEERING        = 0;
static const uint8_t CH_THROTTLE        = 1;
static const uint8_t CH_FLIPPER_MOVE    = 2;
static const uint8_t CH_FLIPPER_SELECT  = 3;
static const uint8_t CH_FLIPPER_PAIR    = 4;

static const int FLIP_SEL_CENTER_LOW    = 1450;
static const int FLIP_SEL_CENTER_HIGH   = 1550;
static const int FLIP_PAIR_FRONT_MAX    = 1450;
static const int FLIP_PAIR_REAR_MIN     = 1550;

// ─── Motor IDs ──────────────────────────────────────────────────────────────
static const uint8_t VESC_LEFT_ID         = 10;
static const uint8_t VESC_RIGHT_ID        = 20;
static const uint8_t VESC_FLIP_FL_ID      = 30;
static const uint8_t VESC_FLIP_FR_ID      = 40;
static const uint8_t ODRIVE_REAR_RIGHT_ID = 50;
static const uint8_t ODRIVE_REAR_LEFT_ID  = 60;
static const uint8_t VESC_CMD_SET_RPM     = 3;

// ─── Control limits / rates ─────────────────────────────────────────────────
static const float    RPM_MAX                = 14000.0f;
static const float    FLIPPER_FRONT_RPM_MAX  = 10500.0f;
static const float    FLIPPER_REAR_RPM_MAX   =  1500.0f;
static const float    TRACTION_ACCEL         =  6000.0f;
static const float    FRONT_FLIP_ACCEL       =  2000.0f;
static const float    REAR_FLIP_ACCEL        =  1000.0f;
static const uint32_t CAN_PERIOD_MS          = 20;    // 50 Hz control
static const uint32_t FAILSAFE_MS            = 300;

#define INVERT_LEFT      false
#define INVERT_RIGHT     false
#define INVERT_FLIP_FL   false
#define INVERT_FLIP_FR   false
#define INVERT_FLIP_RL   false
#define INVERT_FLIP_RR   false

*/

// ─── MQ-2 ADC thresholds ────────────────────────────────────────────────────
static const int   MQ2_CLEAN_ADC = 3750;
static const int   MQ2_MAX_ADC   = 4095;
static const float MQ2_MAX_PPM   = 2000.0f;

// ─── Binary protocol IDs ────────────────────────────────────────────────────
static const uint8_t MSG_TELEMETRY      = 0x01;
static const uint8_t MSG_THERMAL        = 0x02;
static const uint8_t MSG_MAG            = 0x03;
static const uint8_t MSG_GAS            = 0x04;
static const uint8_t MSG_SENSOR_ENABLE  = 0x11;
static const uint8_t MSG_ESTOP          = 0x12;
static const uint8_t MSG_ESTOP_CLEAR    = 0x13;

static const uint8_t BIT_MAG     = (1 << 0);
static const uint8_t BIT_THERMAL = (1 << 1);
static const uint8_t BIT_GAS     = (1 << 2);

/*
// ─── Hardware objects ───────────────────────────────────────────────────────
MCP2515          mcp2515(PIN_CAN_CS);
ODrive           rearLeft (ODRIVE_REAR_LEFT_ID,  &mcp2515);
ODrive           rearRight(ODRIVE_REAR_RIGHT_ID, &mcp2515);
*/
Adafruit_MLX90640 mlx;
Adafruit_LIS3MDL lis3mdl;

/*
// ─── PPM ISR state ──────────────────────────────────────────────────────────
volatile uint32_t ppmValues[PPM_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint8_t  ppmCh            = 0;
volatile uint32_t ppmLastEdgeUs    = 0;
volatile uint32_t ppmLastFrameUs   = 0;
volatile bool     ppmFrameReady    = false;
*/

// ─── Sensor state ───────────────────────────────────────────────────────────
static bool    has_mlx = false, has_lis3mdl = false;
static float   mlx_pixels[32 * 24] = {};
static float   mq2_ppm = 0.0f;
static float   mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
static uint8_t sensor_enable = 0x00;

/*
// ─── Control state ──────────────────────────────────────────────────────────
static bool  armed      = false;
static bool  inFailsafe = true;
static bool  softEstop  = false;
static float rpmLeftCmd = 0.0f,  rpmRightCmd = 0.0f;
static float rpmFlFlCmd = 0.0f,  rpmFlFrCmd  = 0.0f;   // front flippers (VESC)
static float rpmFlRlCmd = 0.0f,  rpmFlRrCmd  = 0.0f;   // rear flippers (ODrive)
*/

// ────────────────────────────────────────────────────────────────────────────
// Binary frame TX / RX
// ────────────────────────────────────────────────────────────────────────────
static void sendFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
    uint8_t lh  = (len >> 8) & 0xFF;
    uint8_t ll  = len & 0xFF;
    uint8_t crc = type ^ lh ^ ll;
    for (uint16_t i = 0; i < len; i++) crc ^= payload[i];
    uint8_t hdr[5] = { 0xAA, 0x55, type, lh, ll };
    Serial.write(hdr, 5);
    if (len) Serial.write(payload, len);
    Serial.write(crc);
}

static uint8_t  rxSt = 0, rxType = 0, rxCrc = 0;
static uint16_t rxLen = 0, rxCnt = 0;
static uint8_t  rxBuf[32];

static void handleIncoming(uint8_t type, const uint8_t* buf, uint16_t len) {
    if (type == MSG_SENSOR_ENABLE && len >= 1)  sensor_enable = buf[0];
    /*
    else if (type == MSG_ESTOP)                 softEstop = true;
    else if (type == MSG_ESTOP_CLEAR)           softEstop = false;
    */
}

static void processRxByte(uint8_t b) {
    switch (rxSt) {
        case 0: if (b == 0xAA) rxSt = 1; break;
        case 1: rxSt = (b == 0x55) ? 2 : 0; break;
        case 2: rxType = b; rxCrc = b; rxSt = 3; break;
        case 3: rxLen = (uint16_t)b << 8; rxCrc ^= b; rxSt = 4; break;
        case 4:
            rxLen |= b; rxCrc ^= b; rxCnt = 0;
            if (rxLen == 0)                   rxSt = 6;
            else if (rxLen <= sizeof(rxBuf))  rxSt = 5;
            else                              rxSt = 0;
            break;
        case 5:
            rxBuf[rxCnt++] = b; rxCrc ^= b;
            if (rxCnt >= rxLen) rxSt = 6;
            break;
        case 6:
            if (b == rxCrc) handleIncoming(rxType, rxBuf, rxLen);
            rxSt = 0;
            break;
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PPM ISR + helpers
// ────────────────────────────────────────────────────────────────────────────
/*
void IRAM_ATTR ppmISR() {
    uint32_t now = micros();
    uint32_t w   = now - ppmLastEdgeUs;
    ppmLastEdgeUs = now;
    if (w > PPM_SYNC_US) {
        ppmCh = 0;
        ppmFrameReady = true;
        ppmLastFrameUs = now;
    } else if (ppmCh < PPM_CHANNELS) {
        ppmValues[ppmCh++] = w;
    }
}

static inline uint32_t readPpm(uint8_t ch) {
    noInterrupts();
    uint32_t v = ppmValues[ch];
    interrupts();
    return v;
}

static inline float ppmToNorm(int us) {
    if (us < PPM_MIN_US) us = PPM_MIN_US;
    if (us > PPM_MAX_US) us = PPM_MAX_US;
    if (us >= PPM_DEAD_LOW && us <= PPM_DEAD_HIGH) return 0.0f;
    if (us > PPM_DEAD_HIGH)
        return (float)(us - PPM_DEAD_HIGH) / (float)(PPM_MAX_US - PPM_DEAD_HIGH);
    return -(float)(PPM_DEAD_LOW - us) / (float)(PPM_DEAD_LOW - PPM_MIN_US);
}

// ────────────────────────────────────────────────────────────────────────────
// Motor CAN helpers
// ────────────────────────────────────────────────────────────────────────────
static inline uint32_t vescEid(uint8_t unit_id, uint8_t cmd) {
    return ((uint32_t)cmd << 8) | unit_id;
}

static void sendVescRPM(uint8_t unit_id, int32_t rpm) {
    can_frame f{};
    f.can_id  = vescEid(unit_id, VESC_CMD_SET_RPM) | CAN_EFF_FLAG;
    f.can_dlc = 4;
    f.data[0] = (rpm >> 24) & 0xFF;
    f.data[1] = (rpm >> 16) & 0xFF;
    f.data[2] = (rpm >>  8) & 0xFF;
    f.data[3] = (rpm >>  0) & 0xFF;
    mcp2515.sendMessage(&f);
}

// Bypass ODrive::setRPMs — library does integer division (int rpm / 60), which
// truncates fractional turns/s. Use setVelocity with a proper float conversion.
static inline void setOdriveRPM(ODrive& od, float rpm) {
    od.setVelocity(rpm / 60.0f, 0.0f);
}

static float rampTowards(float cur, float tgt, float accel, float dt) {
    float step = accel * dt;
    if (tgt > cur + step) return cur + step;
    if (tgt < cur - step) return cur - step;
    return tgt;
}
*/

// ────────────────────────────────────────────────────────────────────────────
// Sensor reads + publishers
// ────────────────────────────────────────────────────────────────────────────
static void readMQ2() {
    int raw = analogRead(PIN_MQ2);
    if (raw <= MQ2_CLEAN_ADC) { mq2_ppm = 0.0f; return; }
    mq2_ppm = ((float)(raw - MQ2_CLEAN_ADC) / (MQ2_MAX_ADC - MQ2_CLEAN_ADC)) * MQ2_MAX_PPM;
    mq2_ppm = constrain(mq2_ppm, 0.0f, MQ2_MAX_PPM);
}

static void readLIS3MDL() {
    if (!has_lis3mdl) return;
    sensors_event_t ev;
    lis3mdl.getEvent(&ev);
    mag_x = ev.magnetic.x;
    mag_y = ev.magnetic.y;
    mag_z = ev.magnetic.z;
}

static void readMLX90640() {
    if (!has_mlx) return;
    mlx.getFrame(mlx_pixels);
}

static void publishGas() {
    int16_t v = (int16_t)mq2_ppm;
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((uint16_t)v >> 8) };
    sendFrame(MSG_GAS, b, 2);
}

static void publishMag() {
    int16_t x = (int16_t)constrain(mag_x * 100.0f, -32768.0f, 32767.0f);
    int16_t y = (int16_t)constrain(mag_y * 100.0f, -32768.0f, 32767.0f);
    int16_t z = (int16_t)constrain(mag_z * 100.0f, -32768.0f, 32767.0f);
    uint8_t b[6] = {
        (uint8_t)(x & 0xFF), (uint8_t)((uint16_t)x >> 8),
        (uint8_t)(y & 0xFF), (uint8_t)((uint16_t)y >> 8),
        (uint8_t)(z & 0xFF), (uint8_t)((uint16_t)z >> 8),
    };
    sendFrame(MSG_MAG, b, 6);
}

static void publishThermal() {
    static const int N = 32 * 24;
    static uint8_t buf[N * 2];
    for (int i = 0; i < N; i++) {
        int16_t v = (int16_t)(mlx_pixels[i] * 10.0f);
        buf[i*2]   = (uint8_t)(v & 0xFF);
        buf[i*2+1] = (uint8_t)((uint16_t)v >> 8);
    }
    sendFrame(MSG_THERMAL, buf, sizeof(buf));
}

static void publishTelemetry() {
    // TelemetryPayload (24 B): u8 mode, u8 flags, u16[6] ppm,
    //   i16 spd_l×10, i16 spd_r×10, i16 flipper_angle×10, u32 uptime_ms
    // MODE_NAMES in the bridge: 0=INIT 1=STANDBY 2=NORMAL 3=ARM 4=ESTOP 5=FLIPPER
    uint8_t mode = 1;                              // STANDBY

    uint8_t flags = 0;
    if (sensor_enable) flags |= 0x02;              // sensors_on
    flags |= 0x04;                                 // can_ok (optimistic)

    uint32_t uptime = millis();

    uint8_t pl[24] = {};
    pl[0] = mode;
    pl[1] = flags;

    int idx = 20;
    pl[idx++] =  uptime        & 0xFF;
    pl[idx++] = (uptime >>  8) & 0xFF;
    pl[idx++] = (uptime >> 16) & 0xFF;
    pl[idx++] = (uptime >> 24) & 0xFF;

    sendFrame(MSG_TELEMETRY, pl, 24);
}

// ────────────────────────────────────────────────────────────────────────────
// Setup
// ────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.setTxBufferSize(4096);     // thermal frame is 1543 bytes → need slack
    Serial.begin(921600);

    // I2C sensors
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    pinMode(PIN_MQ2, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_MQ2, ADC_11db);

    has_lis3mdl = lis3mdl.begin_I2C();
    if (has_lis3mdl) {
        lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
    }
    has_mlx = mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
    if (has_mlx) {
        mlx.setMode(MLX90640_CHESS);
        mlx.setResolution(MLX90640_ADC_18BIT);
        mlx.setRefreshRate(MLX90640_8_HZ);
        Wire.setClock(400000);
    }

    /*
    // PPM input
    pinMode(PIN_PPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_PPM), ppmISR, FALLING);

    // CAN bus (MCP2515 shared between ODrive library + VESC helpers)
    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
    rearLeft.begin(CAN_500KBPS, MCP_8MHZ);   // first call configures the bus
    rearRight.begin(CAN_500KBPS, MCP_8MHZ);  // no-op thanks to _configuredBuses

    // ODrive init sequence
    rearLeft.clearErrors();
    rearRight.clearErrors();
    delay(10);
    rearLeft.setControllerMode (ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    rearRight.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    delay(10);    pl[idx++] = spdL & 0xFF;            pl[idx++] = (uint16_t)spdL >> 8;
    pl[idx++] = spdR & 0xFF;            pl[idx++] = (uint16_t)spdR >> 8;
    pl[idx++] = flip & 0xFF;            pl[idx++] = (uint16_t)flip >> 8;
    setOdriveRPM(rearLeft,  0.0f);
    setOdriveRPM(rearRight, 0.0f);
    delay(10);
    rearLeft.setAxisState (ODrive::CLOSED_LOOP_CONTROL);
    rearRight.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    */
}

// ────────────────────────────────────────────────────────────────────────────
// Loop (Core 1, Arduino default task)
// ────────────────────────────────────────────────────────────────────────────
void loop() {
    // Serial RX (binary protocol)
    while (Serial.available()) processRxByte(Serial.read());

    // Drain incoming CAN frames into both ODrive state trackers
    /*
    can_frame f;
    while (mcp2515.readMessage(&f) == MCP2515::ERROR_OK) {
        rearLeft.feedMsg(f);
        rearRight.feedMsg(f);
    }

    // Hard-stop on ODrive fatal error → trip the soft e-stop; keep running so
    // the bridge still sees telemetry and can issue ESTOP_CLEAR/clearErrors.
    if (rearLeft.axisError != 0 || rearRight.axisError != 0) softEstop = true;
    */

    static uint32_t lastCtrlMs  = 0;
    static uint32_t lastSensMs  = 0;
    static uint32_t lastTelemMs = 0;
    uint32_t now = millis();

    /*

    // ── 50 Hz control cycle ───────────────────────────────────────────────
    if (now - lastCtrlMs >= CAN_PERIOD_MS) {
        float dt = (now - lastCtrlMs) / 1000.0f;
        lastCtrlMs = now;

        uint32_t ageMs = (micros() - ppmLastFrameUs) / 1000;
        float tgtL = 0, tgtR = 0, tgtFL = 0, tgtFR = 0, tgtRL = 0, tgtRR = 0;

        if (softEstop || !ppmFrameReady || ageMs > FAILSAFE_MS) {
            inFailsafe = (!ppmFrameReady || ageMs > FAILSAFE_MS);
            armed = false;
        } else {
            inFailsafe = false;

            int thr_us = (int)readPpm(CH_THROTTLE);
            int str_us = (int)readPpm(CH_STEERING);
            int mov_us = (int)readPpm(CH_FLIPPER_MOVE);
            int sel_us = (int)readPpm(CH_FLIPPER_SELECT);
            int pr_us  = (int)readPpm(CH_FLIPPER_PAIR);

            float throttle    =  ppmToNorm(thr_us);
            float steering    = -ppmToNorm(str_us);
            float flipperMove =  ppmToNorm(mov_us);

            if (!armed) {
                if (fabsf(throttle)    < 0.05f &&
                    fabsf(steering)    < 0.05f &&
                    fabsf(flipperMove) < 0.05f) armed = true;
            } else {
                // Diff-drive mix (FIXED: reference used clampInt which truncated floats)
                float leftMix  = constrain(throttle + steering, -1.0f, 1.0f);
                float rightMix = constrain(throttle - steering, -1.0f, 1.0f);
                tgtL = leftMix  * RPM_MAX;
                tgtR = rightMix * RPM_MAX;

                // Flipper selection
                float frontBase = flipperMove * FLIPPER_FRONT_RPM_MAX;
                float rearBase  = flipperMove * FLIPPER_REAR_RPM_MAX;
                bool both  = (sel_us >= FLIP_SEL_CENTER_LOW && sel_us <= FLIP_SEL_CENTER_HIGH);
                bool left  = (sel_us <  FLIP_SEL_CENTER_LOW);
                bool right = (sel_us >  FLIP_SEL_CENTER_HIGH);
                bool front = (pr_us  <  FLIP_PAIR_FRONT_MAX);
                bool rear  = (pr_us  >  FLIP_PAIR_REAR_MIN);

                if (front) {
                    if      (both)  { tgtFL = frontBase; tgtFR = frontBase; }
                    else if (left)  { tgtFL = frontBase; }
                    else if (right) { tgtFR = frontBase; }
                }
                if (rear) {
                    if      (both)  { tgtRL = rearBase; tgtRR = rearBase; }
                    else if (left)  { tgtRL = rearBase; }
                    else if (right) { tgtRR = rearBase; }
                }

                if (INVERT_LEFT)     tgtL  = -tgtL;
                if (INVERT_RIGHT)    tgtR  = -tgtR;
                if (INVERT_FLIP_FL)  tgtFL = -tgtFL;
                if (INVERT_FLIP_FR)  tgtFR = -tgtFR;
                if (INVERT_FLIP_RL)  tgtRL = -tgtRL;
                if (INVERT_FLIP_RR)  tgtRR = -tgtRR;
            }
        }

        // Ramps (applied even in failsafe to avoid slam-braking)
        rpmLeftCmd  = rampTowards(rpmLeftCmd,  tgtL,  TRACTION_ACCEL,   dt);
        rpmRightCmd = rampTowards(rpmRightCmd, tgtR,  TRACTION_ACCEL,   dt);
        rpmFlFlCmd  = rampTowards(rpmFlFlCmd,  tgtFL, FRONT_FLIP_ACCEL, dt);
        rpmFlFrCmd  = rampTowards(rpmFlFrCmd,  tgtFR, FRONT_FLIP_ACCEL, dt);
        rpmFlRlCmd  = rampTowards(rpmFlRlCmd,  tgtRL, REAR_FLIP_ACCEL,  dt);
        rpmFlRrCmd  = rampTowards(rpmFlRrCmd,  tgtRR, REAR_FLIP_ACCEL,  dt);

        // Snap tiny residuals to zero to stop humming
        if (fabsf(rpmLeftCmd)  < 20 && tgtL  == 0) rpmLeftCmd  = 0;
        if (fabsf(rpmRightCmd) < 20 && tgtR  == 0) rpmRightCmd = 0;
        if (fabsf(rpmFlRlCmd)  < 10 && tgtRL == 0) rpmFlRlCmd  = 0;
        if (fabsf(rpmFlRrCmd)  < 10 && tgtRR == 0) rpmFlRrCmd  = 0;

        // 1. VESCs first (tracks + front flippers) — 4 × 8-byte CAN frames
        sendVescRPM(VESC_LEFT_ID,    (int32_t)rpmLeftCmd);
        sendVescRPM(VESC_RIGHT_ID,   (int32_t)rpmRightCmd);
        sendVescRPM(VESC_FLIP_FL_ID, (int32_t)rpmFlFlCmd);
        sendVescRPM(VESC_FLIP_FR_ID, (int32_t)rpmFlFrCmd);

        // 2. Give the MCP2515's 3 TX buffers a moment to drain
        vTaskDelay(pdMS_TO_TICKS(2));

        // 3. Rear flippers via ODrive (bypass the buggy int setRPMs)
        setOdriveRPM(rearLeft,  rpmFlRlCmd);
        setOdriveRPM(rearRight, rpmFlRrCmd);
    }
    */

    // ── 10 Hz sensor publishing (only enabled streams) ────────────────────
    if (now - lastSensMs >= 100) {
        lastSensMs = now;
        if (sensor_enable & BIT_GAS)     { readMQ2();     publishGas();     }
        if (sensor_enable & BIT_MAG)     { readLIS3MDL(); publishMag();     }
        if (sensor_enable & BIT_THERMAL) { readMLX90640(); publishThermal(); }
    }

    // ── 10 Hz mode / PPM / commanded-speed telemetry ─────────────────────
    if (now - lastTelemMs >= 100) {
        lastTelemMs = now;
        publishTelemetry();
    }
}
