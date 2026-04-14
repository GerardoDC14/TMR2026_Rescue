#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "ODrive.hpp"

// ======================================================
//                      PPM CONFIG
// ======================================================
#define PPM_PIN               4
#define CHANNELS              6
#define BAUD_RATE             115200
#define SYNC_PULSE_US         2100
#define PPM_MIN_US            1000
#define PPM_MAX_US            2000
#define PPM_DEADBAND_LOW      1480
#define PPM_DEADBAND_HIGH     1520

#define STEERING_CH           0   
#define THROTTLE_CH           1   
#define FLIPPER_MOVE_CH       2   
#define FLIPPER_SELECT_CH     3   
#define FLIPPER_PAIR_CH       4   

volatile uint32_t ppmValues[CHANNELS] = {1500,1500,1500,1500,1500,1500};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPulse = 0;
volatile uint32_t lastFrameMicros = 0;
volatile bool frameReady = false;

// ======================================================
//                  CAN / VESC / ODRIVE CONFIG
// ======================================================
static const uint8_t PIN_CS = 5;
static const uint8_t PIN_CAN_SCK  = 18;
static const uint8_t PIN_CAN_MISO = 19;
static const uint8_t PIN_CAN_MOSI = 23;

static const uint8_t VESC_LEFT_ID   = 10;
static const uint8_t VESC_RIGHT_ID  = 20;
static const uint8_t VESC_FLIPPER_FRONT_LEFT_ID  = 30;
static const uint8_t VESC_FLIPPER_FRONT_RIGHT_ID = 40;
static const uint8_t CMD_SET_RPM = 3;

MCP2515 mcp2515(PIN_CS);
struct can_frame tx;

ODrive rearRight(50, &mcp2515);
ODrive rearLeft(60, &mcp2515);

// ======================================================
//                  CONTROL PARAMETERS
// ======================================================
static const float RPM_MAX = 14000.0f;
static const float FLIPPER_FRONT_RPM_MAX = 10500.0f;
static const float FLIPPER_REAR_RPM_MAX  = 1500.0f;

static const float TRACTION_ACCELERATION_RPM_PER_S      = 6000.0f;
static const float FRONT_FLIPPER_ACCELERATION_RPM_PER_S = 2000.0f;
static const float REAR_FLIPPER_ACCELERATION_RPM_PER_S  = 1000.0f;

static const uint32_t CAN_PERIOD_MS = 20;   // 50 Hz
static const uint32_t FAILSAFE_MS   = 300;

#define INVERT_LEFT_MOTOR              false
#define INVERT_RIGHT_MOTOR             false
#define INVERT_FRONT_LEFT_FLIPPER      false
#define INVERT_FRONT_RIGHT_FLIPPER     false
#define INVERT_REAR_LEFT_FLIPPER       false
#define INVERT_REAR_RIGHT_FLIPPER      false

#define FLIPPER_SELECT_CENTER_LOW      1450
#define FLIPPER_SELECT_CENTER_HIGH     1550
#define FLIPPER_PAIR_FRONT_MAX         1450
#define FLIPPER_PAIR_REAR_MIN          1550

// ======================================================
//            VARIABLES GLOBALES PARA TELEMETRÍA
// ======================================================
volatile float gl_rpmLeftCmd = 0.0f;
volatile float gl_rpmRightCmd = 0.0f;
volatile float gl_rpmFrontLeftCmd = 0.0f;
volatile float gl_rpmFrontRightCmd = 0.0f;
volatile float gl_rpmRearLeftCmd = 0.0f;
volatile float gl_rpmRearRightCmd = 0.0f;
volatile bool  gl_sistemaArmado = false;
volatile bool  gl_inFailsafe = true;

// Tarea de FreeRTOS para el Core 0
TaskHandle_t TelemetryTask;

// ======================================================
//                    HELPER FUNCTIONS
// ======================================================
static inline uint32_t vescEID(uint8_t unit_id, uint8_t cmd) {
    return ((uint32_t)cmd << 8) | unit_id;
}

static inline void putInt32BE(uint8_t *d, int32_t v) {
    d[0] = (v >> 24) & 0xFF; d[1] = (v >> 16) & 0xFF;
    d[2] = (v >>  8) & 0xFF; d[3] = (v >>  0) & 0xFF;
}

static inline int clampInt(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float ppmToNormalized(int ppm) {
    ppm = clampInt(ppm, PPM_MIN_US, PPM_MAX_US);
    if (ppm >= PPM_DEADBAND_LOW && ppm <= PPM_DEADBAND_HIGH) return 0.0f;
    if (ppm > PPM_DEADBAND_HIGH) {
        return (float)(ppm - PPM_DEADBAND_HIGH) / (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
    } else {
        return -(float)(PPM_DEADBAND_LOW - ppm) / (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
    }
}

float rampTowards(float current, float target, float accel_rpm_per_s, float dt) {
    float maxStep = accel_rpm_per_s * dt;
    if (target > current + maxStep) return current + maxStep;
    else if (target < current - maxStep) return current - maxStep;
    else return target;
}

void sendVescRPM(uint8_t vesc_id, int32_t rpm) {
    tx.can_id  = vescEID(vesc_id, CMD_SET_RPM) | CAN_EFF_FLAG;
    tx.can_dlc = 4;
    putInt32BE(tx.data, rpm);
    mcp2515.sendMessage(&tx);
}

void sendVescGroupRPM(int32_t rpmLeft, int32_t rpmRight, int32_t rpmFrontLeft, int32_t rpmFrontRight) {
    sendVescRPM(VESC_LEFT_ID, rpmLeft);
    sendVescRPM(VESC_RIGHT_ID, rpmRight);
    sendVescRPM(VESC_FLIPPER_FRONT_LEFT_ID, rpmFrontLeft);
    sendVescRPM(VESC_FLIPPER_FRONT_RIGHT_ID, rpmFrontRight);
}

// ======================================================
//                      PPM ISR
// ======================================================
void IRAM_ATTR ppmISR() {
    uint32_t now = micros();
    uint32_t pulseWidth = now - lastPulse;
    lastPulse = now;
    if (pulseWidth > SYNC_PULSE_US) {
        currentChannel = 0;
        frameReady = true;
        lastFrameMicros = now;
    } else {
        if (currentChannel < CHANNELS) {
            ppmValues[currentChannel] = pulseWidth;
            currentChannel++;
        }
    }
}

// ======================================================
//        TAREA CORE 0: TELEMETRÍA (Lenta y Bloqueante)
// ======================================================
void telemetryTaskCode(void * parameter) {
    for(;;) {
        Serial.print(gl_inFailsafe ? "[FAILSAFE] " : "[OK] ");
        Serial.print("Armado: "); Serial.print(gl_sistemaArmado ? "SI" : "NO");
        Serial.print(" | Trac L:"); Serial.print(gl_rpmLeftCmd, 0);
        Serial.print(" R:"); Serial.print(gl_rpmRightCmd, 0);
        Serial.print(" | F.Flip L:"); Serial.print(gl_rpmFrontLeftCmd, 0);
        Serial.print(" R:"); Serial.print(gl_rpmFrontRightCmd, 0);
        Serial.print(" | R.Flip L:"); Serial.print(gl_rpmRearLeftCmd, 0);
        Serial.print(" R:"); Serial.print(gl_rpmRearRightCmd, 0);
        
        Serial.print(" | ErrL:0x"); Serial.print(rearLeft.axisError, HEX);
        Serial.print(" ErrR:0x"); Serial.println(rearRight.axisError, HEX);
        
        // Esperamos 100ms para no saturar la pantalla
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// ======================================================
//                        SETUP
// ======================================================
void setup() {
    Serial.begin(BAUD_RATE);
    delay(1000);

    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);

    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CS);

    // Ajustado a 500KBPS según tu solicitud.
    if (!rearRight.begin(CAN_500KBPS, MCP_8MHZ)) {
        Serial.println("Error inicializando MCP2515");
        while (1);
    }

    rearRight.clearErrors();
    rearLeft.clearErrors();
    delay(10);

    rearRight.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    rearLeft.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    delay(10);

    rearRight.setRPMs(0);
    rearLeft.setRPMs(0);
    delay(10);

    rearRight.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    rearLeft.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    delay(10);

    // Creamos la tarea de telemetría en el Core 0 (Procesador PRO)
    xTaskCreatePinnedToCore(
        telemetryTaskCode, "TelemetryTask", 10000, NULL, 1, &TelemetryTask, 0);

    Serial.println("Sistema Iniciado - Esperando Palancas en Neutral");
}

// ======================================================
//        TAREA CORE 1: LOOP DE CONTROL (Rápido y Preciso)
// ======================================================
void loop() {
    static uint32_t lastCanMs = 0;
    
    // Variables locales para evitar latigazos
    static float rpmLeftCmd = 0.0f;
    static float rpmRightCmd = 0.0f;
    static float rpmFrontLeftCmd = 0.0f;
    static float rpmFrontRightCmd = 0.0f;
    static float rpmRearLeftCmd = 0.0f;
    static float rpmRearRightCmd = 0.0f;

    uint32_t nowMs = millis();

    // 1. Leer mensajes entrantes del CAN (Para vaciar el buffer)
    can_frame frame;
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        rearRight.feedMsg(frame);
        rearLeft.feedMsg(frame);
    }

    // 2. Paro de Emergencia por fallas en ODrive
    if (rearRight.axisError != 0 || rearLeft.axisError != 0) {
        rearRight.Estop();
        rearLeft.Estop();
        sendVescGroupRPM(0, 0, 0, 0); // Frena VESC también
        vTaskSuspend(TelemetryTask);  // Detiene la impresión normal
        Serial.println("\n!!! E-STOP: ODRIVE FATAL ERROR !!!");
        while (true) { delay(1000); }
    }

    // 3. Bucle principal a 50Hz
    if (nowMs - lastCanMs >= CAN_PERIOD_MS) {
        float dt = (nowMs - lastCanMs) / 1000.0f;
        lastCanMs = nowMs;

        uint32_t ageMs = (micros() - lastFrameMicros) / 1000;
        
        float rpmLeftTarget = 0.0f;
        float rpmRightTarget = 0.0f;
        float rpmFrontLeftTarget = 0.0f;
        float rpmFrontRightTarget = 0.0f;
        float rpmRearLeftTarget = 0.0f;
        float rpmRearRightTarget = 0.0f;

        // VERIFICACIÓN DE FAILSAFE (Solo ponemos METAS a 0)
        if (ageMs > FAILSAFE_MS || !frameReady) {
            gl_inFailsafe = true;
            gl_sistemaArmado = false;
        } else {
            gl_inFailsafe = false;

            uint32_t throttle_us, steering_us, flipperMove_us, flipperSelect_us, flipperPair_us;
            noInterrupts();
            throttle_us      = ppmValues[THROTTLE_CH];
            steering_us      = ppmValues[STEERING_CH];
            flipperMove_us   = ppmValues[FLIPPER_MOVE_CH];
            flipperSelect_us = ppmValues[FLIPPER_SELECT_CH];
            flipperPair_us   = ppmValues[FLIPPER_PAIR_CH];
            interrupts();

            float throttle = ppmToNormalized((int)throttle_us);
            float steering = -ppmToNormalized((int)steering_us);
            float flipperMove = ppmToNormalized((int)flipperMove_us);

            // LOGICA DE ARMADO SEGURO
            if (!gl_sistemaArmado) {
                // Si todas las palancas de movimiento están en el centro (+/- 5%)
                if (fabs(throttle) < 0.05f && fabs(steering) < 0.05f && fabs(flipperMove) < 0.05f) {
                    gl_sistemaArmado = true;
                }
            } else {
                // TRACCIÓN DIFERENCIAL
                float leftMix  = clampInt(throttle + steering, -1.0f, 1.0f);
                float rightMix = clampInt(throttle - steering, -1.0f, 1.0f);

                rpmLeftTarget  = leftMix  * RPM_MAX;
                rpmRightTarget = rightMix * RPM_MAX;

                if (INVERT_LEFT_MOTOR)  rpmLeftTarget  = -rpmLeftTarget;
                if (INVERT_RIGHT_MOTOR) rpmRightTarget = -rpmRightTarget;

                // LOGICA DE FLIPPERS
                float frontBaseTarget = flipperMove * FLIPPER_FRONT_RPM_MAX;
                float rearBaseTarget  = flipperMove * FLIPPER_REAR_RPM_MAX;

                bool selectBoth  = (flipperSelect_us >= FLIPPER_SELECT_CENTER_LOW && flipperSelect_us <= FLIPPER_SELECT_CENTER_HIGH);
                bool selectLeft  = (flipperSelect_us < FLIPPER_SELECT_CENTER_LOW);
                bool selectRight = (flipperSelect_us > FLIPPER_SELECT_CENTER_HIGH);
                bool selectFrontPair = (flipperPair_us < FLIPPER_PAIR_FRONT_MAX);
                bool selectRearPair  = (flipperPair_us > FLIPPER_PAIR_REAR_MIN);

                if (selectFrontPair) {
                    if (selectBoth) { rpmFrontLeftTarget = frontBaseTarget; rpmFrontRightTarget = frontBaseTarget; }
                    else if (selectLeft) { rpmFrontLeftTarget = frontBaseTarget; }
                    else if (selectRight) { rpmFrontRightTarget = frontBaseTarget; }
                }

                if (selectRearPair) {
                    if (selectBoth) { rpmRearLeftTarget = rearBaseTarget; rpmRearRightTarget = rearBaseTarget; }
                    else if (selectLeft) { rpmRearLeftTarget = rearBaseTarget; }
                    else if (selectRight) { rpmRearRightTarget = rearBaseTarget; }
                }

                if (INVERT_FRONT_LEFT_FLIPPER)   rpmFrontLeftTarget  = -rpmFrontLeftTarget;
                if (INVERT_FRONT_RIGHT_FLIPPER)  rpmFrontRightTarget = -rpmFrontRightTarget;
                if (INVERT_REAR_LEFT_FLIPPER)    rpmRearLeftTarget   = -rpmRearLeftTarget;
                if (INVERT_REAR_RIGHT_FLIPPER)   rpmRearRightTarget  = -rpmRearRightTarget;
            }
        }

        // APLICAR RAMPAS SUAVES (Incluso en Failsafe para no frenar de golpe)
        rpmLeftCmd  = rampTowards(rpmLeftCmd,  rpmLeftTarget,  TRACTION_ACCELERATION_RPM_PER_S, dt);
        rpmRightCmd = rampTowards(rpmRightCmd, rpmRightTarget, TRACTION_ACCELERATION_RPM_PER_S, dt);
        rpmFrontLeftCmd  = rampTowards(rpmFrontLeftCmd,  rpmFrontLeftTarget,  FRONT_FLIPPER_ACCELERATION_RPM_PER_S, dt);
        rpmFrontRightCmd = rampTowards(rpmFrontRightCmd, rpmFrontRightTarget, FRONT_FLIPPER_ACCELERATION_RPM_PER_S, dt);
        rpmRearLeftCmd  = rampTowards(rpmRearLeftCmd,  rpmRearLeftTarget,  REAR_FLIPPER_ACCELERATION_RPM_PER_S, dt);
        rpmRearRightCmd = rampTowards(rpmRearRightCmd, rpmRearRightTarget, REAR_FLIPPER_ACCELERATION_RPM_PER_S, dt);

        // Limpieza de ceros puros para evitar zumbidos
        if (fabs(rpmLeftCmd) < 20.0f && rpmLeftTarget == 0.0f) rpmLeftCmd = 0.0f;
        if (fabs(rpmRightCmd) < 20.0f && rpmRightTarget == 0.0f) rpmRightCmd = 0.0f;
        if (fabs(rpmRearLeftCmd) < 10.0f && rpmRearLeftTarget == 0.0f) rpmRearLeftCmd = 0.0f;
        if (fabs(rpmRearRightCmd) < 10.0f && rpmRearRightTarget == 0.0f) rpmRearRightCmd = 0.0f;

        // Actualizar variables para la telemetría (Core 0)
        gl_rpmLeftCmd = rpmLeftCmd; gl_rpmRightCmd = rpmRightCmd;
        gl_rpmFrontLeftCmd = rpmFrontLeftCmd; gl_rpmFrontRightCmd = rpmFrontRightCmd;
        gl_rpmRearLeftCmd = rpmRearLeftCmd; gl_rpmRearRightCmd = rpmRearRightCmd;

        // ==================================================
        //             TRANSMISIÓN CAN ESCALONADA
        // ==================================================
        
        // 1. Llenamos el buffer con los VESC
        sendVescGroupRPM((int32_t)rpmLeftCmd, (int32_t)rpmRightCmd, 
                         (int32_t)rpmFrontLeftCmd, (int32_t)rpmFrontRightCmd);

        // 2. RETRASO VITAL DE 2 MILISEGUNDOS PARA EVITAR OVERFLOW DEL MCP2515
        vTaskDelay(pdMS_TO_TICKS(2));

        // 3. Enviamos a las ODrive ya con la calle libre
        rearLeft.setRPMs(rpmRearLeftCmd);
        rearRight.setRPMs(rpmRearRightCmd);
    }
}