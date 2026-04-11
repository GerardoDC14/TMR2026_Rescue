#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "ODrive.hpp"

// -------------------- PPM CONFIG --------------------
#define PPM_PIN            21
#define CHANNELS           6
#define SYNC_PULSE_US      2100
#define BAUD_RATE          115200

#define PPM_DEADBAND_LOW   1450
#define PPM_DEADBAND_HIGH  1550
#define PPM_MIN_US         1000
#define PPM_MAX_US         2000

// Inicializamos en 1500 (centro) para evitar basura al encender
volatile uint32_t ppmValues[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPulse = 0;
volatile bool frameReady = false;
volatile uint32_t lastFrameMicros = 0;

void IRAM_ATTR ppmISR() {
    uint32_t now = micros();
    uint32_t pulseWidth = now - lastPulse;
    lastPulse = now;

    if (pulseWidth > SYNC_PULSE_US) {
        // Solo aceptamos el frame si recibimos suficientes canales (ignora el ruido)
        if (currentChannel >= 2) { 
            frameReady = true;
            lastFrameMicros = now;
        }
        currentChannel = 0;
    } else {
        if (currentChannel < CHANNELS) {
            ppmValues[currentChannel] = pulseWidth;
            currentChannel++;
        }
    }
}

// -------------------- ODRIVE / CAN CONFIG --------------------
const uint8_t PIN_CAN_CS   = 5;
const uint8_t PIN_CAN_SCK  = 18;
const uint8_t PIN_CAN_MISO = 19;
const uint8_t PIN_CAN_MOSI = 23;

MCP2515 mcp2515(PIN_CAN_CS);
ODrive motorA(0x02, &mcp2515);
ODrive motorB(0x03, &mcp2515);

// -------------------- CONTROL TUNING --------------------
static const float VEL_MAX_RPM  = 2500.0f;  
static const float ACELERACION  = 1000.0f;  

static const uint32_t CAN_PERIOD_MS = 20;   
static const uint32_t FAILSAFE_MS = 200;    

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

void setup() {
    Serial.begin(BAUD_RATE);
    delay(1000);

    // Activamos PULLUP para evitar que el pin "flote" y lea basura
    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
    Serial.println("Receptor PPM Iniciado");

    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

    if (!motorA.begin(CAN_1000KBPS, MCP_8MHZ)) {
        Serial.println("Error inicializando MCP2515");
        while (1);
    }
    
    // --- SECUENCIA DE INICIO SEGURO ODRIVE ---
    motorA.clearErrors();
    motorB.clearErrors();
    delay(10);
    
    motorA.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    motorB.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::PASSTHROUGH);
    delay(10);
    
    // 1. MANDAMOS 0 RPM ANTES DE CERRAR EL LAZO (Evita el jalón)
    motorA.setRPMs(0);
    motorB.setRPMs(0);
    delay(10);
    
    // 2. AHORA SÍ CERRAMOS EL LAZO
    motorA.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    motorB.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    delay(10);
    
    Serial.println("ODrive armado en 0 RPM. Esperando palanca en neutral...");
}

void loop() {
    static uint32_t lastCanMs = 0;
    static uint32_t lastTelemetryMs = 0;
    
    // Variables de seguridad
    static bool inFailsafe = true; // Iniciamos asumiendo que no hay señal
    static bool sistemaArmado = false; // Lock de seguridad
    
    static float velocidadActualCmd = 0.0f;

    uint32_t nowMs = millis();

    can_frame frame;
    // 1. LEER MENSAJES DEL BUS CAN
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        motorA.feedMsg(frame);
        motorB.feedMsg(frame);
    }

    // ---------------------------------------------------------
    // ¡NUEVO! CORTACORRIENTES DE EMERGENCIA POR ERROR DE ODRIVE
    // ---------------------------------------------------------
    if (motorA.axisError != 0 || motorB.axisError != 0) {
        Serial.println("\n==================================================");
        Serial.println("      !!! ERROR CRITICO DETECTADO EN ODRIVE !!!     ");
        Serial.println("==================================================");
        
        if (motorA.axisError != 0) {
            Serial.print(">> Falla en Motor A (Nodo 0x02). Codigo: 0x");
            Serial.println(motorA.axisError, HEX);
        }
        if (motorB.axisError != 0) {
            Serial.print(">> Falla en Motor B (Nodo 0x03). Codigo: 0x");
            Serial.println(motorB.axisError, HEX);
        }

        Serial.println("Enviando comando E-STOP (Parada de emergencia) a ambos motores...");
        motorA.Estop();
        motorB.Estop();

        Serial.println("Sistema bloqueado por seguridad. Desconecta la bateria o reinicia el ESP32.");
        
        // Atrapamos al microcontrolador en un bucle infinito para evitar que 
        // vuelva a mandar comandos de velocidad.
        while (true) {
            delay(1000); 
        }
    }
    // ---------------------------------------------------------

    // Bucle de Control a 50Hz (20ms)
    if (nowMs - lastCanMs >= CAN_PERIOD_MS) {
        float dt = (nowMs - lastCanMs) / 1000.0f;
        lastCanMs = nowMs;

        uint32_t ageMs = (micros() - lastFrameMicros) / 1000;
        float velocidadObjetivo = 0.0f;
        if (ageMs > FAILSAFE_MS || !frameReady) {
            if (!inFailsafe) {
                Serial.println("FAILSAFE: Señal perdida. Forzando 0 RPM y desarmando.");
                inFailsafe = true;
                sistemaArmado = false;
            }
            velocidadObjetivo = 0.0f;
        } 
        else {
            inFailsafe = false;
            
            uint32_t ch2_us;
            noInterrupts();
            ch2_us = ppmValues[1]; 
            interrupts();

            float joystickNorm = ppmToNormalized((int)ch2_us); 
            
            if (!sistemaArmado) {
                // Le damos una tolerancia del 5% (+/- 0.05) para considerar que está "al centro"
                if (joystickNorm > -0.05f && joystickNorm < 0.05f) {
                    sistemaArmado = true;
                    Serial.println("PALANCA EN NEUTRAL: Sistema Armado y Listo para moverse.");
                }
                velocidadObjetivo = 0.0f; // Mientras no esté armado, el objetivo sigue siendo 0
            } else {
                // Una vez armado, procedemos normal
                velocidadObjetivo = joystickNorm * VEL_MAX_RPM; 
            }
        }

        float cambioMaximo = ACELERACION * dt; 

        if (velocidadObjetivo > velocidadActualCmd + cambioMaximo) {
            velocidadActualCmd += cambioMaximo; 
        } 
        else if (velocidadObjetivo < velocidadActualCmd - cambioMaximo) {
            velocidadActualCmd -= cambioMaximo; 
        } 
        else {
            velocidadActualCmd = velocidadObjetivo; 
        }

        // motorA.setRPMs(velocidadActualCmd);
        motorB.setRPMs(velocidadActualCmd);
    }

    // Telemetría a 2Hz (500ms)
    if (nowMs - lastTelemetryMs >= 500) {
        // Leemos el valor crudo del receptor una sola vez para este bloque
        uint32_t rawPPM;
        noInterrupts();
        rawPPM = ppmValues[1];
        interrupts();

        // TELEMETRIA MOTOR A
        Serial.print("A: Armado: ");
        Serial.print(sistemaArmado ? "SI" : "NO");
        Serial.print(" | Err: 0x"); 
        Serial.print(motorA.axisError, HEX); // Agregado código de error
        Serial.print(" | RPMS: ");
        Serial.print(motorA.getErpms());
        Serial.print(" | Volt: ");
        Serial.print(motorA.getVoltage());
        Serial.print(" | Amp: ");
        Serial.print(motorA.getCurrent());
        
        Serial.println(); // Salto de línea para que se lea mejor

        // TELEMETRIA MOTOR B
        Serial.print("B: Armado: ");
        Serial.print(sistemaArmado ? "SI" : "NO");
        Serial.print(" | Err: 0x"); 
        Serial.print(motorB.axisError, HEX); // Agregado código de error
        Serial.print(" | RPMS: ");
        Serial.print(motorB.getErpms());
        Serial.print(" | Volt: ");
        Serial.print(motorB.getVoltage());
        Serial.print(" | Amp: ");
        Serial.print(motorB.getCurrent());

        Serial.print(" || CH2: ");
        Serial.print(rawPPM);
        Serial.print(" us (");
        Serial.print(ppmToNormalized((int)rawPPM));
        Serial.println(")");
        
        // Peticiones de voltaje para que se actualicen en la siguiente lectura
        motorA.requestVoltage();
        motorB.requestVoltage();
        
        lastTelemetryMs = nowMs;
    }
}