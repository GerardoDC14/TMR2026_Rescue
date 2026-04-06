#include "driver/twai.h"
#include "ODrive.hpp"

#define TWAI_TX GPIO_NUM_22
#define TWAI_RX GPIO_NUM_21

// -------------------- PPM CONFIG --------------------
#define PPM_PIN            4
#define CHANNELS           6
#define SYNC_PULSE_US      2100
#define BAUD_RATE          115200

#define PPM_DEADBAND_LOW   1480
#define PPM_DEADBAND_HIGH  1520
#define PPM_MIN_US         1000
#define PPM_MAX_US         2000

volatile uint32_t ppmValues[CHANNELS] = {0};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPulse = 0;
volatile bool frameReady = false;
volatile uint32_t lastFrameMs = 0;

// -------------------- CONTROL TUNING --------------------
static const int32_t RPM_MAX = 2500;      // RPM máximo a pedir
static const uint32_t PRINT_PERIOD = 50;  // Mostrar datos cada 50ms
static const uint32_t FAILSAFE_MS = 200;  // Tiempo sin señal para failsafe

static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Odrive declaration
ODrive can(1);

void IRAM_ATTR ppmISR() {
  uint32_t nowUs = micros(); // Necesario para medir el ancho del pulso
  uint32_t pulseWidth = nowUs - lastPulse;
  lastPulse = nowUs;

  if (pulseWidth > SYNC_PULSE_US) {
    currentChannel = 0;
    lastFrameMs = millis(); // Registramos la última vez que llegó señal (en milisegundos)
  } else {
    if (currentChannel < CHANNELS) {
      ppmValues[currentChannel] = pulseWidth;
      currentChannel++;
    }
  }
}

float ppmToNormalized(int ppm) {
  ppm = clampInt(ppm, PPM_MIN_US, PPM_MAX_US);

  // Zona muerta (centro del stick)
  if (ppm >= PPM_DEADBAND_LOW && ppm <= PPM_DEADBAND_HIGH) return 0.0f;

  if (ppm > PPM_DEADBAND_HIGH) {
    // Escala positiva [1520..2000] -> [0..1]
    return (float)(ppm - PPM_DEADBAND_HIGH) / (float)(PPM_MAX_US - PPM_DEADBAND_HIGH);
  } else {
    // Escala negativa [1480..1000] -> [0..-1]
    return -(float)(PPM_DEADBAND_LOW - ppm) / (float)(PPM_DEADBAND_LOW - PPM_MIN_US);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
  Serial.println("Lectura PPM iniciada...");

  delay(5000);
  Serial.println("Starting CAN sniffer...");

  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  Serial.print("driver_install: ");
  Serial.println(err);

  err = twai_start();
  Serial.print("twai_start: ");
  Serial.println(err);

  if (err == ESP_OK) {
    Serial.println("Listening at 500 kbps...");
  }
}

void loop() {
  twai_message_t msg;

  while (twai_receive(&msg, 0) == ESP_OK) {
      can.feedMsg(msg); 
  }

  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();

  // Ejecutar lógica cada 50ms
  if (nowMs - lastPrintMs >= PRINT_PERIOD) {
    lastPrintMs = nowMs;
    can.requestVoltage();
    Serial.print("Current State: ");
    Serial.println(can.currentState);
    Serial.print("bus Current: ");
    Serial.println(can.busCurrent);
    Serial.print("Bus Voltage: ");
    Serial.println(can.busVoltage);
    Serial.print("Sensorless pos estimate: ");
    Serial.println(can.senlessEstPos);
    Serial.print("Sensorless vel estimate: ");
    Serial.println(can.senlessEstVel);
    Serial.println();

    // Failsafe: Si han pasado más de 200ms sin señal
    if (nowMs - lastFrameMs > FAILSAFE_MS) {
      Serial.println("FAILSAFE ACTIVO - Sin señal del control");
      return; // Evita que el motor se mueva
    }

    // Leer el canal de aceleración (CH2 = índice 1) de forma segura
    uint32_t ch2_us;
    noInterrupts();
    ch2_us = ppmValues[1];
    interrupts();

    // 1. Normalizar el valor
    float normalizado = ppmToNormalized((int)ch2_us);     
    
    // 2. Convertir a objetivo de RPM
    int32_t rpmTarget = (int32_t)(normalizado * RPM_MAX); 

    Serial.printf("Señal (us): %u | Mapeo: %.2f | RPM Objetivo: %d\n", ch2_us, normalizado, rpmTarget);
  }

  // Falta declarar la funcion de envio de RPMs dentro de la libreria de ODrive
}