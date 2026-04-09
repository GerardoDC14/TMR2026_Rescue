#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "ODrive.hpp"

const uint8_t PIN_CAN_CS   = 5;
const uint8_t PIN_CAN_SCK  = 18;
const uint8_t PIN_CAN_MISO = 19;
const uint8_t PIN_CAN_MOSI = 23;

MCP2515 mcp2515(PIN_CAN_CS);
ODrive motorA(0x02, &mcp2515);
ODrive motorB(0x03, &mcp2515);

unsigned long tiempoAnteriorMovimiento = 0;
unsigned long tiempoAnteriorTelemetria = 0;
int estadoActual = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);

    Serial.println("Iniciando sistema ODrive ESP32...");

    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

    if (!motorA.begin(CAN_1000KBPS, MCP_8MHZ)) {
        Serial.println("Error iniciando MCP2515. Revisa conexiones.");
        while (1);
    }
    motorB.begin(CAN_1000KBPS, MCP_8MHZ); 

    Serial.println("MCP2515 configurado correctamente.");

    motorA.clearErrors();
    motorB.clearErrors();
    delay(10); 

    motorA.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::VEL_RAMP);
    motorB.setControllerMode(ODrive::VELOCITY_CONTROL, ODrive::VEL_RAMP);
    delay(10);

    motorA.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    motorB.setAxisState(ODrive::CLOSED_LOOP_CONTROL);
    delay(10);

    Serial.println("¡Arrancando motores a 1 rev/segundo!");
    motorA.setRPMs(120);
    motorB.setVelocity(2);

    tiempoAnteriorMovimiento = millis();
    tiempoAnteriorTelemetria = millis();
}

void loop() {
    can_frame frame;
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        motorA.feedMsg(frame);
        motorB.feedMsg(frame);
    }

    unsigned long tiempoActual = millis();

    if (estadoActual == 0) {
        if (tiempoActual - tiempoAnteriorMovimiento >= 5000) {
            Serial.println("Tiempo cumplido. Deteniendo motores...");
            motorA.setRPMs(0);
            motorB.setVelocity(0.0);
            
            estadoActual = 1;
            tiempoAnteriorMovimiento = tiempoActual;
        }
    } 
    else if (estadoActual == 1) {
        if (tiempoActual - tiempoAnteriorMovimiento >= 3000) {
            Serial.println("Reiniciando ciclo: ¡Arrancando a 1 rev/segundo!");
            motorA.setRPMs(120);
            motorB.setVelocity(2);
            
            estadoActual = 0;
            tiempoAnteriorMovimiento = tiempoActual;
        }
    }

    if (tiempoActual - tiempoAnteriorTelemetria >= 500) {
        Serial.print("Motor A -> Vel: ");
        Serial.print(motorA.getEvel());
        Serial.print(" rev/s | V: ");
        Serial.print(motorA.getVoltage());
        Serial.print("V  ||  Motor B -> Vel: ");
        Serial.print(motorB.getErpms());
        Serial.print(" rpm: ");
        Serial.print(motorB.getVoltage());
        Serial.println("V");
        
        motorA.requestVoltage();
        motorB.requestVoltage();

        tiempoAnteriorTelemetria = tiempoActual;
    }
}