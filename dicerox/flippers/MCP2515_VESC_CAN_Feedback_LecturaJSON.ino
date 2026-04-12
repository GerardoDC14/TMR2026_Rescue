// Autor: Adriana González (adaptado a MCP2515)

#include <mcp_can.h>
#include <SPI.h>

// ================== MCP2515 ==================
#define CAN_CS 10 // Pin MCP2515 a la ESP32
MCP_CAN CAN0(CAN_CS); //Crea un objeto del controlador CAN


// ================== Control de la rampa ==================
static int32_t rpm_cmd = 0; // Valor real de RPM enviado a los motores
static int32_t rpm_target = 0; // Valor de RPM esperado
static const int32_t RPM_SLEW = 200; // Máximo cambio en RPMs por ciclo
static const uint32_t CONTROL_PERIOD_MS = 20; // Se actualiza cada 20ms

static uint32_t last_control_time = 0; // Variable de control para el tiempo

// ================== VESC IDs ==================
uint8_t vesc_ids[] = {1,2,3,4,5}; // IDs a utilizar en la comunicación CAN
const int num_vesc = sizeof(vesc_ids)/sizeof(vesc_ids[0]);

// ================== Aplicación de la rampa ==================
int32_t apply_ramp(int32_t target, int32_t current, int32_t step) {
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

void convert_data(uint32_t id, uint8_t *data) {

    uint8_t vesc_id = id & 0xFF;
    if (vesc_id == 0 || vesc_id > 50) return;

    uint8_t cmd = (id >> 8) & 0xFF;

    switch(cmd){

    case 9:{ // Status 1
        int32_t erpm = ((int32_t)data[0] << 24) |
                       ((int32_t)data[1] << 16) |
                       ((int32_t)data[2] << 8)  |
                       ((int32_t)data[3]);

        float current = (((int16_t)data[4] << 8) | data[5]) * 0.1;
        float duty = (((int16_t)data[6] << 8) | data[7]) * 0.001;

        Serial.printf(
          "{\"id\":%d,\"type\":\"status1\",\"erpm\":%ld,\"current\":%.2f,\"duty\":%.3f}\n",
          vesc_id, erpm, current, duty
        );
        break;
    }

    case 14:{ // Status 2
        int32_t ah = ((int32_t)data[0] << 24) |
                     ((int32_t)data[1] << 16) |
                     ((int32_t)data[2] << 8)  |
                     ((int32_t)data[3]);

        int32_t ah_ch = ((int32_t)data[4] << 24) |
                        ((int32_t)data[5] << 16) |
                        ((int32_t)data[6] << 8)  |
                        ((int32_t)data[7]);

        Serial.printf(
          "{\"id\":%d,\"type\":\"status2\",\"ah\":%.4f,\"ah_charged\":%.4f}\n",
          vesc_id, ah * 0.0001, ah_ch * 0.0001
        );
        break;
    }

    case 15:{ // Status 3
        int32_t wh = ((int32_t)data[0] << 24) |
                     ((int32_t)data[1] << 16) |
                     ((int32_t)data[2] << 8)  |
                     ((int32_t)data[3]);

        int32_t wh_ch = ((int32_t)data[4] << 24) |
                        ((int32_t)data[5] << 16) |
                        ((int32_t)data[6] << 8)  |
                        ((int32_t)data[7]);

        Serial.printf(
          "{\"id\":%d,\"type\":\"status3\",\"wh\":%.4f,\"wh_charged\":%.4f}\n",
          vesc_id, wh * 0.0001, wh_ch * 0.0001
        );
        break;
    }

    case 16:{ // Status 4
        float temp_fet = (((int16_t)data[0] << 8) | data[1]) * 0.1;
        float temp_motor = (((int16_t)data[2] << 8) | data[3]) * 0.1;
        float current_in = (((int16_t)data[4] << 8) | data[5]) * 0.1;
        float pid_pos = (((int16_t)data[6] << 8) | data[7]) * 0.02;

        Serial.printf(
          "{\"id\":%d,\"type\":\"status4\",\"temp_fet\":%.1f,\"temp_motor\":%.1f,\"current_in\":%.2f,\"pid_pos\":%.2f}\n",
          vesc_id, temp_fet, temp_motor, current_in, pid_pos
        );
        break;
    }

    case 27:{ // Status 5
        int32_t tacho = ((int32_t)data[0] << 24) |
                        ((int32_t)data[1] << 16) |
                        ((int32_t)data[2] << 8)  |
                        ((int32_t)data[3]);

        float vin = (((int16_t)data[4] << 8) | data[5]) * 0.1;

        Serial.printf(
          "{\"id\":%d,\"type\":\"status5\",\"tacho\":%ld,\"vin\":%.2f}\n",
          vesc_id, tacho, vin
        );
        break;
    }
    }
}

// ================== Funciones con datos ==================
void send_can(uint32_t id, uint8_t *data, uint8_t len) {
    if (len == 0 || len > 8) return; // Salir si la longitud no es correcta
    CAN0.sendMsgBuf(id, 1, len, data); // extended frame
}

void set_duty_cycle(uint8_t id, float duty) {
    if (duty > 1.0) duty = 1.0;
    if (duty < -1.0) duty = -1.0;

    uint32_t can_id = 0x00000000 | id;

    int32_t duty_int = duty * 100000;

    uint8_t data[4];
    data[0] = (duty_int >> 24) & 0xFF;
    data[1] = (duty_int >> 16) & 0xFF;
    data[2] = (duty_int >> 8) & 0xFF;
    data[3] = duty_int & 0xFF;

    send_can(can_id, data, 4);
}

void set_rpm(uint8_t id, int32_t rpm, int32_t polos) {
    int32_t erpm = rpm * (polos / 2.0f);

    uint32_t can_id = 0x00000300 | id;

    uint8_t data[4];
    data[0] = (erpm >> 24) & 0xFF;
    data[1] = (erpm >> 16) & 0xFF;
    data[2] = (erpm >> 8) & 0xFF;
    data[3] = erpm & 0xFF;

    send_can(can_id, data, 4);
}

// ================== Setup ==================
void setup() {
    Serial.begin(115200);
    delay(2000);

    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("MCP2515 OK");
    } else {
        Serial.println("Error MCP2515");
        while(1);
    }

    CAN0.setMode(MCP_NORMAL);
}

// ================== Loop principal ==================
void loop() {

    // ===== Recibe el mensaje =====
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        uint32_t rxId;
        uint8_t len;
        uint8_t buf[8];

        CAN0.readMsgBuf(&rxId, &len, buf);
       if (len == 8){
        convert_data(rxId, buf);}
    }

    // ===== Loop de control =====
    uint32_t now = millis();

    if (now - last_control_time >= CONTROL_PERIOD_MS) {
        last_control_time = now;

        rpm_target = 500;

        rpm_cmd = apply_ramp(rpm_target, rpm_cmd, RPM_SLEW);

        for (int i = 0; i < num_vesc; i++) {
            set_rpm(vesc_ids[i], rpm_cmd, 14);
        }

        Serial.print("RPM enviada: ");
        Serial.println(rpm_cmd);
    }

    delay(5); 
}