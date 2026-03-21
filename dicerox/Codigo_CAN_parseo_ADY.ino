#include "driver/twai.h"

#define TWAI_TX GPIO_NUM_22
#define TWAI_RX GPIO_NUM_21


void convert_data(const twai_message_t &msg){
    uint32_t id = msg.identifier;

    uint8_t vesc_id = id & 0xFF;
    uint8_t cmd = (id >> 8) & 0xFF;

    switch(cmd){

    case 9: //Status 1
        int32_t erpm = ((int32_t)msg.data[0] << 24) |
                       ((int32_t)msg.data[1] << 16) |
                       ((int32_t)msg.data[2] << 8)  |
                       ((int32_t)msg.data[3]);

        int16_t current_CAN = ((int16_t)msg.data[4] << 8) | msg.data[5];
        float current = current_CAN *0.1;

        int16_t duty_cycle_CAN = ((int16_t)msg.data[6] << 8) | msg.data[7];
        float duty_cycle = duty_cycle_CAN *0.001;

        Serial.print("VESC ID: "); Serial.print(vesc_id);
        Serial.print(", ERPM/RPM: "); Serial.print(erpm);
        Serial.print(", Current: "); Serial.print(current);
        Serial.print(", Duty: "); Serial.println(duty_cycle);
        Serial.println("------");
        break;

    case 14: // Status 2
        int32_t amp_hours_CAN = ((int32_t)msg.data[0] << 24) |
                       ((int32_t)msg.data[1] << 16) |
                       ((int32_t)msg.data[2] << 8)  |
                       ((int32_t)msg.data[3]);

        float amp_hours = amp_hours_CAN *0.0001;

        int32_t amp_hours_charged_CAN = ((int32_t)msg.data[4] << 24) |
                       ((int32_t)msg.data[5] << 16) |
                       ((int32_t)msg.data[6] << 8)  |
                       ((int32_t)msg.data[7]);

        float amp_hours_charged = amp_hours_charged_CAN  *0.0001 ;

        Serial.print("VESC ID: "); Serial.print(vesc_id);
        Serial.print(", Amp hours: "); Serial.print(amp_hours);
        Serial.print(", Amp hours charged: "); Serial.println(amp_hours_charged);
        Serial.println("------");
        break;

    case 15: //Status 3
        int32_t watt_hours_CAN = ((int32_t)msg.data[0] << 24) |
                       ((int32_t)msg.data[1] << 16) |
                       ((int32_t)msg.data[2] << 8)  |
                       ((int32_t)msg.data[3]);

        float watt_hours = watt_hours_CAN  *0.0001;

        int32_t watt_hours_charged_CAN = ((int32_t)msg.data[4] << 24) |
                       ((int32_t)msg.data[5] << 16) |
                       ((int32_t)msg.data[6] << 8)  |
                       ((int32_t)msg.data[7]);

        float watt_hours_charged = watt_hours_charged_CAN  *0.0001;

        Serial.print("VESC ID: "); Serial.print(vesc_id);
        Serial.print(", Watt hours: "); Serial.print(watt_hours);
        Serial.print(", Watt hours charged: "); Serial.println(watt_hours_charged);
        Serial.println("------");
        break;

    case 16: //Status 4

        int16_t temp_fet_CAN = ((int16_t)msg.data[0] << 8) | msg.data[1];
        float temp_fet = temp_fet_CAN * 0.1;

        int16_t temp_motor_CAN = ((int16_t)msg.data[2] << 8) | msg.data[3];
        float temp_motor = temp_motor_CAN * 0.1;

        int16_t current_in_CAN = ((int16_t)msg.data[4] << 8) | msg.data[5];
        float current_in = current_in_CAN * 0.1;

        int16_t pid_pos_now_CAN = ((int16_t)msg.data[6] << 8) | msg.data[7];
        float pid_pos_now = pid_pos_now_CAN * 0.02;

        Serial.print("VESC ID: "); Serial
.print(vesc_id);
        Serial.print(", Temp fet: "); Serial.print(temp_fet);
        Serial.print(", Temp motor: "); Serial.print(temp_motor);
        Serial.print(", Current in: "); Serial.print(current_in);
        Serial.print(", Pid Pos now: "); Serial.println(pid_pos_now);
        Serial.println("------");
        break;

    case 27: //Status 5
        int32_t tacho_value = ((int32_t)msg.data[0] << 24) |
                       ((int32_t)msg.data[1] << 16) |
                       ((int32_t)msg.data[2] << 8)  |
                       ((int32_t)msg.data[3]);

        int16_t v_in_CAN = ((int16_t)msg.data[4] << 8) | msg.data[5];
        float v_in = v_in_CAN * 0.1 ;

        Serial.print("VESC ID: "); Serial.print(vesc_id);
        Serial.print(", Tacho value: "); Serial.print(tacho_value);
        Serial.print(", Voltage in: "); Serial.println(v_in);
        Serial.println("------");
        break;
    default: break;
}}



void setup() {
  Serial.begin(115200);
  delay(1000);
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
  esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(200));

  if (err == ESP_OK) {
    convert_data(msg);
  }
}

