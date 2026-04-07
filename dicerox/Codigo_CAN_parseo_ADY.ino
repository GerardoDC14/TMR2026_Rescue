//Autor: Adriana González

#include "driver/twai.h" // Librería CAN (TWAI) de la ESP32

// VARIABLES CAN

#define TWAI_TX GPIO_NUM_22
#define TWAI_RX GPIO_NUM_21

// CONTROL DE RAMPA 

static int32_t rpm_cmd = 0;          // valor actual (con rampa)
static int32_t rpm_target = 0;       // valor deseado

static const int32_t RPM_SLEW = 200; // cambio máximo por ciclo
static const uint32_t CONTROL_PERIOD_MS = 20; // 50 Hz

static uint32_t last_control_time = 0;


void convert_data(const twai_message_t &msg){ // Recibe un mensaje CAN
    uint32_t id = msg.identifier; // Guarda el ID completo del mensaje

    uint8_t vesc_id = id & 0xFF; // Extrae el ID del VESC (último Byte)
    uint8_t cmd = (id >> 8) & 0xFF; // Extrae el comando (segundo Byte)

    switch(cmd){ // Elige el Status a parsear en base al comando

    case 9:{ //Status 1
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
        break;}

    case 14:{ // Status 2
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
        break;}

    case 15:{ //Status 3
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
        break;}

    case 16: {//Status 4

        int16_t temp_fet_CAN = ((int16_t)msg.data[0] << 8) | msg.data[1];
        float temp_fet = temp_fet_CAN * 0.1;

        int16_t temp_motor_CAN = ((int16_t)msg.data[2] << 8) | msg.data[3];
        float temp_motor = temp_motor_CAN * 0.1;

        int16_t current_in_CAN = ((int16_t)msg.data[4] << 8) | msg.data[5];
        float current_in = current_in_CAN * 0.1;

        int16_t pid_pos_now_CAN = ((int16_t)msg.data[6] << 8) | msg.data[7];
        float pid_pos_now = pid_pos_now_CAN * 0.02;

        Serial.print("VESC ID: "); Serial.print(vesc_id);
        Serial.print(", Temp fet: "); Serial.print(temp_fet);
        Serial.print(", Temp motor: "); Serial.print(temp_motor);
        Serial.print(", Current in: "); Serial.print(current_in);
        Serial.print(", Pid Pos now: "); Serial.println(pid_pos_now);
        Serial.println("------");
        break;}

    case 27:{ //Status 5
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
        break;}
    default: {break;}
}}

esp_err_t set_duty_cycle(uint8_t id, float duty){
    uint8_t cmd = 0; // Comando VESC : "Set duty cycle"
    twai_message_t msg = {};    // Inicializa mensaje limpio

    // Validación básica
    if (duty > 1.0) duty = 1.0;
    if (duty < -1.0) duty = -1.0;

    // Se define la longitud de mensaje, su ID y el comando
    msg.identifier = ((uint32_t)cmd << 8) | id;
    msg.extd = 1;
    msg.data_length_code = 4;

    // Se escala el valor de duty cycle
    int32_t duty_int = (int32_t)(duty * 100000.0f);

    // Se divide el mensaje en 4 bytes
    msg.data[0] = (duty_int >> 24) & 0xFF;
    msg.data[1] = (duty_int >> 16) & 0xFF;
    msg.data[2] = (duty_int >> 8) & 0xFF;
    msg.data[3] = (duty_int) & 0xFF;

    // Transmisión del mensaje por CAN, espera 100 ms
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}


esp_err_t set_rpm(uint8_t id, int32_t rpm, int32_t polos_motor){
    int32_t erpm = (int32_t)(rpm * (polos_motor / 2.0f));; // Cálculo de ERPM
    uint8_t cmd = 3;  // Comando VESC : "Set RPM"
    twai_message_t msg = {}; // Inicializa mensaje limpio

    // Se define la longitud de mensaje, su ID y el comando
    msg.identifier = ((uint32_t)cmd << 8) | id;
    msg.extd = 1;
    msg.data_length_code = 4;

    // Se divide el mensaje en 4 bytes
    msg.data[0] = (erpm >> 24) & 0xFF;
    msg.data[1] = (erpm >> 16) & 0xFF;
    msg.data[2] = (erpm >> 8) & 0xFF;
    msg.data[3] = (erpm) & 0xFF;

    // Transmisión del mensaje por CAN, espera 100 ms
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}


void setup() {

  // Se inicializa la comunicación serial
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting CAN sniffer...");

  // Se configura el CAN
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX, TWAI_RX, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Velocidad CAN 500 KBITS al igual que VESC
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  Serial.print("driver_install: ");
  Serial.println(err);

  err = twai_start();
  Serial.print("twai_start: ");
  Serial.println(err);

  // Se reporta si la conexión fue exitosa
  if (err == ESP_OK) {
    Serial.println("Listening at 500 kbps...");
  }
}

void loop() {
  twai_message_t msg;

  // Se recibe el mensaje CAN
  esp_err_t err1 = twai_receive(&msg, pdMS_TO_TICKS(200)); 

  if (err1 == ESP_OK) { // Si fue exitoso, se convierte el valor y se realiza el parseo
    convert_data(msg);
  }

  // Se envía el mensaje CAN 

  //NOTA: Modificar los valores 
  esp_err_t err2 = set_duty_cycle(1, 0.2); // Ej. ID=1, 20% duty


  if (err2 == ESP_OK) {
    Serial.println("Duty enviado correctamente");
  } else {
    Serial.print("Error enviando duty: ");
    Serial.println(err2);
  }

  //NOTA: Modificar los valores 
  esp_err_t err3 = set_rpm(2, 1000, 14)); // Ej. ID=2, 1000 RPM, 14 polos

  uint32_t now = millis();

  if (now - last_control_time >= CONTROL_PERIOD_MS) {
      last_control_time = now;

      // RPM objetivo
      rpm_target = 2000;   // <- Ejemplo 2000 RPM

      // Aplicar rampa
      rpm_cmd = apply_ramp(rpm_target, rpm_cmd, RPM_SLEW);

      // Enviar valor con la función
      esp_err_t err3 = set_rpm(1, rpm_cmd, 14);

      //Confirmar que se haya enviado correctamente
      if (err3 == ESP_OK) {
        Serial.print("RPM enviada: ");
        Serial.println(rpm_cmd);
      } else {
        Serial.print("Error enviando RPM: ");
        Serial.println(err3);
      }
    }
    delay(10);
  }

    delay(1000); // importante para no saturar el bus
  }

