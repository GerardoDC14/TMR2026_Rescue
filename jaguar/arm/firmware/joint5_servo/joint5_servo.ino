#include <ESP32Servo.h>

Servo servoJ5;

const int PIN_J5    = 21;
const int MIN_PW_J5 = 800;
const int MAX_PW_J5 = 2100;

void setup() {
  Serial.begin(115200);
  delay(500);

  servoJ5.setPeriodHertz(330);
  servoJ5.attach(PIN_J5, MIN_PW_J5, MAX_PW_J5);

  servoJ5.write(90);
  Serial.println("Ready — send 0-180");
}

void loop() {
  if (!Serial.available()) return;

  int deg = Serial.readStringUntil('\n').toInt();
  deg = constrain(deg, 0, 180);
  servoJ5.write(deg);
  Serial.println("-> " + String(deg));
}
