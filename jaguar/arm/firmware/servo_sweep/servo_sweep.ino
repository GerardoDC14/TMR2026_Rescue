#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include "servo_config.h"

static const int PIN_JOINT4  = 22;
static const int PIN_JOINT5  = 21;
static const int PIN_JOINT6  = 19;
static const int PIN_GRIPPER = 18;

Servo joint4, joint5, joint6, gripper;

struct ServoState { float current; float target; };
ServoState s4, s5, s6, sGR;

unsigned long lastMs = 0;
String serialBuf = "";

float fmap(float v, float a, float b, float c, float d) {
  return c + (v - a) / (b - a) * (d - c);
}

float degreeToPulse(float deg, float minPw, float maxPw) {
  return fmap(constrain(deg, 0.0f, 180.0f), 0.0f, 180.0f, minPw, maxPw);
}

float j4RadToDeg(float rad) {
  return fmap(constrain(rad, J4_MOVEIT_MIN_RAD, J4_MOVEIT_MAX_RAD),
              J4_MOVEIT_MIN_RAD, J4_MOVEIT_MAX_RAD,
              J4_SERVO_MIN_DEG,  J4_SERVO_MAX_DEG);
}

// Piecewise linear — two measured segments
float j5RadToDeg(float rad) {
  float deg = rad * (180.0f / M_PI);
  if (deg <= J5_P2_MOVEIT_DEG)
    return fmap(deg, J5_P1_MOVEIT_DEG, J5_P2_MOVEIT_DEG, J5_P1_SERVO_DEG, J5_P2_SERVO_DEG);
  return   fmap(deg, J5_P2_MOVEIT_DEG, J5_P3_MOVEIT_DEG, J5_P2_SERVO_DEG, J5_P3_SERVO_DEG);
}

float j6RadToDeg(float rad) {
  return fmap(constrain(rad, J6_MOVEIT_MIN_RAD, J6_MOVEIT_MAX_RAD),
              J6_MOVEIT_MIN_RAD, J6_MOVEIT_MAX_RAD,
              J6_SERVO_MIN_DEG,  J6_SERVO_MAX_DEG);
}

void stepToward(ServoState &s, float elapsed, Servo &srv) {
  if (fabsf(s.current - s.target) < 0.5f) { s.current = s.target; return; }
  float step = SPEED_US_PER_MS * elapsed;
  s.current = (s.target > s.current)
              ? min(s.current + step, s.target)
              : max(s.current - step, s.target);
  srv.writeMicroseconds((int)s.current);
}

void handleJson(const String &line) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, line)) return;

  if (doc.containsKey("j4"))
    s4.target  = degreeToPulse(j4RadToDeg(doc["j4"].as<float>()), J4_PULSE_MIN_US, J4_PULSE_MAX_US);
  if (doc.containsKey("j5"))
    s5.target  = degreeToPulse(j5RadToDeg(doc["j5"].as<float>()), J5_PULSE_MIN_US, J5_PULSE_MAX_US);
  if (doc.containsKey("j6"))
    s6.target  = degreeToPulse(j6RadToDeg(doc["j6"].as<float>()), J6_PULSE_MIN_US, J6_PULSE_MAX_US);
  if (doc.containsKey("gr"))
    sGR.target = degreeToPulse(constrain(doc["gr"].as<float>(), GR_SERVO_MIN_DEG, GR_SERVO_MAX_DEG),
                               GR_PULSE_MIN_US, GR_PULSE_MAX_US);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  joint4.setPeriodHertz(SERVO_FREQ_HZ);
  joint5.setPeriodHertz(SERVO_FREQ_HZ);
  joint6.setPeriodHertz(SERVO_FREQ_HZ);
  gripper.setPeriodHertz(SERVO_FREQ_HZ);

  joint4.attach (PIN_JOINT4,  J4_PULSE_MIN_US, J4_PULSE_MAX_US);
  joint5.attach (PIN_JOINT5,  J5_PULSE_MIN_US, J5_PULSE_MAX_US);
  joint6.attach (PIN_JOINT6,  J6_PULSE_MIN_US, J6_PULSE_MAX_US);
  gripper.attach(PIN_GRIPPER, GR_PULSE_MIN_US, GR_PULSE_MAX_US);

  float homeJ4 = degreeToPulse(j4RadToDeg(0.0f), J4_PULSE_MIN_US, J4_PULSE_MAX_US);
  float homeJ5 = degreeToPulse(j5RadToDeg(0.0f), J5_PULSE_MIN_US, J5_PULSE_MAX_US);
  float homeJ6 = degreeToPulse(j6RadToDeg(0.0f), J6_PULSE_MIN_US, J6_PULSE_MAX_US);
  float homeGR = degreeToPulse(GR_SERVO_MIN_DEG,  GR_PULSE_MIN_US, GR_PULSE_MAX_US);

  s4  = { homeJ4, homeJ4 };
  s5  = { homeJ5, homeJ5 };
  s6  = { homeJ6, homeJ6 };
  sGR = { homeGR, homeGR };

  joint4.writeMicroseconds ((int)homeJ4);
  joint5.writeMicroseconds ((int)homeJ5);
  joint6.writeMicroseconds ((int)homeJ6);
  gripper.writeMicroseconds((int)homeGR);

  lastMs = millis();
  Serial.println("Ready — JSON {j4,j5,j6,gr}");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) { handleJson(serialBuf); serialBuf = ""; }
    } else {
      serialBuf += c;
    }
  }

  unsigned long now = millis();
  float elapsed = (float)(now - lastMs);
  lastMs = now;

  stepToward(s4,  elapsed, joint4);
  stepToward(s5,  elapsed, joint5);
  stepToward(s6,  elapsed, joint6);
  stepToward(sGR, elapsed, gripper);

  delay(5);
}
