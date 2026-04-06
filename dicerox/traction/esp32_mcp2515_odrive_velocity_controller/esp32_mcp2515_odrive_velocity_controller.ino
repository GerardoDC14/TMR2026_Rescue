#include "odrive_velocity_controller_app.h"

odrive_velocity::ControllerApp g_odriveVelocityApp;

void setup() {
  g_odriveVelocityApp.begin();
}

void loop() {
  g_odriveVelocityApp.update();
}
