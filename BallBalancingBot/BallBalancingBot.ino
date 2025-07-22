#include <Arduino.h>
#include "InverseKinematics.h"
#include "MotorControl.h"
#include "Screen.h"
#include "PIDControllers.h"

void setup() {
  Serial.begin(115200);
  Serial.println("----------------NEW RUN-----------------"); 

  screen_init();
  motor_init();

  home_motors();

  go_home();
  delay(1000);
}

void loop() {
  move_star(30, 20, 2);
  move_figure8(30, 20, 2);
  move_spiral(40, 30, 2);
  move_heart(40, 20, 2);
}
