#include <Arduino.h>
#include "InverseKinematics.h"
#include "Robot.h"
#include "Screen.h"
#include "Controllers.h"

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
  move_line(30, 30, 20, 2);

  move_ellipse(25, 25, 50, 2);
}
