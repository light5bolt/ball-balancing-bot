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
  pid_balance(0,0);
  //test_motor_speed();
}
