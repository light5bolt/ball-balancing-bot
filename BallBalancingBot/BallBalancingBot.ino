#include <Arduino.h>
#include "InverseKinematics.h"
#include "MotorControl.h"
#include "Screen.h"
#include "PIDControllers.h"


// Global variable declaration
bool enable_serial_output = true;

// Function definitions
void muteAllSerialOutput() {
  enable_serial_output = false;
}

void enableAllSerialOutput() {
  enable_serial_output = true;
}


void setup() {
  // Initialization functions
  Serial.begin(115200);
  muteAllSerialOutput();
  if (enable_serial_output) Serial.println("----------------NEW RUN-----------------"); 
  screen_init();
  motor_init();
  home_motors();
  go_home();
  delay(1000);

  // Headers for CSV file reading
  //Serial.println("Time(ms),Ball_X(mm),Ball_Y(mm),Target_X(mm),Target_Y(mm)");
}

void loop() {
  pid_balance(0,0);
  //move_ellipse(30, 20, 20, 4);
  //move_figure8(30, 20, 4);
  //move_square(30, 20, 4);
  //move_heart(40, 20, 4);
}