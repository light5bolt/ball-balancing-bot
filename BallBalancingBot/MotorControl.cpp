#include "MotorControl.h"

#define ANGLE_TO_ORIGIN_A 71 // angle offset
#define ANGLE_TO_ORIGIN_B 66.75// angle offset
#define ANGLE_TO_ORIGIN_C 61.95// angle offset
#define ENA 0 // ENA pin
#define h0 87 // height of platform when motors are at zero position
#define ks 100.0 // a constant to change our proportional speed function

// setup for steppers
AccelStepper motorA(1, 3, 2);  //(driver type, STEP, DIR) Driver A
AccelStepper motorB(1, 5, 4);  //(driver type, STEP, DIR) Driver B
AccelStepper motorC(1, 7, 6);  //(driver type, STEP, DIR) Driver C
MultiStepper motors;           // Create instance of MultiStepper

long int pos[3]; // stores positions of stepper motors
extern double speed[3]; // an initialization of the stepper motor speeds
extern double speed_prev[3]; // an array to store previous speeds which we use later to change speed proportionally

//initializes motors by adding them in multistepper class
void motor_init() {
  motors.addStepper(motorA);
  motors.addStepper(motorB);
  motors.addStepper(motorC);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  delay(2000);
  digitalWrite(ENA, LOW);
}

// angle in degrees to the nearest step (rounds up)
long int angle_to_steps(double angle) {
  return round((3200 / 360)  * angle);
}

//steps to nearest angle (as a decimal)
double steps_to_angle(int steps) {
  return (360 / 3200) * steps;
}

//moves motors to position where bottom leg is parallel to the ground. this is the zero position
void home_motors() {
  motorA.setMaxSpeed(1000);
  motorB.setMaxSpeed(1000);
  motorC.setMaxSpeed(1000);

  pos[0] = angle_to_steps(ANGLE_TO_ORIGIN_A);
  pos[1] = angle_to_steps(ANGLE_TO_ORIGIN_B);
  pos[2] = angle_to_steps(ANGLE_TO_ORIGIN_C);

  //calculates position and moves all motors to the zero position
  motors.moveTo(pos);
  motors.runSpeedToPosition();

  //makes the new point the origin (zero position)
  motorA.setCurrentPosition(0);
  motorB.setCurrentPosition(0);
  motorC.setCurrentPosition(0);

  //Serial.println("Motors to zero position");
}

//goes to zero position (home). requires home_motors first.
void go_home() {
  for (int i = 0; i < 3; i++) pos[i] = 0;
  motors.moveTo(pos);
  motors.runSpeedToPosition();
}

// calculates positions to move to a specific angle, but only moves at most one step per call.
void move_to_angle(double theta_deg, double phi_deg, double h, double speed[3]) {

  //uses get_angles function to get positions required to move to position
  CalculatedAngles result = get_angles(theta_deg, phi_deg, h);
  pos[0] = angle_to_steps(result.thetaA);
  pos[1] = angle_to_steps(result.thetaB);
  pos[2] = angle_to_steps(result.thetaC);

  //sets max speed
  motorA.setMaxSpeed(speed[0]);
  motorB.setMaxSpeed(speed[1]);
  motorC.setMaxSpeed(speed[2]);

  //sets acceleration proportional to a calculated speed
  motorA.setAcceleration(speed[0] * 50);
  motorB.setAcceleration(speed[1] * 50);
  motorC.setAcceleration(speed[2] * 50);

  //calculates speed to each motor gets to target position at the same time
  motors.moveTo(pos);
  for (int i = 0; i < 10; i++) {
    if (motors.run()) break; // Stop if all motors reached target
  }
}

//Calculates proportional motor speeds for all three motors whenever function gets called.
void speed_controller(double speed[3]) {
  static double current_pos[3]; // Variable for current positions

  for (int i = 0; i < 3; i++) {
    speed_prev[i] = speed[i];
    current_pos[i] = (i==0) * motorA.currentPosition() + (i==1) * motorB.currentPosition() + (i==2) * motorC.currentPosition(); // Finds current position of motor
    speed[i] = abs(current_pos[i] - pos[i]) * ks; // Position error * speed gain
    speed[i] = constrain(speed[i], speed_prev[i] - 300, speed_prev[i] + 300); // Constrains speed so there aren't any sudden jumps. Essentially substituting for our D term in PID.
    speed[i] = constrain(speed[i], 0, 1300);
  }
}

// Debugger function to test motor speeds
void test_motor_speed() {
    motorA.setMaxSpeed(4000);
    motorA.setAcceleration(10000);
    motorA.moveTo(500); // Move 1000 steps
    
    unsigned long start = millis();
    while (motorA.distanceToGo() != 0) {
        motorA.run();
    }
    unsigned long duration = millis() - start;
    
    Serial.println("Time to move 500 steps: " + String(duration) + "ms");
    Serial.println("Actual speed: " + String(500.0 / (duration / 500.0)) + " steps/sec");
}