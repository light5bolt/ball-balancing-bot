#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "InverseKinematics.h"
#include <Arduino.h>
#include <math.h>
#include "Screen.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

// Global stepper motor objects
extern AccelStepper motorA;
extern AccelStepper motorB;
extern AccelStepper motorC;
extern MultiStepper motors;


// Function prototypes
void motor_init();
long int angle_to_steps(double angle);
double steps_to_angle(int steps);
void home_motors();
void go_home();
void move_to_angle(double theta_deg, double phi_deg, double h, double speed[3]);
void speed_controller(double speed[3]);
void test_motor_speed();

#endif