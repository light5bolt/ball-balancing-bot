#ifndef LEG_H
#define LEG_H

#include "Robot.h"
#include "InverseKinematics.h"
#include <Arduino.h>
#include <math.h>
#include "Screen.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

void pid_balance(double setpoint_x, double setpoint_y, bool print_output = 1);
void move_to_point(double setpoint_x, double setpoint_y, unsigned long delay, bool print_output = 1);
void move_line(double rx, double ry, double speed, int repeat=0);
void move_ellipse(double rx, double ry, double speed, double repeat=0);


#endif