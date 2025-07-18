#ifndef LEG_H
#define LEG_H

#include "Robot.h"
#include "InverseKinematics.h"
#include <Arduino.h>
#include <math.h>
#include "Screen.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

void pid_balance(double setpoint_x, double setpoint_y);


#endif