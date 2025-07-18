#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>
#include <TouchScreen.h>

struct coords {
  double x_mm;
  double y_mm;
  double z;
};

double mapf(double x, double in_min, double in_max, double out_min, double out_max);
void screen_init();
bool check_detected();
coords get_coords();

#endif