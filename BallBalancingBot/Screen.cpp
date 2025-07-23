#include "Screen.h"

// Touchscreen wiring (lettering on the ribbon pin is the underside, red wire goes to 14)
#define YP 16  // Must be an analog pin
#define XM 15  // Must be an analog pin
#define YM 14
#define XP 17


// Touch screen calibration (adjust if needed)
#define TS_MINX 60
#define TS_MAXX 963
#define TS_MINY 80
#define TS_MAXY 947

// Screen dimensions
#define SCREEN_WIDTH_MM 167.0
#define SCREEN_HEIGHT_MM 135.5

// Pressure thresholds (adjust if needed)
#define MINPRESSURE .000000000001
#define MAXPRESSURE 500

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 200);  // 300 = ohms of touchscreen
TSPoint currentPoint;

//map command but can return floating point values
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//initializes touchscreen pins
void screen_init() {
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
}

//checks if touchscreen detects ball
bool check_detected() {
  currentPoint = ts.getPoint();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  return (currentPoint.x > 0 && currentPoint.y < 1023);
}

//returns coordinates of the ball's position
coords get_coords() {
  static double last_x = 0;
  static double last_y = 0;
  coords p;

  currentPoint = ts.getPoint();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // Map raw values to mm with (0,0) in center
  double x_mm = mapf(currentPoint.x, TS_MINX, TS_MAXX, SCREEN_WIDTH_MM / 2, -SCREEN_WIDTH_MM / 2);
  double y_mm = mapf(currentPoint.y, TS_MINY, TS_MAXY, SCREEN_HEIGHT_MM / 2, -SCREEN_HEIGHT_MM / 2);


  if (x_mm == 94.60 || y_mm == -79.63) {
    p.x_mm = last_x;
    p.y_mm = last_y;
    p.z = 0;
    return p;
  }

  // All good — save and return
  last_x = x_mm;
  last_y = y_mm;
  p.x_mm = x_mm;
  p.y_mm = y_mm;
  p.z = currentPoint.z;

  return p;
}
