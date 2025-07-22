#include <Arduino.h>
#include <math.h>
#include "PIDControllers.h"

// PID Constants
#define kp .8  //.8
#define ki .2 //.2
#define kd .09 //.09
#define kv 0.05 //.05
#define kp_adj .4 //.4
#define ki_adj .25 //.25
#define kd_adj .23 //.23
#define max_output 83.5  // max X distance away from the center
#define max_angle 12.5   // max tilt angle

//Variables needed for PID control
double error[2] = { 0, 0 }, error_prev[2], integ[2] = { 0, 0 }, deriv[2] = { 0, 0 }, output[2], output_angles[2];  // error/error_prev for P, integ for I, deriv for D
double speed[3] = { 0, 0, 0 }, speed_prev[3];  // an initialization of the stepper motor speeds

// variables for velocity damping
double ball_vel[2] = {0,0}, p_prev[2] = {0,0};
double predict_time = 0.3; 

//Runs X and Y PID Controllers to balance ball at a specific point. Set print_output to 0 to turn off serial monitor prints every run
void pid_balance(double setpoint_x, double setpoint_y, bool print_output) {

  static unsigned long t_prev = 0;              // Records previous time (uses static so the variable gets remembered through each loop iteration)
  static unsigned long last_detected_time = 0;  // Track when ball was last detected
  unsigned long t = millis();                   // Records current time using millis() (which counts total time since the program started)
  double dt = (t - t_prev) / 1000.0;            // Converts milliseconds to seconds

  // Handles first run or unreasonable time gaps
  if (t_prev == 0 || dt > .075) {
    t_prev = t;
    //Serial.println((String) "dt: " + dt + ". Time gap too large or first run. Resetting timing.");
    return;  // Ends function
  }

  // Only runs if minimum sample time has passed (0.02 seconds or 20 milliseconds)
  //Remember, a higher sample size will reduce max speed as it will only call the run function once per sample
  if (dt >= 0.01) {
    bool detected = check_detected();  //checks if ball is detected
    coords p = get_coords();           // retrieves ball's position


    if (detected) {
    // Output data for plotting (every 10th iteration to avoid overwhelming)
    static int plot_counter = 0;
    if (plot_counter % 10 == 0 && print_output) {
      // Format for Arduino Serial Plotter
      Serial.print("Ball_X:");
      Serial.print(p.x_mm);
      Serial.print(",");
      Serial.print("Ball_Y:");
      Serial.print(p.y_mm);
      Serial.print(",");
      Serial.print("Target_X:");
      Serial.print(setpoint_x);
      Serial.print(",");
      Serial.print("Target_Y:");
      Serial.println(setpoint_y);
      
      // Alternative: CSV format for external plotting
      // Serial.print(millis());
      // Serial.print(",");
      // Serial.print(p.x_mm);
      // Serial.print(",");
      // Serial.println(p.y_mm);
    }
    plot_counter++;

      last_detected_time = t;

      // Predictive velocity control
      ball_vel[0] = (p.y_mm - p_prev[0]) / (dt*50);
      ball_vel[1] = (p.x_mm - p_prev[1]) / (dt*50);

      p_prev[0] = p.y_mm;
      p_prev[1] = p.x_mm;


      for (int i = 0; i < 2; i++) {
        //if (i == 1) continue; // Skip X axis during Y tuning
        //if (i == 0) continue; // Skip Y axis during X tuning

        error_prev[i] = error[i];
        double error_current = (i == 0) ? (p.y_mm - setpoint_y) : (p.x_mm - setpoint_x);  //Calculates error based on ball position
        double v = constrain(ball_vel[i], -1000, 1000); // chooses ball velocity from earlier depending on axis
        error[i] = error_current; 
        integ[i] += error[i] * dt;                                                       // Calculates integral term by summing up error * dt
        integ[i] = constrain(integ[i], -50, 50);                                         // Constrains integral values to prevent windup
        deriv[i] = (error[i] - error_prev[i]) / dt;                                      //Calculates derivative term by finding rate of change in the given interval
        deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                    // Simple check to eliminate nonreal or infinite numbers, as derivative is being divided by a number that could be zero
        //deriv[i] = constrain(deriv[i], -55, 55); // Constrains derivative, which prevents random spikes

        if (abs(error[i]) < 25) {
          output[i] = kp_adj * error[i] + ki_adj * integ[i] + kd_adj * deriv[i];
        }
        else {
          output[i] = kp * error[i] + ki * integ[i] + kd * deriv[i] - kv*v; // Forms output by adding P, I, and D terms. Currently an arbitrary value
        }

        output_angles[i] = constrain(output[i], -max_output, max_output) * (max_angle / max_output);  // scales down PID output and maps it to an angle
        //Serial.println((String) "P: " + (kp * error[i]) + " I: " + (ki * integ[i]) + " D: " + (kd * deriv[i]) + " V: " + (kv * v));
        //Serial.println((String) "error[i]: " + error[i] + " .error_prev[i]: " + error_prev[i] + " .dt: " + dt);
      }

      // SPIKE FILTERING
      // static double prev_output_angles[2] = {output_angles[0], output_angles[1]};  // Store previous angles for spike detection
      // double spike_threshold = 7.0; 
      // static int spike_count[2] = { 0, 0 };  // Track consecutive spikes

      // for (int i = 0; i < 2; i++) {
      //   if (abs(output_angles[i] - prev_output_angles[i]) > spike_threshold) {
      //     spike_count[i]++;
      //     if (spike_count[i] < 3) {
      //       Serial.println((String) "Spike detected on axis " + i +  ": " + output_angles[i] + " -> " + prev_output_angles[i]);  
            
      //       // Allow up to 3 consecutive spikes
      //       output_angles[i] = prev_output_angles[i];  // Reject spike
            
      //     }
      //     // After 3 spikes, allow the change (might be a valid large correction)
      //   } else {
      //     spike_count[i] = 0;  // Reset counter on normal operation
      //   }
      //   prev_output_angles[i] = output_angles[i];
      // }
      
      // calculates requires speeds for each motors
      speed_controller(speed);

      unsigned long move_start = millis();
      while (millis() - move_start < 20) {
        // for each calculated PID value, run the move command for 10ms
        move_to_angle(output_angles[0], -output_angles[1], 80, speed);
      }

      if (print_output) Serial.println((String) "X angle: " + output_angles[1] + ". Y angle: " + output_angles[0] + ". Speed of motor A: " + speed[0]);
    }

    else {
      // Check if ball has been undetected for 3 seconds (3000 milliseconds)
      if (t - last_detected_time >= 3000) {
        integ[0] = integ[1] = 0;  // Reset integral terms after 3 seconds
        move_to_angle(0,0,80, speed);
        //Serial.println("Ball not detected for 3 seconds - resetting integral terms");
      }
      //waits and checks again if ball is detected (eliminates error from input)
      delay(10);
      detected = check_detected();
      if (!detected) {
        //integ[0] = integ[1] = 0; // Reset integral value
        //Serial.println("Ball not detected!");
        return;
      }
    }
    t_prev = t;  // resets value of t_prev
  }
}

//uses the PID controller to move the ball to a point for a specified amount of time (in milliseconds)
void move_to_point(double setpoint_x, double setpoint_y, unsigned long delay, bool print_output) {
  unsigned long t_prev = millis();
  if (print_output) Serial.println((String) "Moving to pt: ( " + setpoint_x + " , " + setpoint_y + " )");
  while (millis() - t_prev < delay) {
    pid_balance(setpoint_x, setpoint_y, 0);
  }
}

// moves in a line pattern. rx and ry are distance from endpoint to center (0-80). speed: (10-50 ms). repeat: num of times (0 = infinite loop)
void move_line(double rx, double ry, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;

  move_to_point(-rx, -ry, 2000); // moves to start point
  // move from points (-rx, -ry) to (rx, ry)
  for (int i = 0; i < cycles; i++) {
    // calculate amount of intermediate steps
    double steps = 100; 
    double steps_x = (2 * rx) / steps;
    double steps_y = (2 * ry) / steps;

    // forward direction
    for (int j = 0; j <= steps; j++) {
      double x = -rx + (j * steps_x);
      double y = -ry + (j * steps_y);
      move_to_point(x, y, speed);
    }

    // backwards direction
    for (int j = steps; j >= 0; j--) {
      double x = -rx + (j * steps_x);
      double y = -ry + (j * steps_y);
      move_to_point(x, y, speed);
    }
  }
  Serial.println("---------------Line pattern finished--------------");
}

// Moves in an ellipse pattern. rx = radius in x direction, ry = radius in y direction, speed: (10-50ms), repeat = num of times (0 = infinite loop)
void move_ellipse(double rx, double ry, double speed, double repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  move_to_point(rx, ry, 2000); // moves to start point

  for (int i = 0; i < cycles; i++) {
    double steps = 100; // number of intermediate steps

    for (int j = 0; j <= steps; j++) {
      double t = (double)j / steps;
      double angle = 2 * PI * t;           // Full circle: 0 to 2π
      double x = rx * cos(angle);
      double y = ry * sin(angle);
      move_to_point(x, y, speed);
    }
  }
  Serial.println("---------------Ellipse pattern finished--------------");
}


// Moves in a square pattern
void move_square(double side_length, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  double half_side = side_length / 2;
  
  move_to_point(-half_side, -half_side, 2000); // Start at bottom-left corner
  
  for (int i = 0; i < cycles; i++) {
    // Bottom edge (left to right)
    move_to_point(half_side, -half_side, speed * 50);
    // Right edge (bottom to top)
    move_to_point(half_side, half_side, speed * 50);
    // Top edge (right to left)
    move_to_point(-half_side, half_side, speed * 50);
    // Left edge (top to bottom)
    move_to_point(-half_side, -half_side, speed * 50);
  }
  Serial.println("---------------Square pattern finished--------------");
}

// Moves in a figure-8 pattern
void move_figure8(double radius, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  
  move_to_point(0, 0, 2000); // Start at center
  
  for (int i = 0; i < cycles; i++) {
    double steps = 200;
    
    for (int j = 0; j <= steps; j++) {
      double t = (double)j / steps;
      double angle = 4 * PI * t; // Two full circles
      double x = radius * sin(angle);
      double y = radius * sin(2 * angle) / 2; // Creates the figure-8 shape
      move_to_point(x, y, speed);
    }
  }
  Serial.println("---------------Figure-8 pattern finished--------------");
}

// Moves in a spiral pattern (outward)
void move_spiral(double max_radius, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  
  move_to_point(0, 0, 2000); // Start at center
  
  for (int i = 0; i < cycles; i++) {
    double steps = 200;
    
    for (int j = 0; j <= steps; j++) {
      double t = (double)j / steps;
      double angle = 6 * PI * t; // Three full rotations
      double r = max_radius * t; // Gradually increase radius
      double x = r * cos(angle);
      double y = r * sin(angle);
      move_to_point(x, y, speed);
    }
  }
  Serial.println("---------------Spiral pattern finished--------------");
}

// Moves in a star pattern (5-pointed)
void move_star(double radius, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  
  for (int i = 0; i < cycles; i++) {
    // Define the 5 points of the star
    double angles[5] = {-PI/2, -PI/2 + 4*PI/5, -PI/2 + 8*PI/5, -PI/2 + 2*PI/5, -PI/2 + 6*PI/5};
    
    // Move to first point
    double x = radius * cos(angles[0]);
    double y = radius * sin(angles[0]);
    move_to_point(x, y, 1000);
    
    // Draw the star by connecting every second point
    for (int j = 0; j < 5; j++) {
      int next_point = (j + 2) % 5;
      x = radius * cos(angles[next_point]);
      y = radius * sin(angles[next_point]);
      move_to_point(x, y, speed * 100);
    }
  }
  Serial.println("---------------Star pattern finished--------------");
}

// Moves in a heart pattern
void move_heart(double size, double speed, int repeat) {
  int cycles = (repeat == 0) ? 9999 : repeat;
  
  for (int i = 0; i < cycles; i++) {
    double steps = 200;
    
    for (int j = 0; j <= steps; j++) {
      double t = (double)j / steps * 2 * PI;
      // Parametric heart equation
      double x = size * 16 * pow(sin(t), 3) / 16;
      double y = size * (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t)) / 16;
      move_to_point(x, y, speed);
    }
  }
  Serial.println("---------------Heart pattern finished--------------");
}