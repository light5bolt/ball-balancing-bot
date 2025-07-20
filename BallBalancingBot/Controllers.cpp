#include <Arduino.h>
#include <math.h>
#include "Controllers.h"

// PID Constants
#define kp .8  //.8
#define ki .2 //.15
#define kd .09 //.09
#define ks 10            // For speed controller
#define max_output 83.5  // defines maximum output for PID controller (in this case it means the maximum distance from current point to setpoint for both x and y)
#define max_angle 20     // defines maximum tilt angle that PID controller can output (this will be mapped with max_output so that if max_output is returned, it translates to max_angle)

//Variables needed for PID control
double error[2] = { 0, 0 }, error_prev[2], integ[2] = { 0, 0 }, deriv[2] = { 0, 0 }, output[2], output_angles[2];  // error/error_prev for P, integ for I, deriv for D
double speed[3] = { 0, 0, 0 }, speed_prev[3];                                                                      // an initialization of the stepper motor speeds

//Runs X and Y PID Controllers to balance ball at a specific point
void pid_balance(double setpoint_x, double setpoint_y) {

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
      last_detected_time = t;

      for (int i = 0; i < 2; i++) {
        //if (i == 1) continue; // Skip X axis during Y tuning
        //if (i == 0) continue; // Skip Y axis during X tuning

        error_prev[i] = error[i];
        error[i] = (i == 0) * (setpoint_y - p.y_mm) + (i == 1) * (setpoint_x - p.x_mm);  //Calculates error based on ball position
        integ[i] += error[i] * dt;                                                       // Calculates integral term by summing up error * dt
        integ[i] = constrain(integ[i], -50, 50);                                         // Constrains integral values to prevent windup
        deriv[i] = (error[i] - error_prev[i]) / dt;                                      //Calculates derivative term by finding rate of change in the given interval
        deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                    // Simple check to eliminate nonreal or infinite numbers, as derivative is being divided by a number that could be zero

        output[i] = kp * error[i] + ki * integ[i] + kd * deriv[i];                                    // Forms output by adding P, I, and D terms. Currently an arbitrary value
        output_angles[i] = constrain(output[i], -max_output, max_output) * (max_angle / max_output);  // scales down PID output and maps it to an angle
      }

      // SPIKE FILTERING
      static double prev_output_angles[2] = {output_angles[0], output_angles[1]};  // Store previous angles for spike detection
      double spike_threshold = 12.0; 
      static int spike_count[2] = { 0, 0 };  // Track consecutive spikes

      for (int i = 0; i < 2; i++) {
        if (abs(output_angles[i] - prev_output_angles[i]) > spike_threshold) {
          spike_count[i]++;
          if (spike_count[i] < 3) {                    // Allow up to 3 consecutive spikes
            output_angles[i] = prev_output_angles[i];  // Reject spike
            Serial.println((String) "Spike detected on axis " + i +  ": " + output_angles[i] + " -> " + prev_output_angles[i]);
          }
          // After 3 spikes, allow the change (might be a valid large correction)
        } else {
          spike_count[i] = 0;  // Reset counter on normal operation
        }
        prev_output_angles[i] = output_angles[i];
      }
      
      // calculates requires speeds for each motors
      speed_controller(speed);

      unsigned long move_start = millis();
      while (millis() - move_start < 20) {
        // for each calculated PID value, run the move command for 10ms
        move_to_angle(-output_angles[0], output_angles[1], 80, speed);
      }

      Serial.println((String) "X angle: " + output_angles[1] + ". Y angle: " + output_angles[0] + ". Speed of motor A: " + speed[0]);
    }

    else {
      // Check if ball has been undetected for 3 seconds (3000 milliseconds)
      if (t - last_detected_time >= 3000) {
        integ[0] = integ[1] = 0;  // Reset integral terms after 3 seconds
        go_home();
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
