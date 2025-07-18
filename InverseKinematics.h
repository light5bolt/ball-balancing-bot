#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include <math.h>

// Structure to hold triangle points
struct Triangle {
  double VA[3];
  double VB[3];
  double VC[3];
};

// Structure to hold IK solver results
struct IKResult {
  double alpha;  // elbow angle in degrees
  double theta2; // shoulder angle in degrees
};

//Structure to hold calculated angles for each motor. Combines calculate_triangle and ik_solver functions
struct CalculatedAngles {
  double thetaA;
  double thetaB;
  double thetaC;
};

// Function declarations
Triangle calculate_triangle(double theta_deg, double phi_deg, double d, double h);
IKResult ik_solver(double base_point[3], double top_point[3], double e, double f);
CalculatedAngles get_angles(double theta, double phi, double h);

// Matrix multiplication helper functions
void matrix_multiply_3x3(const double A[3][3], const double B[3][3], double result[3][3]);
void matrix_vector_multiply_3x3(const double matrix[3][3], const double vector[3], double result[3]);
double matrix_row_vector_multiply(const double matrix_row[3], const double vector[3]);

#endif