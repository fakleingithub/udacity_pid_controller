#include "PID.h"
#include <numeric>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 * test
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  pidinit = false;
  
  diff_cte = 0.0;
  prev_cte = 0.0;
  int_cte = 0.0;  
  
  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;
}

void PID::Adapt(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  
  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;
}


