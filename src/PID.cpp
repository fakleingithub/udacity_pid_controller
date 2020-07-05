#include "PID.h"
#include <numeric>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients 
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
   * Adapt PID coefficients during twiddle-algorithm optimization
   */
  
  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;
}


