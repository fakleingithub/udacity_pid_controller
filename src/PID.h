#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();
  /**
  * PID Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double diff_cte;
  double prev_cte;
  double int_cte;  

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  bool pidinit;

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);
  
  void Adapt(double Kp_, double Ki_, double Kd_);
  
  void Twiddle(double Kp_);

 private:

};

#endif  // PID_H