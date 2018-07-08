#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Bool option to display errors
  */

  bool debug;
  /*
  * Errors
  */

  double p_error;
  double i_error;
  double d_error;
  double i_error_limit;

  /*
  * Coefficients
  */ 
  double kp;
  double ki;
  double kd;

  /*
  * Constructor
  */
  PID(): debug(false) {};

  PID(bool is_debug): debug(is_debug) {};

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
