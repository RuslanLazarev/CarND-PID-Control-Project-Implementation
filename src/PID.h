#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Coefficients for Twiddle
  */ 
  std::vector<double> dp;
  std::vector<double> p;
  double total_err;
  double best_err;
  int iter;

  /*
  * Constructor
  */
  PID();

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

  double ComputeSteer();
  double ComputeThrottle(double throttleMax);


  /*
  * Twiddle algorithm as per lesson by Sebastian Thrun
  */
  std::vector<double> Twiddle(double cte);
};

#endif /* PID_H */
