#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double pre_p_error;  // cte of the previous state

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * flag to indicate the first reading
   */
  bool is_first_reading;

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
  * Update the PID error variables given 'cross track error'.
  */
  void UpdateError(double cte);

  /*
  * Update the PID error variables given 'cross speed error' (i.e. reference-speed - speed).
  */
    void UpdateSpeedError(double cte_spd);

  /*
  * Calculate the total PID error for 'cross track error'.
  */
  double TotalError();

  /*
  * Calculate the total PID error for 'cross speed error'.
  */
  double TotalSpeedError();
};

#endif /* PID_H */
