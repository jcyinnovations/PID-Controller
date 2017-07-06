#ifndef PID_H
#define PID_H

/**
 * ID of the parameter to tune
 */
enum PIDParam {
  KP, KD, KI
};

/**
 * RESET - after a tuning run, center the car before restarting twiddle
 * TWIDDLE - tune the POD parameters
 * RAMP - Accelerate to training speed prior to twiddle
 * RUN - run with established parameters
 */
enum PIDPhase {
  RESET, TWIDDLE, RAMP, RUN
};

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double int_cte;
  double prev_cte;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * TWiddle control parameters
   */
  double best_error;
  double error;
  PIDParam tune_param;  //Which parameter to tune
  PIDPhase pid_phase;   //Phase of execution
  int tuning_pass;
  int run_count;        //How many control steps to run before running Twiddle
  double tolerance;     //Target error to stop tuning
  int N;                //loop count
  double prev_control;
  long prev_ts;         //timestamp used to calculate dt

  /**
   * Change in parameters per twiddle loop
   */
  double dp_Kp;
  double dp_Ki;
  double dp_Kd;

  /**
   * Best parameters found
   */

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
  void Init(double Kp, double Ki, double Kd, PIDPhase phase);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /**
   * Controller output
   */
  double Control(double cte, double speed, double angle);

  /**
   * Set the Twiddle back to start for next iteration
   */
  void Reset();

  /**
   * Run Twiddle to tune the controller
   */
  void Twiddle(double cte, double speed, double angle);

private:
  /**
   * Tuning algorithm for updating a PID parameter
   */
  void UpdateParameter(PIDParam next_step, double *delta, double *parameter);
};

#endif /* PID_H */
