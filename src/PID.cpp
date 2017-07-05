#include "PID.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <stdio.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  //Initialize errors
  PID::p_error = 0.0;
  PID::i_error = 0.0;
  PID::d_error = 0.0;
  error = 0.0;
  int_cte = 0.0;
  prev_cte = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, PIDPhase phase) {
  tune_param = PIDParam::KP;  //First parameter to tune
  pid_phase = phase;          //Assumes an untuned controller and centered car at startup
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  tuning_pass= 1;
  tolerance = 0.001;
  best_error = numeric_limits<double>::max();
  N = 400;
  dp_Kp = 0.5;
  dp_Ki = 0.25;
  dp_Kd = 0.25;
  run_count = 0;
  prev_control = 0.0;
  prev_ts = 0;

  /**
   * TEST MODE
  pid_phase = PIDPhase::RUN;
  PID::Kp = 0.25;
  PID::Ki = 0.001;
  PID::Kd = 3.00;
   */
}

/**
 * Twiddle control parameters.
 * Since The run loop is controlled by the JSON server, Twiddle must be split up into passes
 * tuning_pass = 1 : increase parameter by current increment
 * tuning_pass = 2 : if error improved, increase increment and switch to next parameter.
 *                    Else decrease parameter.
 * tuning_pass = 3 : if error improved, increase increment. Else reset parameter to previous and decrease increment.
 *                    Switch to next parameter
 *
 * After each pass, reset the simulator before starting the next tuning run.
 */
void PID::UpdateParameter(PIDParam next_param, double *delta, double *parameter ) {
  switch(tuning_pass)
  {
    case 1:
      (*parameter) += *delta;
      tuning_pass = 2;
      break;
    case 2:
      if (error < best_error)
      {
        best_error = error;
        (*delta) *= 1.1;
        tune_param = next_param; //switch to the next parameter.
        tuning_pass = 1;
      } else {
        (*parameter) -= 2 * (*delta);
        tuning_pass = 3;
      }
      break;
    case 3:
      if (error < best_error)
      {
        best_error = error;
        (*delta) *= 1.1;
      } else {
        (*parameter) += (*delta);
        (*delta) *= 0.9;
      }
      tune_param = next_param; //switch to the next parameter.
      tuning_pass = 1;
      break;
  }
}

void PID::UpdateError(double cte) {
  p_error = cte*cte;
  error += p_error;
  i_error = i_error + p_error;
  d_error = p_error - prev_cte;
  run_count++;
  double dp = dp_Kp + dp_Kd + dp_Ki;

  //Stop Twiddle and just RUN
  if (dp <= tolerance && tune_param == KP && tuning_pass == 1)
  {
    pid_phase = PIDPhase::RUN;
    std::cout << "RUN MODE" << std::endl;
  }

  //Tuning only happens in TWIDDLE phase
  if (pid_phase == PIDPhase::TWIDDLE)
  {
    // Run Twiddle once every N loops
    if (run_count >= N)
    {
      error += cte*cte;
      if (run_count >= 2*N)
      {
        error = error/N;
        printf("TotalError: %6.4f \t", error);
        //Update the current tuning parameter before running the controller
        switch(tune_param)
        {
          case KP:
            UpdateParameter(KD, &dp_Kp, &Kp);
            break;
          case KD:
            UpdateParameter(KI, &dp_Kd, &Kd);
            break;
          case KI:
            UpdateParameter(KP, &dp_Ki, &Ki);
            break;
        }
        pid_phase = PIDPhase::RESET; //Message to main loop to reset simulator
        error = 0.0;
        run_count = 0;
      }
    }
  } //End of TWIDDLE

}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

/**
 * Reset Twiddle controls to start
 */
void PID::Reset() {
  pid_phase = PIDPhase::TWIDDLE;
  int_cte = 0;
  prev_cte = 0;
  error = 0;
  run_count = 0;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

double PID::Control(double cte, double speed, double angle) {
  double dt = 1;
  if (speed > 0.01)
    dt = 1/speed; // Use speed as a proxy for time
  int_cte += (cte*dt);
  //double control = -Kp*cte - Ki*int_cte - Kd*(cte - prev_cte);
  double control = -Kp*cte - Ki*int_cte - Kd*(cte - prev_cte)/dt;
  printf("CTE: %6.4f \t Control: %6.4f \t Speed: %6.4f \t", cte, control, speed);
  printf(" Errors: Kp: %6.4f, Ki: %6.4f, Kd: %6.4f \t", cte, int_cte, (cte-prev_cte)/dt);
  //Reduce control to 75% max if too large
  if (control > 1)
    control = 1.0;
  else if (control < -1)
    control = -1.0;
  prev_cte = cte;
  printf("Params: Kp: %6.4f, Ki: %7.6f, Kd: %6.4f \t", Kp, Ki, Kd);
  printf(" Deltas: Kp: %4.2f, Ki: %4.2f, Kd: %4.2f \t", dp_Kp, dp_Ki, dp_Kd);
  return control;
}
