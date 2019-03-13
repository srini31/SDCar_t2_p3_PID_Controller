#include "PID.h"
#include<iostream>
#include<math.h>
using std::cout;
using std::endl;

// Complete the PID class. You may add any additional desired functions.

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID coefficients (and errors, if needed)
  //Kp = 0.2; Ki = 0.004; Kd = 3.0; //tau_p, tau_d, tau_i = 0.2, 3.0, 0.004
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  //initialize the errors to zero
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  is_initialized = true;
  std::cout << "done initializing." << std::endl;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte.
    d_error = cte - p_error; //differential error
    p_error = cte; //proportional error
    i_error += cte; //integral error
}

double PID::TotalError() {
  // Calculate and return the total error
  double steer = -1.0*Kp*p_error -  Ki*i_error - Kd*d_error;
  return steer;  // Add your total error calc here!
}

//-----Start of code to try twiddle-----//
double PID::set_tuneKpid_flags(int kp_flag, int ki_flag, int kd_flag) { 
  tune_Kpid[0] = kp_flag;
  tune_Kpid[1] = ki_flag; 
  tune_Kpid[2] = kd_flag;
}

double  PID::initialize_tuneKpid_var(int index) {
  kp_err[index] = 0; 
  sum_dKpid = tol;  //sum_dKp[index] = 0; 
  step1 = true;
  tuneErr = 0.0;
  tuneCount = 0;
}

/* Trial of a variation of twiddle 
update Kp, Ki, Kd one at a time, 
check the squared error every 4 iterations (simulator call back counts)
compare to total cteError -- (may not be right as CTE error changes drastically 
during turns and its almost zero on straight road)
update the dKp or Kp parameters accordingly and try to follow twiddle logic 
*/
double PID::tuneKpid_parameter(double cte, double avgErr, int cur_Kidx) { //, bool tune_Kp
  int i = cur_Kidx; 
  if(i >=0 && tune_Kpid[i] == 1) {
    if(tuneCount == 0) {
        Kpid[i] += dKp[i]; //add some val to parameter
    }
    tuneCount++; //will be reset for each K based on resetCount in main
    tuneErr += pow(cte, 2);//actually should start at tuneCount = 1
    //first time update Kpid[i], run 2 times, collect error, compare to total error
    if(tuneCount == 4) {
      double curTuneErr = tuneErr/4.0;
      //run 4 iterations and then compare error to totalError
      kp_err[i] = curTuneErr;
      std::cout << "tc 4 - avgErr: "<<avgErr<<" kp_err[i]: "<< kp_err[i] << std::endl;
      if(curTuneErr <= avgErr) {
        dKp[i] *= 1.1;
        tuneErr = 0;
      } else {
        Kpid[i] -= 2*dKp[i];
        tuneErr = 0;
      }
    } else if (tuneCount == 8) {
      double curTuneErr = tuneErr/4.0;
      std::cout << "tc 8 - curTuneErr: "<<curTuneErr<<" kp_err[i]: "<< kp_err[i] << std::endl;
      if(curTuneErr <= kp_err[i]) {
        dKp[i] *= 1.1;
      } else {
        Kpid[i] += dKp[i];
        dKp[i] *= 0.9;
      }
    } // else if (tuneCount == 8)
    std::cout << "..Kpid[i]: "<< Kpid[i]<<" dKp[i]: "<<dKp[i] <<" tuneCount: "<<tuneCount<<std::endl;
    setPID_parameters(Kpid); //update the parameters 
  } // if(i >=0 && tune_Kpid[i] == 1)
} //double PID::tuneKpid_parameter(double cte)

void PID::setPID_parameters(double K_in[]) {
  this->Kp = K_in[0];
  this->Ki = K_in[1];
  this->Kd = K_in[2];
  std::cout << " Kp: " << this->Kp << " Ki: " << this->Ki << " Kd: " << this->Kd <<std::endl;
}
/*
double PID::cqe_SqError(double cte) {
  tot_err+= cte**2;
  return tot_err;
}
double PID::getPID_parameters() {
  double K_out[3] = {Kp, Ki, Kd};
  return K_out;
} */
//-----End of code to try twiddle-----//
