#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double tuneKpid_parameter(double cte, double avgErr, int cur_K);
  double set_tuneKpid_flags(int kp, int ki, int kd); 
  double initialize_tuneKpid_var(int index); 
  void setPID_parameters(double K_in[]);
 /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }


 private:
  // PID Errors
  double p_error;
  double i_error;
  double d_error;

  // PID Coefficients
  double Kp;
  double Ki;
  double Kd;

  //double tot_err;
  // Flag, if filter is initialized
  bool is_initialized;
  
//-----Start of code to try twiddle-----//
int tune_Kpid[3]  = {0,0,0};
double kp_err[3]  = {0,0,0};
double sum_dKpid = tol;
double dKp[3] = {0.02, 0.0005, 0.03}; //{0.5, 0.5, 0.5};
double Kpid[3] = {0.25, 0.0, 0.75}; //initial values Kp, Ki, Kd //0.2, 0.004, 3.0
bool step1 = true;
double tol = 0.01;    // tol=0.001
double tuneErr = 0.0;
int tuneCount = 0;
//-----End of code to try twiddle-----//
};

#endif  // PID_H