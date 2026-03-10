#include "pid_r.hpp"
#include <iostream>
double PID_r::pid_calculate(double dts, double in_ff, double expect,
                            double feedback) {

  if (dts != 0) {
    dts = 1.0 / dts;
  }

  exp_d = (expect - exp_old) * dts;

  if (fb_d_mode == 0) {
    fb_d = (feedback - feedback_old) * dts;
  } else {
    fb_d = fb_d;
  }

  double differential = kd_ex * exp_d - kd_fb * fb_d;

  err = expect - feedback;

  if (err > d_limit) {
    err = d_limit;
  } else if (err < -d_limit) {
    err = -d_limit;
  }

  if (std::abs(feedback - expect) < 5 || exp_old * expect < 0) {
    err_i = 0;
  }

  err_i += ki * err;

  if (err_i > limit) {
    err_i = limit;
  } else if (err_i < -limit) {
    err_i = -limit;
  }

  out = k_ff * in_ff + kp * err + differential + err_i;

  feedback_old = feedback;
  exp_old = expect;

  std::cout << "row 1:" << err << " 2:" << err_i << " 3:" << differential
            << " 4:" << out << std::endl;

  return out;
}

/* void PID_r::change_mode(std::string type)
{

    if (type == 'Yaw') {

    } else if (type == 'Line') {

    } else if (type == 'Row') {

    }

} */