#include "pid_t.hpp"

double PID_t::pid_calculate(double dts, double in_ff, uint32_t expect,
                            uint32_t feedback) {
  if (dts != 0) {
    dts = 1 / dts;
  }
  this->exp_d = (expect - this->exp_old) * dts;
  if (this->fb_d_mode == 0) {
    this->fb_d = (feedback - this->feedback_old) * dts;
  }
  /* else
  {
      this->fb_d = this->fb_d;
  } */

  double differential = this->kd_ex * this->exp_d - this->kd_fb * this->fb_d;
  this->err = expect - feedback;

  if (this->err > this->d_limit) {
    this->err = this->d_limit;
  } else if (this->err < -this->d_limit) {
    this->err = -this->d_limit;
  }

  this->err_i += this->ki * this->err;

  if (this->err_i > this->limit) {
    this->err_i = this->limit;
  } else if (this->err_i < -this->limit) {
    this->err_i = -this->limit;
  }
  this->out =
      this->k_ff * in_ff + this->kp * this->err + differential + this->err_i;
  this->feedback_old = feedback;
  this->exp_old = expect;
  return this->out;
}