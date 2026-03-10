#include <stdint.h>
#ifndef __PID_H
#define __PID_H

class PID_t {
public:
  double d_limit = 100.0;
  double limit = 10.0;
  double err = 0.0;
  double exp_old = 0.0;
  double feedback_old = 0.0;
  double fb_d = 0.0;
  double fb_d_ex = 0.0;
  double exp_d = 0.0;
  double err_i = 0.0;
  double ff = 0.0;
  double pre_d = 0.0;
  double out = 0.0;
  int fb_d_mode = 0;
  double kp = 3.45;
  double ki = 0.1;
  double kd_ex = 0.0;
  double kd_fb = 0.0;
  double k_ff = 0.8;
  double inc_hz = 0.0;
  double pid_calculate(double dts, double in_ff, uint32_t expect,
                       uint32_t feedback);
};
#endif