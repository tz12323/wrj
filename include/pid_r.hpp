#ifndef __PID_R_H
#define __PID_R_H
#include <string>

class PID_r {
public:
  // ===== 限幅参数 =====
  double d_limit = 60.0;
  double limit = 25.0;

  // ===== pid 数据值 =====
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

  // ===== pid 参数值 =====
  int fb_d_mode = 0;
  double kp = 1.0;
  double ki = 0.1;
  double kd_ex = 0.0;
  double kd_fb = 0.0;
  double k_ff = 0.9;
  double inc_hz = 0.0;

  // ===== 构造函数（不做任何初始化）=====
  PID_r() = default;

  // // ===== 模式切换（保持原样）=====
  // void change_mode(std::string type);

  // ===== PID 计算 =====
  double pid_calculate(double dts, double in_ff, double expect,
                       double feedback);
};

#endif