#include "SunPID.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
    : min_val_(min_val), max_val_(max_val), kp_(kp), ki_(ki), kd_(kd) {}

double PID::Compute(float setpoint, float measured_value) {
  double error;
  double pid;

  // setpoint is constrained between min and max to prevent pid from having too
  // much error
  error = setpoint - measured_value;
  //优化抗积分饱和测试
  if (ki_ * integral_ <= 0.5 * max_val_ && ki_ * integral_ >= 0.5 * min_val_) {
    integral_ += error;
  }
  
  derivative_ = error - prev_error_;

  if (setpoint == 0 && error == 0) {
    integral_ = 0;
  }

  pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
  prev_error_ = error;

  return constrain(pid, min_val_, max_val_);
}

void PID::UpdateTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
