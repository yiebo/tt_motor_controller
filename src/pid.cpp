#include "pid.hpp"

#include <Arduino.h>

namespace PID {

PIDController::PIDController(const pid_ctrl_config_t cfg)
    : calc_type_(cfg.calc_type),
      kp_(cfg.kp),
      ki_(cfg.ki),
      kd_(cfg.kd),
      max_output_(cfg.max_output),
      min_output_(cfg.min_output),
      max_integral_(cfg.max_integral),
      min_integral_(cfg.min_integral) {
  if (calc_type_ == PID_CALC_TYPE_INCREMENTAL) {
    calc_func = [this](const float error) {
      return this->calc_incremental(error);
    };
  } else {
    // PID_CALC_TYPE_POSITIONAL
    calc_func = [this](const float error) {
      return this->calc_positional(error);
    };
  }
}

float PIDController::compute(const float error) { return calc_func(error); }

float PIDController::calc_positional(const float error) {
  float output = 0;
  /* Add current error to the integral error */
  integral_err_ += error;
  /* If the integral error is out of the range, it will be limited */
  integral_err_ = constrain(integral_err_, min_integral_, max_integral_);

  /* Calculate the pid control value by location formula */
  /* u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki */
  output = error * kp_ + (error - previous_err1_) * kd_ + integral_err_ * ki_;

  /* If the output is out of the range, it will be limited */
  output = constrain(output, min_output_, max_output_);

  /* Update previous error */
  previous_err1_ = error;

  return output;
}

float PIDController::calc_incremental(const float error) {
  float output = 0;

  /* Calculate the pid control value by increment formula */
  /* du(k) = (e(k)-e(k-1))*Kp + (e(k)-2*e(k-1)+e(k-2))*Kd + e(k)*Ki */
  /* u(k) = du(k) + u(k-1) */
  output = (error - previous_err1_) * kp_ +
           (error - 2 * previous_err1_ + previous_err2_) * kd_ + error * ki_ +
           last_output_;

  /* If the output is beyond the range, it will be limited */
  output = constrain(output, min_output_, max_output_);

  /* Update previous error */
  previous_err2_ = previous_err1_;
  previous_err1_ = error;

  /* Update last output */
  last_output_ = output;

  return output;
}

void PIDController::update_param(const pid_ctrl_config_t cfg) {
  calc_type_ = cfg.calc_type;
  kp_ = cfg.kp;
  ki_ = cfg.ki;
  kd_ = cfg.kd;

  max_output_ = cfg.max_output;
  min_output_ = cfg.min_output;
  max_integral_ = cfg.max_integral;
  min_integral_ = cfg.min_integral;

  reset();

  if (calc_type_ == PID_CALC_TYPE_INCREMENTAL) {
    calc_func = [this](const float error) {
      return this->calc_incremental(error);
    };
  } else {
    // PID_CALC_TYPE_POSITIONAL
    calc_func = [this](const float error) {
      return this->calc_positional(error);
    };
  }
}

void PIDController::reset() {
  integral_err_ = 0;
  previous_err1_ = 0;
  previous_err2_ = 0;
  last_output_ = 0;
}
}  // namespace PID