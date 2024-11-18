#pragma once

#include <Arduino.h>

#include <functional>

namespace PID {

class PIDController {
 public:
  enum pid_calculate_type_t {
    PID_CALC_TYPE_INCREMENTAL, /*!< Incremental PID control */
    PID_CALC_TYPE_POSITIONAL,  /*!< Positional PID control */
  };
  struct pid_ctrl_config_t {
    float kp;                        // PID Kp parameter
    float ki;                        // PID Ki parameter
    float kd;                        // PID Kd parameter
    float max_output;                // PID maximum output limitation
    float min_output;                // PID minimum output limitation
    float max_integral;              // PID maximum integral value limitation
    float min_integral;              // PID minimum integral value limitation
    pid_calculate_type_t calc_type;  // PID calculation type
  };
  PIDController(const pid_ctrl_config_t cfg);
  void update_param(const pid_ctrl_config_t config);
  float compute(const float error);
  void reset();

 private:
  pid_calculate_type_t calc_type_;  // PID calculation type
  float kp_;                        // PID Kp parameter
  float ki_;                        // PID Ki parameter
  float kd_;                        // PID Kd parameter
  float previous_err1_ = 0;         // e(k-1)
  float previous_err2_ = 0;         // e(k-2)
  float integral_err_ = 0;          // Sum of error
  float last_output_ = 0;           // PID output in last control period

  float max_output_;    // PID maximum output limitation
  float min_output_;    // PID minimum output limitation
  float max_integral_;  // PID maximum integral value limitation
  float min_integral_;  // PID minimum integral value limitation

  std::function<float(const float)> calc_func;
  float calc_incremental(const float error);
  float calc_positional(const float error);
};

}  // namespace PID