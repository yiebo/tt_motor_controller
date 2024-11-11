#pragma once

#include <Arduino.h>

namespace PID {
class PID {
 public:
  enum ControlType {
    POSITION,
    SPEED,
  };
  struct PIDConfig {
    ControlType ctrl_type;
    float k_p_;
    float k_i_;
    float k_d_;
  };

  PID(const PIDConfig cfg);

  void set_PID(const float k_p, const float k_i, const float k_d);

  void clear();

 private:
  float k_p_;
  float k_i_;
  float k_d_;

  /* history variables */
  float previous_err1_;            // e(k-1)
  float previous_err2_;            // e(k-2)
  float integral_err_;             // Sum of error
  float last_output_;              // PID output in last control period

  /* limitation */
  float max_output_limit_;         // PID maximum output limitation
  float min_output_limit_;         // PID minimum output limitation
  float max_integral_limit_;       // PID maximum integral value limitation
  float min_integral_limit_;       // PID minimum integral value limitation

  /* PID calculation function type (Set by user) */
  ControlType ctrl_type;      // PID calculation type

};

}  // namespace PID