#pragma once
#include "driver/mcpwm.h"
/*
  The outputs are organized into three pairs.
  Within a pair they are labeled “A” and “B”
  and each driven by a submodule called an “Generator”
*/
enum mcpwm_pwm_pair_t {
  MCPWM_OUTPUT_0,
  MCPWM_OUTPUT_1,
  MCPWM_OUTPUT_2,
  MCPWM_OUTPUT_MAX,
};

static mcpwm_io_signals_t get_io_signal(const mcpwm_pwm_pair_t pwm_pair,
                                        const mcpwm_generator_t opr) {
  uint8_t io_signal =
      static_cast<uint8_t>(pwm_pair) * 2 + static_cast<uint8_t>(opr);
  return static_cast<mcpwm_io_signals_t>(io_signal);
}