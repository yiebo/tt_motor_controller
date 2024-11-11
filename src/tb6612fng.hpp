#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

#include <memory>
#include <optional>

#include "custom_mcpwm.hpp"
#include "driver/mcpwm.h"
#include "rotary_encoder.hpp"

namespace tb6612fng {

class Controller {
 public:
  struct ChannelConfig {
    mcpwm_generator_t generator;  // MCPWMXA or MCPWMXB
    gpio_num_t gpio_pwm;
    gpio_num_t gpio_in1;
    gpio_num_t gpio_in2;
  };
  struct Config {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_pwm_pair_t pwm_pair;
    gpio_num_t gpio_stby;
    uint32_t frequency;  // MCPWM output frequency
    ChannelConfig cfg_a;
    ChannelConfig cfg_b;
  };

  class Channel {
   public:
    Channel(const Controller &controller, const ChannelConfig cfg);
    void stop();
    void command(float duty_cycle);
    float get_duty() const;

   private:
    const Controller &controller_;
    mcpwm_io_signals_t io_signal_;
    mcpwm_generator_t generator_;
    gpio_num_t gpio_pwm_;
    gpio_num_t gpio_in1_;
    gpio_num_t gpio_in2_;
  };

  Channel channel_a_;
  Channel channel_b_;

  Controller(const Config cfg);

  void start();

  void stop();

 private:
  mcpwm_unit_t unit_;
  mcpwm_timer_t timer_;
  mcpwm_pwm_pair_t pwm_pair_;
  gpio_num_t gpio_stby_;
};

}  // namespace tb6612fng