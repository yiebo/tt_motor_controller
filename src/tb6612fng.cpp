#include "tb6612fng.hpp"

#include <driver/gpio.h>

#include "driver/mcpwm.h"
// #include "config.hpp"

namespace tb6612fng {

Controller::Controller(const Controller::Config cfg)
    : unit_(cfg.unit),
      timer_(cfg.timer),
      pwm_pair_(cfg.pwm_pair),
      gpio_stby_(cfg.gpio_stby),
      channel_a_(*this, cfg.cfg_a),
      channel_b_(*this, cfg.cfg_b) {
  // set stby output mode
  pinMode(gpio_stby_, OUTPUT);

  // init gpio unit, timer, gen
  mcpwm_config_t pwm_config;
  pwm_config.frequency = cfg.frequency;
  pwm_config.cmpr_a = 0.0f;  // initial duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0.0f;  // initial duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;  // up counting mode
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(unit_, timer_, &pwm_config);
}

void Controller::start() {
  // set stby level high
  channel_a_.command(0.0f);
  channel_b_.command(0.0f);
  gpio_set_level(gpio_stby_, HIGH);
}

void Controller::stop() {
  channel_a_.stop();
  channel_b_.stop();
  // set stby level low
  gpio_set_level(gpio_stby_, LOW);
}

Controller::Channel::Channel(const Controller &controller,
                             const ChannelConfig cfg)
    : controller_(controller),
      generator_(cfg.generator),
      io_signal_(get_io_signal(controller_.pwm_pair_, generator_)),
      gpio_pwm_(cfg.gpio_pwm),
      gpio_in1_(cfg.gpio_in1),
      gpio_in2_(cfg.gpio_in2) {
  // set gpio output mode
  pinMode(gpio_in1_, OUTPUT);
  pinMode(gpio_in2_, OUTPUT);
  mcpwm_gpio_init(controller_.unit_, io_signal_, gpio_pwm_);
}

void Controller::Channel::stop() {
  // Changes both input pins to a low level as to stop the motor (High
  // impedance)
  gpio_set_level(gpio_in1_, LOW);
  gpio_set_level(gpio_in2_, LOW);
}

void Controller::Channel::command(float duty_cycle) {
  if (duty_cycle >= 1) {
    // ccw
    gpio_set_level(gpio_in1_, LOW);
    gpio_set_level(gpio_in2_, HIGH);
  } else if (duty_cycle <= -1) {
    // cw
    gpio_set_level(gpio_in1_, HIGH);
    gpio_set_level(gpio_in2_, LOW);
  } else {
    // short break
    gpio_set_level(gpio_in1_, HIGH);
    gpio_set_level(gpio_in2_, HIGH);
  }
  mcpwm_set_duty(controller_.unit_, controller_.timer_, generator_,
                 abs(duty_cycle));
}

float Controller::Channel::get_duty() const {
  return mcpwm_get_duty(controller_.unit_, controller_.timer_, generator_);
}

}  // namespace tb6612fng