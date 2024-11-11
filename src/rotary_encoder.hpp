#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>

#include <array>

// https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/peripherals/pcnt.html?highlight=pcnt

namespace tt_motor {

class Encoder {
 public:
  struct PcntConfig {
    pcnt_unit_t unit;
    gpio_num_t gpio_a;
    gpio_num_t gpio_b;
  };
  Encoder(const PcntConfig cfg);

  ~Encoder();

  esp_err_t set_glitch_filter(uint32_t max_glitch_us);

  esp_err_t start();

  esp_err_t stop();

  esp_err_t reset();

  int16_t position() const;

  int64_t count() const;

 private:
  static bool pcnt_isr_service_installed_;
  static _lock_t isr_service_install_lock_;

  pcnt_unit_t pcnt_unit_;
  gpio_num_t gpio_a_;
  gpio_num_t gpio_b_;
  int64_t overlow_count_ = 0;

  void overflow_handler();

  void configure_full_quad();
};
}  // namespace tt_motor