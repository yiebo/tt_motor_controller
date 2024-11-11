#include "rotary_encoder.hpp"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <soc/rtc.h>

#include "config.hpp"

static const char *TAG = "rotary_encoder";

namespace tt_motor {

bool Encoder::pcnt_isr_service_installed_ = false;
_lock_t Encoder::isr_service_install_lock_;

Encoder::Encoder(const PcntConfig cfg)
    : pcnt_unit_(cfg.unit), gpio_a_(cfg.gpio_a), gpio_b_(cfg.gpio_b) {

  pinMode(this->gpio_a_, INPUT_PULLDOWN);
  pinMode(this->gpio_b_, INPUT_PULLDOWN);

  this->configure_full_quad();

  // PCNT pause and reset value
  pcnt_counter_pause(this->pcnt_unit_);
  pcnt_counter_clear(this->pcnt_unit_);

  // register interrupt handler in a thread-safe way
  _lock_acquire(&isr_service_install_lock_);
  if (!pcnt_isr_service_installed_) {
    ESP_ERROR_CHECK(pcnt_isr_service_install(0));
    // make sure pcnt isr service won't be installed more than one time
    pcnt_isr_service_installed_ = true;
  }
  _lock_release(&isr_service_install_lock_);

  pcnt_isr_handler_add(
      this->pcnt_unit_,
      [](void *arg) { static_cast<Encoder *>(arg)->overflow_handler(); }, this);

  pcnt_event_enable(this->pcnt_unit_, PCNT_EVT_H_LIM);
  pcnt_event_enable(this->pcnt_unit_, PCNT_EVT_L_LIM);
}

Encoder::~Encoder() {};

void Encoder::configure_full_quad() {
  pcnt_config_t pcnt_config;
  // Configure channel 0
  pcnt_config.pulse_gpio_num = this->gpio_a_;
  pcnt_config.ctrl_gpio_num = this->gpio_b_;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = this->pcnt_unit_;

  pcnt_config.pos_mode = PCNT_COUNT_DEC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;

  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = kMaxPositions;
  pcnt_config.counter_l_lim = -kMaxPositions;

  ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

  // Configure channel 1
  pcnt_config.pulse_gpio_num = this->gpio_b_;
  pcnt_config.ctrl_gpio_num = this->gpio_a_;
  pcnt_config.channel = PCNT_CHANNEL_1;

  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));
}

esp_err_t Encoder::set_glitch_filter(uint32_t max_glitch_us) {
  max_glitch_us = max_glitch_us * rtc_clk_apb_freq_get();
  if (max_glitch_us > 1023) {
    max_glitch_us = 1023;
  }
  esp_err_t ret_code = pcnt_set_filter_value(this->pcnt_unit_, max_glitch_us);
  if (ret_code != ESP_OK) {
    return ESP_FAIL;
  }
  if (max_glitch_us > 0) {
    return pcnt_filter_enable(this->pcnt_unit_);
  }
  return pcnt_filter_disable(this->pcnt_unit_);
}

esp_err_t Encoder::start() { return pcnt_counter_resume(this->pcnt_unit_); }

esp_err_t Encoder::stop() { return pcnt_counter_pause(this->pcnt_unit_); }

esp_err_t Encoder::reset() {
  this->overlow_count_ = 0;
  return pcnt_counter_clear(this->pcnt_unit_);
}

int16_t Encoder::position() const {
  int16_t val = 0;
  pcnt_get_counter_value(this->pcnt_unit_, &val);
  return val;
}

int64_t Encoder::count() const {
  int16_t val = 0;
  pcnt_get_counter_value(this->pcnt_unit_, &val);
  return static_cast<int64_t>(val) + this->overlow_count_;
}

void Encoder::overflow_handler() {
  uint32_t status = 0;
  esp_err_t ret_code = pcnt_get_event_status(this->pcnt_unit_, &status);

  if (status & PCNT_EVT_H_LIM) {
    this->overlow_count_ += kMaxPositions;
  } else if (status & PCNT_EVT_L_LIM) {
    this->overlow_count_ += -kMaxPositions;
  }
}

}  // namespace tt_motor