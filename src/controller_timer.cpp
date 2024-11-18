#include "driver/timer.h"

#include <Arduino.h>

// #include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "controller_timer.hpp"

#define MOTOR_CTRL_TIMER_DIVIDER (16)  //  Hardware timer clock divider
// convert counter value to seconds
#define MOTOR_CTRL_TIMER_SCALE (TIMER_BASE_CLK / MOTOR_CTRL_TIMER_DIVIDER)

namespace controller_timer {

Timer::Timer(const Timer::config cfg, const QueueHandle_t queue)
    : timer_group_(cfg.timer_group),
      timer_idx_(cfg.timer_idx),
      queue_(queue),
      pulse_cnt_(0) {
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = MOTOR_CTRL_TIMER_DIVIDER,
  };  // default clock source is APB
  timer_init(timer_group_, timer_idx_, &config);
  timer_set_counter_value(timer_group_, timer_idx_, 0);

  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(timer_group_, timer_idx_,
                        cfg.period_ms * MOTOR_CTRL_TIMER_SCALE / 1000);
  timer_enable_intr(timer_group_, timer_idx_);

  timer_isr_callback_add(
      timer_group_, timer_idx_,
      [](void *arg) { return static_cast<Timer *>(arg)->isr_callback(); }, this,
      0);
}

Timer::~Timer() { timer_deinit(timer_group_, timer_idx_); }

void Timer::register_callback(const std::function<int()> fn) { callback_ = fn; }

void Timer::set_period(const uint64_t period) {
  timer_set_alarm_value(timer_group_, timer_idx_,
                        period * MOTOR_CTRL_TIMER_SCALE / 1000);
}

void Timer::start() { timer_start(timer_group_, timer_idx_); }

/**
 * @brief Pause the timer and clear the counting value
 */
void Timer::stop() {
  /* stop the timer */
  timer_pause(timer_group_, timer_idx_);
  timer_set_counter_value(timer_group_, timer_idx_, 0);
}

/**
 * @brief Every `period_ms` calls `callback` and sends result to queue.
 */
bool IRAM_ATTR Timer::isr_callback() {
  BaseType_t high_task_awoken = pdFALSE;
  pulse_cnt_ = callback_();

  /* Now just send the event data back to the main program task */
  xQueueSendFromISR(queue_, &pulse_cnt_, &high_task_awoken);

  return high_task_awoken ==
         pdTRUE;  // return whether we need to yield at the end of ISR
}

}  // namespace timer
