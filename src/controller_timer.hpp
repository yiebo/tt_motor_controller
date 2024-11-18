#pragma once

#include <functional>

#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace controller_timer {

class Timer {
 public:
  struct config {
    timer_group_t timer_group; /* Timer Group number */
    timer_idx_t timer_idx;     /* Timer ID */
    uint64_t period_ms;        /* Motor control period, unit in ms */
  };

  Timer(const config cfg, const QueueHandle_t queue);

  ~Timer();

  void register_callback(std::function<int()> fn);

  void set_period(const uint64_t period);

  void start();

  void stop();

 private:
  timer_group_t timer_group_;
  timer_idx_t timer_idx_;

  QueueHandle_t queue_;

  int pulse_cnt_;
  bool IRAM_ATTR isr_callback();
  std::function<int()> callback_;
};

}  // namespace timer
