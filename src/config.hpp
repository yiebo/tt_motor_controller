#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include "driver/mcpwm.h"
#include <driver/pcnt.h>

#include "custom_mcpwm.hpp"
#include "rotary_encoder.hpp"
#include "tb6612fng.hpp"

namespace tt_motor {

// ENCODER
constexpr int16_t kLines = 14;
constexpr int16_t kReductionRatio = 45;
constexpr int16_t kMaxPositions = 4 * kLines * kReductionRatio;  // quadrature

constexpr tb6612fng::Controller::Config config = {
    .unit = MCPWM_UNIT_0,
    .timer = MCPWM_TIMER_0,
    .pwm_pair = MCPWM_OUTPUT_0,
    .gpio_stby = GPIO_NUM_5,
    .frequency = 1000U,
    .cfg_a =
        {
            .generator = MCPWM_GEN_A,
            .gpio_pwm = GPIO_NUM_21,
            .gpio_in1 = GPIO_NUM_18,
            .gpio_in2 = GPIO_NUM_19,
        },
    .cfg_b =
        {
            .generator = MCPWM_GEN_B,
            .gpio_pwm = GPIO_NUM_35,
            .gpio_in1 = GPIO_NUM_32,
            .gpio_in2 = GPIO_NUM_33,
        },
};

constexpr Encoder::PcntConfig enc_a_cfg = {
    .unit = PCNT_UNIT_0,
    .gpio_a = GPIO_NUM_16,  // change
    .gpio_b = GPIO_NUM_17,  // change
};

// constexpr Encoder::PcntConfig enc_b_cfg = {
//     .unit = PCNT_UNIT_1,
//     .gpio_a = GPIO_NUM_16,  // change
//     .gpio_b = GPIO_NUM_17,  // change
// };

}  // namespace tt_motor