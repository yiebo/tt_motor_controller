#include <Arduino.h>

#include "config.hpp"
#include "rotary_encoder.hpp"
#include "tb6612fng.hpp"
#include "controller_timer.hpp"

tt_motor::Encoder encoder(tt_motor::enc_a_cfg);
tb6612fng::Controller controller(tt_motor::config);

QueueHandle_t queue = xQueueCreate(10, sizeof(int));
controller_timer::Timer timer(PID::timer_cfg, queue);
int64_t count = 0;

void setup() {
  Serial.begin(9600);
  encoder.set_glitch_filter(1);
  encoder.start();

  controller.start();

  timer.register_callback([]() { return ++count; });
  timer.start();
}

void task_loop() {
  int recv_info;
  while (true) {
    // wait for data from queue
    xQueueReceive(queue, &recv_info, portMAX_DELAY);
    // motor_ctrl.pulse_in_one_period = recv_info.pulse_info.pulse_cnt;
    // if (motor_ctrl.cfg.pid_enable) {
    //   /* Calculate the output by PID algorithm according to the pulse.
    //    * Pid_output here is the duty of MCPWM */
    //   motor_ctrl.error = motor_ctrl.expt - motor_ctrl.pulse_in_one_period;
    //   pid_compute(motor_ctrl.pid, motor_ctrl.error, &motor_ctrl.pid_output);
    // }

    // /* Set the MCPWM duty */
    // brushed_motor_set_duty(motor_ctrl.pid_output);
  }
}

void loop() {
  delay(100);
  controller.channel_a_.command(25.0);
  // Serial.println("Set duty");
  Serial.print(controller.channel_a_.get_duty());
  Serial.print(" - ");
  Serial.print(count);
  Serial.print(" - ");
  Serial.println(encoder.position());
}