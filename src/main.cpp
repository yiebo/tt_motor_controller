#include <Arduino.h>

#include "rotary_encoder.hpp"
#include "tb6612fng.hpp"
#include "config.hpp"

tt_motor::Encoder encoder(tt_motor::enc_a_cfg);
tb6612fng::Controller controller(tt_motor::config);
int64_t count = 0;

void setup() {
  Serial.begin(9600);
  encoder.set_glitch_filter(1);
  encoder.start();
  Serial.println("Encoder initialized");
  controller.start();
  Serial.println("Controller initialized");
}

void loop() {
  delay(100);
  controller.channel_a_.command(25.0);
  // Serial.println("Set duty");
  Serial.print(controller.channel_a_.get_duty());
  Serial.print(" - ");
  Serial.println(encoder.position());
}