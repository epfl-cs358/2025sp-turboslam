#include "MotorController.h"

MotorController::MotorController(int pwmPin)
 : _pwm(pwmPin) { }

bool MotorController::begin() {
  // pinMode can fail silently, so we just configure
  const int channel = 0;
  const int freq = 50; // 50 Hz
  const int resolution = 12; // 12 bits
  ledcSetup(channel, freq, resolution);
  // ledcSetup(0, 50, 12); 
  pinMode(_pwm, OUTPUT);
  ledcAttachPin(_pwm, channel);
  ledcWrite(channel, usToDuty(1500)); // stop the motor
  delay(3500); // wait for 3.5 seconds !!
  return true;
}

int MotorController::usToDuty(int us) {
  // Convert microseconds to duty cycle
  float period = 1000000.0f / 50; // 20 ms
  float fraction = us / period;
  return int(fraction * ((1 << 12) - 1));
}

void MotorController::command(int8_t dir, uint8_t) { 
  int us = 1500;
  if (dir > 0) us = 2000; // forward
  else if (dir < 0) us = 1000; // reverse
  ledcWrite(0, usToDuty(us));
    
}
