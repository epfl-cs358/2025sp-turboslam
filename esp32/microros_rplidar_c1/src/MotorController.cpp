#include "MotorController.h"

MotorController::MotorController(int pwmPin)
 : _pwm(pwmPin) { }

bool MotorController::begin() {
  // pinMode can fail silently, so we just configure
  ledcSetup(0, 50, 12); 
  pinMode(_pwm, OUTPUT);
  ledcAttachPin(_pwm, 0);
  ledcWrite(0, usToDuty(1500)); // stop the motor
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
