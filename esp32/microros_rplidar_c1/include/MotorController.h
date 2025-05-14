#pragma once

#include <Arduino.h>

class MotorController {
public:
  MotorController(int in1Pin, int in2Pin, int pwmPin);

  /// Call in setup(), returns false on pin-mode failure
  bool begin();

  /// Drive the motor: +1 forward, -1 reverse, 0 stop
  void command(int8_t dir, uint8_t speed = 200);

private:
  int _in1, _in2, _pwm;
};
