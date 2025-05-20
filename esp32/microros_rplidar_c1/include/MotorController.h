#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class MotorController {
public:
  MotorController(int pwmPin);

  /// Call once in setup(); returns false on failure
  bool begin();

  /// dir = +1 forward, -1 reverse, 0 stop/brake
  void command(int dir);

private:
  int   _pwmPin;
  Servo _esc;
  // int  _ledcChannel;
  // int  _freqHz    = 50;    // 50Hz = 20 ms frame
  // int  _resolution = 12;   // 12-bit duty
  int  _neutralUs = 1500;  // 1.5 ms = stop/brake
  int  _minUs     = 1000;  // 1 ms  = full reverse
  int  _maxUs     = 2000;  // 2 ms  = full forward
};
