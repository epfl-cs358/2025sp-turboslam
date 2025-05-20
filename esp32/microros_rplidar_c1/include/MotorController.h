// #pragma once
// #include <Arduino.h>
// #include <ESP32Servo.h>

// class MotorController {
// public:
//   MotorController(int pwmPin);

//   /// Call once in setup(); returns false on failure
//   bool begin();

//   /// dir = +1 forward, -1 reverse, 0 stop/brake
//   void command(int8_t dir);

// private:
//   int   _pwmPin;
//   Servo _esc;
//   // int  _ledcChannel;
//   // int  _freqHz    = 50;    // 50Hz = 20 ms frame
//   // int  _resolution = 12;   // 12-bit duty
//   int  _neutralUs = 1500;  // 1.5 ms = stop/brake
//   int  _minUs     = 1000;  // 1 ms  = full reverse
//   int  _maxUs     = 2000;  // 2 ms  = full forward
// };

/* MotorController.h */
#pragma once
#include <ESP32Servo.h>

static constexpr uint16_t NEUTRAL_US = 1500;
static constexpr uint16_t MAX_FORWARD_US = 2000;
static constexpr uint16_t MAX_REVERSE_US = 1000;

// number of microseconds to wait between each step
static constexpr uint16_t RAMP_STEP_US = 5; 
// number of microseconds to wait between each step
static constexpr uint16_t UPDATE_PERIOD_MS = 50; // 50ms between each step

class MotorController {
public:
  MotorController(int pwmPin);

  /// Call once in setup(); returns false on failure
  bool begin();

  void setTargetUs(uint16_t us);

  void setTargetPercent(float p);

  void update();

  uint16_t currentUs() const { return _currentUs;}

private:
  int   _pwmPin;
  Servo _esc;
  uint16_t _targetUs = NEUTRAL_US; // target pulse width in microseconds
  uint16_t _currentUs = NEUTRAL_US; // current pulse width in microseconds
};
