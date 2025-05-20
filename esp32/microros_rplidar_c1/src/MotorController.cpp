// #include "MotorController.h"

// MotorController::MotorController(int pwmPin)
//  : _pwm(pwmPin) { }

// bool MotorController::begin() {
//   // pinMode can fail silently, so we just configure
//   const int channel = 0;
//   const int freq = 50; // 50 Hz
//   const int resolution = 12; // 12 bits
//   ledcSetup(channel, freq, resolution);
//   // ledcSetup(0, 50, 12); 
//   pinMode(_pwm, OUTPUT);
//   ledcAttachPin(_pwm, channel);
//   ledcWrite(channel, 1500); // stop the motor
//   delay(3500); // wait for 3.5 seconds !!
//   return true;
// }

// void MotorController::command(int8_t dir, uint8_t) { 
//   int us = 1500;
//   if (dir > 0) us = 2000; // forward
//   else if (dir < 0) us = 1000; // reverse
//   ledcWrite(0, usToDuty(us));
    
// }

#include "MotorController.h"

MotorController::MotorController(int pwmPin)
  : _pwmPin(pwmPin), _esc() {
  // Constructor initializes the PWM pin and ESC
}
  /// Call in setup(), returns false on pin-mode failure
  bool MotorController::begin() {
    _esc.setPeriodHertz(50); // Set frequency to 50 Hz
    _esc.attach(_pwmPin, _minUs, _maxUs); // Attach the ESC to the PWM pin
    Serial.println(_esc.attached() ? "ESC attached" : "ESC not attached"); 
    _esc.writeMicroseconds(_maxUs);  // Set to full reverse
    delay(3500); // Wait for ESC to initialize
    _esc.writeMicroseconds(_neutralUs);  // Set initial angle to neutral
    delay(3500);
    return _esc.attached();
  }

  /// Drive the motor: +1 forward, -1 reverse, 0 stop
  void MotorController::command(int dir) {
    // Convert direction to microseconds
    int targetUs = dir;
    int currentUs = _esc.readMicroseconds();
    const int step = 5; // Adjust step size for smoother acceleration/deceleration
    // Gradually change speed until target is reached
    while (currentUs != targetUs) {
      if (currentUs < targetUs) {
        currentUs = min(currentUs + step, targetUs);
      } else {
        currentUs = max(currentUs - step, targetUs);
      }
      _esc.writeMicroseconds(currentUs);
      delay(50); // Small delay for stability
    }
  }

