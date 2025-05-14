#include "MotorController.h"

MotorController::MotorController(int in1Pin, int in2Pin, int pwmPin)
 : _in1(in1Pin), _in2(in2Pin), _pwm(pwmPin) { }

bool MotorController::begin() {
  // pinMode can fail silently, so we just configure
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_pwm, OUTPUT);
  // Optionally test by writing LOW and reading back?
  digitalWrite(_in1, LOW);  
  digitalWrite(_in2, LOW);
  analogWrite(_pwm, 0);
  return true;
}

void MotorController::command(int8_t dir, uint8_t speed) {
  switch (dir) {
    case  1:  // forward
      digitalWrite(_in1, HIGH);
      digitalWrite(_in2, LOW);
      analogWrite(_pwm, speed);
      break;
    case -1:  // reverse
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, HIGH);
      analogWrite(_pwm, speed);
      break;
    default:  // stop
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, LOW);
      analogWrite(_pwm, 0);
      break;
  }
}
