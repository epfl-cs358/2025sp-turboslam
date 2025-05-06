#include "DMS15.h"
#include <Arduino.h>

DMS15::DMS15(int pin) : servoPin(pin) {}

bool DMS15::begin() {
    servo.attach(servoPin);
    Serial.printf("Servo attached to pin %d\n", servoPin);
    return servo.attached(); 
}

void DMS15::tiltLidar(float angleMin, float angleMax, unsigned long T_ms) {
    static unsigned long startTime = millis(); 
    unsigned long now = millis();
    float elapsed = now - startTime;

    float phase = ((int)elapsed % T_ms) / (float)T_ms * 2 * PI;

    float amplitude = (angleMax - angleMin) / 2.0;
    float mid = (angleMax + angleMin) / 2.0;
    float angle = mid + amplitude * sin(phase);

    currentAngle = angle;
    servo.write(angle);
}

void DMS15::setAngle(int angle) {
    angle = constrain(angle, 0, 180); 
    servo.write(angle);
    Serial.printf("Servo angle set to %d\n", angle);
}

int DMS15::getAngle() const {
    return currentAngle;
}




