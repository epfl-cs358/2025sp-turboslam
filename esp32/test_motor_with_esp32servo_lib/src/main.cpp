#include <ESP32Servo.h>

Servo esc;
#define ESC_PIN 15  // Any PWM-capable GPIO

void increase() {
    Serial.println("Forward...");
    esc.writeMicroseconds(1600);
    delay(3000);
}

void neutral() {
    Serial.println("Neutral...");
    esc.writeMicroseconds(1500);
    delay(3000);
}

void decrease(int dir) {
    Serial.println("Reverse...");
    esc.writeMicroseconds(1400);
    delay(3000);
}



void setup() {
    Serial.begin(115200);

    // Attach ESC to pin, 1000–2000us pulse range, 50Hz
    esc.setPeriodHertz(50); // Optional, defaults to 50 Hz
    esc.attach(ESC_PIN, 1000, 2000); // pin, min pulse, max pulse
    if (!esc.attached()) {
        Serial.println("ERROR : ESC is not attached");
    }

    Serial.println("Arming ESC...");
    neutral();
    Serial.println("ESC Armed.");
}


void loop() {
    forward();
    neutral();
    reverse();
    neutral();
}