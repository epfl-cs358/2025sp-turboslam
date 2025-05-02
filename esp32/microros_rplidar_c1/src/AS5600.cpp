#include "AS5600Encoder.h"
#include <Wire.h>
TwoWire wire = TwoWire(1);
AS5600 AS5600Wire(&wire);

AS5600Encoder::AS5600Encoder() {
    angle_msg.data = 0;
}

bool AS5600Encoder::begin() {
    wire.begin(21, 22); 
    if (!encoder.begin(4)) {
        Serial.println("Failed to initialize AS5600");
        return false;
    }
    encoder.setDirection(AS5600_CLOCK_WISE);
    angle_msg.data = 0;
    return true;
}

void AS5600Encoder::update() {  
    angle_msg.data = encoder.rawAngle();
}
