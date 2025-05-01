#include "AS5600Encoder.h"
#include <Wire.h>
TwoWire AS5600Wire(1);

AS5600Encoder::AS5600Encoder() {
    angle_msg.data = 0;
}

void AS5600Encoder::begin() {
    AS5600Wire.begin(21, 22); 
    encoder.begin(4); 
    encoder.setDirection(AS5600_CLOCK_WISE);
    angle_msg.data = 0;
}

void AS5600Encoder::update() {  
    angle_msg.data = encoder.rawAngle();
}
