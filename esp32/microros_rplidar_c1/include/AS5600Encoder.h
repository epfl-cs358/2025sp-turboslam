#ifndef AS5600_ENCODER_H
#define AS5600_ENCODER_H

#include <AS5600.h>
#include <Wire.h>
#include <std_msgs/msg/int32.h>

class AS5600Encoder {
public:
    AS5600Encoder();
    bool begin();
    void update();
    std_msgs__msg__Int32 angle_msg;

private:
    AS5600 encoder;
};

#endif