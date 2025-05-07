#include "UltraSonicSensor.h"

UltraSonicSensor::UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin)
    : trigPin(triggerPin), echoPin(echoPin) {}

bool UltraSonicSensor::begin() {
    pinMode(trigPin, INPUT);
    pinMode(echoPin, OUTPUT);

    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view = 0.26; // Approximate for HC-SR04
    range_msg.min_range = 0.02;
    range_msg.max_range = 4.00;
    range_msg.header.frame_id.data = (char *)"ultrasonic_frame";
    range_msg.header.frame_id.size = strlen(range_msg.header.frame_id.data);
    range_msg.header.frame_id.capacity = range_msg.header.frame_id.size + 1;

    return true;
}

float UltraSonicSensor::readDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms
    float distance = duration * 0.0343 / 2.0;

    if (duration == 0) {
        return -1.0; // Timeout or no object
    }
    
    return distance / 100.0; // Convert to meters
}
