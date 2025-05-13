// RplidarC1.h
#ifndef RPLIDAR_C1_H
#define RPLIDAR_C1_H

#include <Arduino.h>
#include <sensor_msgs/msg/laser_scan.h>

#define BUF_SIZE 3000

// LaserScan constants
#define MIN_ANGLE 0.0                  // Start angle of the scan [rad]
#define MAX_ANGLE (2.0 * M_PI)         // End angle of the scan [rad]
#define MIN_RANGE 0.05                 // Minimum valid range [m]
#define MAX_RANGE 10.0                 // Maximum valid range [m]
#define ANGLE_INCREMENT (M_PI / 180.0) // 1 degree increment in radians

class RplidarC1
{
public:
    RplidarC1();
    void begin();
    // /// Set the onboard motor controller’s PWM duty.
    // /// pwm duty is a 16-bit value (0–1023 nominal),
    // /// e.g. ~330 for ≈10 Hz, ~660 for ≈20 Hz.
    // void setMotorPWM(uint16_t pwm);
    void resetLidar();
    void startLidar();
    int uartRx();
    void processFrame(int num_points);
    sensor_msgs__msg__LaserScan scan_msg;

private:
    uint8_t DataBuffer[BUF_SIZE];
    uint16_t dataIndex;
    bool pointAlign;
    bool frameActive;
    bool frameComplete;
    float ranges[600];
    float angles[600];
    float intensities[600];

    struct stScanDataPoint_t
    {
        uint8_t quality;
        uint8_t angle_low;
        uint8_t angle_high;
        uint8_t distance_low;
        uint8_t distance_high;
    };
};

#endif
