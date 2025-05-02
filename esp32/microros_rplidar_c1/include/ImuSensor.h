#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <ICM_20948.h>
#include <sensor_msgs/msg/imu.h>

extern sensor_msgs__msg__Imu imu_msg;

class ImuSensor {
public:
    bool begin();
    void readAndUpdate();
    sensor_msgs__msg__Imu& getMsg() { return imu_msg; }
};

extern ImuSensor imuSensor;

#endif
