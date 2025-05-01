// ImuSensor.cpp
#include <ImuSensor.h>
#include <Wire.h>


ICM_20948_I2C imu;
TwoWire ImuWire(0);  // Create a new I2C bus instance

sensor_msgs__msg__Imu imu_msg;

bool ImuSensor::begin() {
    ImuWire.begin(21, 22);  // SDA, SCL pins
    if (imu.begin(ImuWire) != ICM_20948_Stat_Ok) {
        Serial.println("Failed to initialize ICM-20948");
        return false;
    }
    Serial.println("ICM-20948 IMU initialized");

    // Initialize message frame
    imu_msg.header.frame_id.data = (char *)"imu_frame";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    return true;
}

void ImuSensor::readAndUpdate() {
    if (!imu.dataReady()) return;

    imu.getAGMT();  // Refresh internal readings

    // Fill message with data
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    imu_msg.linear_acceleration.x = imu.accX() * 9.80665 / 1000.0;
    imu_msg.linear_acceleration.y = imu.accY() * 9.80665 / 1000.0;
    imu_msg.linear_acceleration.z = imu.accZ() * 9.80665 / 1000.0;

    imu_msg.angular_velocity.x = imu.gyrX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = imu.gyrY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = imu.gyrZ() * DEG_TO_RAD;

    imu_msg.orientation.w = 0.0;  // Orientation is not calculated here
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;

    // Covariances (optional)
    for (int i = 0; i < 9; ++i) {
        imu_msg.linear_acceleration_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.orientation_covariance[i] = -1.0;  // Indicates invalid
    }
}
