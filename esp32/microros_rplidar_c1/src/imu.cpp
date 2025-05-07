// ImuSensor.cpp
#include <ImuSensor.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

ICM_20948_I2C imu;
TwoWire ImuWire(0);  // Create a new I2C bus instance

Madgwick filter; 

bool ImuSensor::begin() {
    ImuWire.begin(21, 22);  // SDA, SCL pins
    ImuWire.setClock(400000); // Set I2C clock speed to 400kHz

    Serial.println("\nI2C Scanner");
    for (byte address = 1; address < 127; address++) {
      ImuWire.beginTransmission(address);
      if (ImuWire.endTransmission() == 0) {
        Serial.print("I2C device found at address 0x");
        Serial.println(address, HEX);
      }
    }

    if (imu.begin(ImuWire, 0x68) != ICM_20948_Stat_Ok) {
        Serial.println("Failed to initialize ICM-20948");
        Serial.println(imu.statusString());
        return false;
    }

    Serial.println("ICM-20948 IMU initialized");
    filter.begin(100); // 100 hz rate (change if needed)

    // Initialize message frame
    imu_msg.header.frame_id.data     = (char *)"imu_frame";
    imu_msg.header.frame_id.size     = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    return true;
}

void ImuSensor::readAndUpdate() {
    // This is a temporary version for the data processing of the IMU,
    // Lio-Sam might need the IMU data in a different format
    if (!imu.dataReady()) return;

    delay(10); 
    imu.getAGMT();

    imu_msg.header.stamp.sec     = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    float ax = imu.accX() / 1000.0;
    float ay = imu.accY() / 1000.0;
    float az = imu.accZ() / 1000.0;

    float gx = imu.gyrX() * DEG_TO_RAD;
    float gy = imu.gyrY() * DEG_TO_RAD;
    float gz = imu.gyrZ() * DEG_TO_RAD;

    filter.updateIMU(gx, gy, gz, ax, ay, az);

    imu_msg.linear_acceleration.x = ax * 9.80665;
    imu_msg.linear_acceleration.y = ay * 9.80665;
    imu_msg.linear_acceleration.z = az * 9.80665;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    // Compute orientation
    float roll  = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw   = filter.getYawRadians();

    // quaternion are privates, so we need to convert them here
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
}
