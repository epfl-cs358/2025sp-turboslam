// #include <Arduino.h>
// #include <Wire.h>
// #include <SparkFun_BNO08x_Arduino_Library.h>
// #include <I2C_wire.h>

// BNO08x myIMU;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   I2C_wire.begin(8, 9);
//   pinMode(4, INPUT_PULLUP);

//   Serial.println("\nI2C Scanner");
//   for (byte address = 1; address < 127; address++) {
//     I2C_wire.beginTransmission(address);
//     if (I2C_wire.endTransmission() == 0) {
//       Serial.print("I2C device found at address 0x");
//       Serial.println(address, HEX);
//     }
//   }
//   Serial.println("I2C Scanner imu done");

//   if (!myIMU.begin(0x4B, I2C_wire)) { 
//     Serial.println("BNO086 not detected. Check wiring.");
//     while (1);
//   }

//   Serial.println("BNO086 connected.");

//   myIMU.enableRotationVector(50);  
//   myIMU.enableLinearAccelerometer(50);
//   myIMU.enableGyro(50);   
// }

// bool calibrationSaved = false;


// void loop() {
//   Serial.println("Looping...");

//   if (myIMU.getSensorEvent()) {
//     Serial.println("New sensor data received!");

//     float quatI    = myIMU.getQuatI();
//     float quatJ    = myIMU.getQuatJ();
//     float quatK    = myIMU.getQuatK();
//     float quatReal = myIMU.getQuatReal();
//     uint8_t rotAcc = myIMU.getQuatAccuracy();

//     float linX = myIMU.getLinAccelX();
//     float linY = myIMU.getLinAccelY();
//     float linZ = myIMU.getLinAccelZ();
//     uint8_t linAcc = myIMU.getAccelAccuracy();

//     float gyrX = myIMU.getGyroX();
//     float gyrY = myIMU.getGyroY();
//     float gyrZ = myIMU.getGyroZ();
//     uint8_t gyrAcc = myIMU.getGyroAccuracy();

//     Serial.printf(
//       "Q=[%0.3f, %0.3f, %0.3f, %0.3f] A=%u | "
//       "Lin=[%0.3f, %0.3f, %0.3f] A=%u | "
//       "Gyr=[%0.3f, %0.3f, %0.3f] A=%u\n",
//       quatReal, quatI, quatJ, quatK, rotAcc,
//       linX, linY, linZ, linAcc,
//       gyrX, gyrY, gyrZ, gyrAcc
//     );

//     if (rotAcc == 3 && linAcc == 3 && gyrAcc == 3 && !calibrationSaved) {
//       myIMU.saveCalibration();
//       Serial.println("Calibration saved to flash.");
//       calibrationSaved = true;
//     }
//   } else {
//     Serial.println("Waiting for new sensor data...");
//   }

//   delay(100); 
// }