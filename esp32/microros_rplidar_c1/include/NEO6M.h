// #ifndef NEO6M_H
// #define NEO6M_H

// #include <Arduino.h>
// #include <TinyGPS++.h>

// class NEO6M {
// public:
//   NEO6M(HardwareSerial &ser, uint32_t baud, int8_t rxPin, int8_t txPin);

//   /// Initialize serial port. Call once in setup()
//   bool begin();

//   /// Call in a loop to feed parser; returns true if a new fix is available
//   bool read();

//   /// After read() returns true, these accessors hold the latest fix
//   double latitude() const;
//   double longitude() const;
//   double altitude() const;  // meters

// private:
//   HardwareSerial &gpsSerial;
//   const uint32_t baudRate;
//   const int8_t rxPin, txPin;
//   TinyGPSPlus gps;
// };

// #endif // NEO6M_H
