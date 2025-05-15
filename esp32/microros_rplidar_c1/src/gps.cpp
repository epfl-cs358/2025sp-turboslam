// #include "NEO6M.h"

// NEO6M::NEO6M(HardwareSerial &ser, uint32_t baud, int8_t rxPin, int8_t txPin)
//   : gpsSerial(ser), baudRate(baud), rxPin(rxPin), txPin(txPin) {}

// bool NEO6M::begin() {
//   gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
//   // Give GPS a moment to wake up
//   delay(1000);
//   return gpsSerial;  // non-null if started
// }

// bool NEO6M::read() {
//   // non-blocking feed of all available chars
//   while (gpsSerial.available()) {
//     gps.encode(gpsSerial.read());
//   }
//   // return true only when a valid, updated fix arrives
//   return gps.location.isUpdated();
// }

// double NEO6M::latitude()  const { return gps.location.lat(); }
// double NEO6M::longitude() const { return gps.location.lng(); }
// double NEO6M::altitude()  const { return gps.altitude.meters(); }
