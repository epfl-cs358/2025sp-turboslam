#include "GPS_NEO6M.h"


GPS_NEO6M::GPS_NEO6M() : gpsSerial(GPS_SERIAL_NUMBER) {}

void GPS_NEO6M::read() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
}

double GPS_NEO6M::latitude() { return gps.location.lat(); }

double GPS_NEO6M::longitude() { return gps.location.lng(); }

double GPS_NEO6M::altitude() { return gps.altitude.meters(); }

double GPS_NEO6M::speed() { return gps.speed.kmph(); }

bool GPS_NEO6M::isValid() { return gps.location.isValid(); }


void GPS_NEO6M::begin() { gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, 16, 17); }