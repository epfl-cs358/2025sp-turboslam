#include "GPS_NEO6M.h"


GPS_NEO6M::GPS_NEO6M() : gpsSerial(GPS_SERIAL_NUMBER) {}

void GPS_NEO6M::begin() { gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); }

void GPS_NEO6M::read() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }
}

double GPS_NEO6M::latitude() { return gps.location.lat(); }

double GPS_NEO6M::longitude() { return gps.location.lng(); }

double GPS_NEO6M::altitude() { return gps.altitude.meters(); }

double GPS_NEO6M::speed() { return gps.speed.kmph(); }

bool GPS_NEO6M::isValid() { return gps.location.isValid(); }


rcl_ret_t GPS_NEO6M::publishCallback(rcl_publisher_t* publisher, rcl_timer_t* gpsTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (gpsTimer == NULL) return RCL_RET_ERROR;

    size_t capacity = 128;
    char buffer[capacity];
    snprintf(buffer, sizeof(buffer), "lat = %.4f, lng = %.4f\n", latitude(), longitude());

    msg.data.data = buffer;
    msg.data.size = strlen(buffer);
    msg.data.capacity = capacity;

    return rcl_publish(publisher, &msg, NULL);
}