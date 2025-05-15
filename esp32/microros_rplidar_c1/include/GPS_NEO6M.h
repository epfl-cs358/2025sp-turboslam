#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <rcl/rcl.h> // node, publisher, etc
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#define GPS_BAUDRATE 9600
#define GPS_SERIAL_NUMBER 2
#define RX_PIN 16
#define TX_PIN 17

class GPS_NEO6M {
    public:
        GPS_NEO6M();
        void begin();
        void read();
        double latitude();
        double longitude();
        double altitude();
        double speed();
        bool isValid();
        rcl_ret_t publishCallback(rcl_publisher_t* publisher, rcl_timer_t* gpsTimer, int64_t last_call_time);
        std_msgs__msg__String msg;
        
        TinyGPSPlus gps;
    private:
        HardwareSerial gpsSerial;
};