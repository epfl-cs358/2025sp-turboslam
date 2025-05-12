#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <std_msgs/msg/string.h>

#define GPS_BAUDRATE 9600
#define GPS_SERIAL_NUMBER 2

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
        std_msgs__msg__String msg;
    
    private:
        TinyGPSPlus gps;
        HardwareSerial gpsSerial;
};