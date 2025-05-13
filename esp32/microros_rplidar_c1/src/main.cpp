// #include "Arduino.h"

// #include "sl_lidar_esp32.h"
// #include <algorithm>

// void receiveScan(void *params);

// // const uint8_t motorControlPin = 14;

// using namespace sl;

// bool checkSLAMTECLIDARHealth(ILidarDriver *drv);

// void setup()
// {
//   // const uint8_t rxPin = 18;
//   // const uint8_t txPin = 17;
//   const uint8_t rxPin = 5;
//   const uint8_t txPin = 4;

//   Serial.begin(115200);
//   Serial.printf("Upload successful!\n");
//   Serial2.begin(115200, SERIAL_8N1, rxPin, txPin);
//   // pinMode(motorControlPin, OUTPUT);
//   // digitalWrite(motorControlPin, LOW);
//   delay(3000);

//   Serial.printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
//                 "Version: %d.%d.%d\n\n",
//                 SL_LIDAR_SDK_VERSION_MAJOR, SL_LIDAR_SDK_VERSION_MINOR, SL_LIDAR_SDK_VERSION_PATCH);
//   sl_result op_result;

//   IChannel *_channel;
//   ILidarDriver *drv = *createLidarDriver();
//   assert(drv != nullptr);
//   Serial.println("Created LiDAR driver");

//   sl_lidar_response_device_info_t devinfo;
//   _channel = *createSerialPortChannel(Serial2);
//   Serial.println("Created Serial Channel");

//   bool connectSuccess = false;
//   if (SL_IS_OK((drv)->connect(_channel)))
//   {
//     Serial.println("Connected to LiDAR");
//     op_result = drv->getDeviceInfo(devinfo);
//     if (SL_IS_OK(op_result))
//     {
//       connectSuccess = true;
//     }
//     else
//     {
//       // log_e("Failed to get Device Info");
//       Serial.println("Failed to get Device Info");
//       delete drv;
//       drv = nullptr;
//     }
//   }
//   else
//   {
//     // log_e("Connect Failed");
//     Serial.println("Connect Failed");
//   }

//   if (connectSuccess)
//   {
//     Serial.printf("SLAMTEC LIDAR S/N: ");
//     for (int pos = 0; pos < 16; ++pos)
//     {
//       Serial.printf("%02X", devinfo.serialnum[pos]);
//     }

//     Serial.printf("\n"
//                   "Firmware Ver: %d.%02d\n"
//                   "Hardware Rev: %d\n",
//                   devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);

//     if (!checkSLAMTECLIDARHealth(drv))
//     {
//       while (1)
//         ;
//     }
//     // BaseType_t returnCode = xTaskCreatePinnedToCore(receiveScan, "receiveScan", 3200, drv, 6, NULL, CONFIG_ARDUINO_RUNNING_CORE);
//     // assert(returnCode != pdFAIL);
//   }
// }

// void loop()
// {
// }

// void receiveScan(void *params)
// {
//   const size_t maxNodes = 8192;
//   ILidarDriver *drv = reinterpret_cast<ILidarDriver *>(params);
//   sl_lidar_response_measurement_node_hq_t *nodes = reinterpret_cast<sl_lidar_response_measurement_node_hq_t *>(ps_malloc(maxNodes * sizeof(sl_lidar_response_measurement_node_hq_t)));
//   assert(nodes != nullptr);
//   size_t count = maxNodes;
//   sl_result op_result;
//   std::vector<LidarScanMode> modes;

//   Result<nullptr_t> ans = drv->getAllSupportedScanModes(modes);
//   if (!ans)
//   {
//     log_e("No Scan Modes Detected");
//     free(nodes);
//     modes.clear();
//     vTaskDelete(NULL);
//   }

//   while (1)
//   {
//     Serial.printf("Avaiable Scan Modes:\n\n");
//     for (auto &mode : modes)
//     {
//       Serial.printf("Mode ID: %d, Mode Name: %s, ", mode.id, mode.scan_mode);
//       Serial.printf("uS Per Sample: %.2f, Max Distance: %.2f, Answer Type: 0x%.2X\n", mode.us_per_sample, mode.max_distance, mode.ans_type);
//     }

//     Serial.printf("\nEnter # for Desired Scan Type (press any key to stop scan after it starts): ");
//     std::vector<LidarScanMode>::iterator selectedMode;
//     int selection;
//     while (1)
//     {

//       if ((selection = Serial.read()) >= '0')
//       {
//         selection -= '0';
//         auto predicate = [selection](LidarScanMode &mode)
//         {
//           return (selection == mode.id);
//         };
//         selectedMode = std::find_if(modes.begin(), modes.end(), predicate);
//         if (selectedMode != modes.end())
//         {
//           break;
//         }
//       }
//     }

//     Serial.printf("\nStarting Scan Mode: %s\n", selectedMode->scan_mode);
//     while (Serial.read() >= 0)
//     {
//     }

//     // digitalWrite(motorControlPin, HIGH);
//     vTaskDelay(500);
//     drv->startScanExpress(false, selectedMode->id, 0, nullptr);

//     while (1)
//     {
//       op_result = drv->grabScanDataHq(nodes, count);
//       if (SL_IS_OK(op_result))
//       {
//         drv->ascendScanData(nodes, count);
//         for (int pos = 0; pos < (int)count; ++pos)
//         {
//           Serial.printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                         (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
//                         (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
//                         nodes[pos].dist_mm_q2 / 4.0f,
//                         nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
//         }
//       }

//       if (Serial.available())
//       {
//         drv->stop();
//         // digitalWrite(motorControlPin, LOW);
//         Serial.println("Scanning Stopped\n\n");
//         while (Serial.read() >= 0)
//         {
//         }
//         break;
//       }
//     }
//   }
// }

// bool checkSLAMTECLIDARHealth(ILidarDriver *drv)
// {
//   sl_lidar_response_device_health_t healthinfo;
//   std::vector<LidarScanMode> modes;
//   Result<nullptr_t> ans = SL_RESULT_OK;
//   MotorCtrlSupport motorCtrlSupport;

//   Serial.printf("Resetting\n");
//   drv->reset();
//   delay(2000);
//   ans = drv->getHealth(healthinfo);
//   if (ans)
//   {
//     Serial.printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
//     if (healthinfo.status == SL_LIDAR_STATUS_ERROR)
//     {
//       log_e("Error, slamtec lidar internal error detected. Please reboot the device to retry.");
//       return false;
//     }
//   }
//   else
//   {
//     log_e("Error, cannot retrieve the lidar health code: %x", ans);
//     return false;
//   }

//   ans = drv->checkMotorCtrlSupport(motorCtrlSupport);
//   if (ans)
//   {
//     Serial.printf("Motor Speed Control Support: ");
//     switch (motorCtrlSupport)
//     {
//     case MotorCtrlSupportNone:
//       Serial.printf("None\n");
//       break;

//     case MotorCtrlSupportPwm:
//       Serial.printf("PWM\n");
//       break;

//     case MotorCtrlSupportRpm:
//       Serial.printf("RPM\n");
//       break;

//     default:
//       break;
//     }
//   }
//   else
//   {
//     log_e("Couldn't Get Motor Speed Control Info");
//   }

//   Serial.printf("\n");

//   return true;
// }

//////////////////////////////////////////////////

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
// #include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "credentials.h"
#include "RplidarC1.h"

// Micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_node_t node;

#define RCCHECK(fn, msg)                           \
    {                                              \
        rcl_ret_t temp_rc = fn;                    \
        if ((temp_rc != RCL_RET_OK))               \
        {                                          \
            printf("err=%d %s\r\n", temp_rc, msg); \
        }                                          \
        return temp_rc;                            \
    }
#define RCSOFTCHECK(fn, msg)                       \
    {                                              \
        rcl_ret_t temp_rc = fn;                    \
        if ((temp_rc != RCL_RET_OK))               \
        {                                          \
            printf("err=%d %s\r\n", temp_rc, msg); \
        }                                          \
        return temp_rc;                            \
    }

RplidarC1 lidar;

void connect_wifi()
{
    WiFi.disconnect(true);  // Reset Wi-Fi
    WiFi.mode(WIFI_STA);    // Set to Station mode
    WiFi.begin(ssid, pass); // ssid and pass are defined in credentials.h
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

rcl_ret_t init_ros()
{
    // Micro-ROS initialization

    rcl_ret_t ret;

    struct my_micro_ros_agent_locator
    {
        IPAddress address;
        int port;
    } static locator;
    locator.address = ros2_agent_ipa; // ros2_agent_ipa and ros2_agent_port are defined in credentials.h
    locator.port = ros2_agent_port;

    printf("rmw_uros_set_custom_transport...\r\n");
    ret = rmw_uros_set_custom_transport(
        false,
        (void *)&locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read);
    if (RCL_RET_OK != ret)
    {
        printf("rmw_uros_set_custom_transport error=%d\r\n", ret);
        return ret;
    }

    allocator = rcl_get_default_allocator();

    printf("rclc_support_init...\r\n");
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (RCL_RET_OK != ret)
    {
        printf("rclc_support_init error=%d\r\n", ret);
        return ret;
    }
    printf("rclc_node_init_default...\r\n");
    ret = rclc_node_init_default(&node, "lidar_node", "", &support);
    if (RCL_RET_OK != ret)
    {
        printf("rclc_node_init_default error=%d\r\n", ret);
        return ret;
    }
    printf("rclc_publisher_init_default...\r\n");
    ret =
        rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "/scan");
    if (RCL_RET_OK != ret)
    {
        printf("rclc_publisher_init_default error=%d\r\n", ret);
        return ret;
    }

    //     // create 20 msecs timer
    //     // printf("create lidar timer...\r\n");
    //     // const unsigned int lidar_timer_timeout = 200;
    //     // ret = rclc_timer_init_default(
    //     //   &timer,
    //     //   &support,
    //     //   RCL_MS_TO_NS(lidar_timer_timeout),
    //     //   lidar_timer_callback);
    //     // if(ret != RCL_RET_OK){
    //     //   printf("rclc_timer_init_default lidar error=%d",ret);
    //     //   return ret;
    //     // }

    //     // // create executor
    //     // printf("create executor...\r\n");
    //     // ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
    //     // if(ret != RCL_RET_OK){
    //     //   printf("rclc_executor_init error=&d\r\n",ret);
    //     //   return ret;
    //     // }

    //     // printf("add  time to executor...\r\n");
    //     // ret = rclc_executor_add_timer(&executor, &timer);
    //     // if(RCL_RET_OK !=ret){
    //     //   printf("rclc_executor_add_timer error=%d\r\n",ret);
    //     //   return ret;
    //     // }

    return RCL_RET_OK;
}

// Setup function
void setup()
{
    Serial.begin(115200); // Initialize Serial for debugging

    connect_wifi();

    if (RCL_RET_OK != init_ros())
    {
        printf("init_ros error. Rebooting ...\r\n");
        esp_restart();
    }

    lidar.begin();
    delay(1000);
    lidar.resetLidar();
    delay(800);
    lidar.startLidar();
}

// rcl_ret_t ret;
int main_loop_count = 0;
unsigned long mil = 0L;
// main timer callback
// unsigned long start_uart;
// unsigned long start_publishing;
// unsigned long start_processing;
unsigned long total_loop_time = 0L;
float loop_period = 0.0;
// int loop_count = 0;

void loop_simple()
{
    int count = lidar.uartRx();
    Serial.printf("got %d points\r\n", count);
    // lidar.processFrame(count);
}

void loop()
{
    // Check Wi-Fi connection
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Wi-Fi disconnected, reconnecting...");
        connect_wifi();
    }

    unsigned long uart_elapsed = millis();
    int count = lidar.uartRx();
    uart_elapsed = millis() - uart_elapsed;

    unsigned long process_elapsed = millis();
    lidar.processFrame(count);
    process_elapsed = millis() - process_elapsed;

    unsigned long publish_elapsed = millis();
    rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
    publish_elapsed = millis() - publish_elapsed;

    if (ret_pub != RCL_RET_OK)
    {
        printf("rcl_publish returned %d\r\n", ret_pub);
        esp_restart();
    }

    // calculate loop period
    total_loop_time = millis() - total_loop_time;
    float total_loop_time_f = (float)total_loop_time;
    loop_period = loop_period * 0.9 + total_loop_time_f * 0.1;

    Serial.printf("got %d points in %lu ms. Frame Processing in %lu. Frame publishing in %lu. Total loop in %lu ms. Freq=%.1f Hz Serial2.available=%d\r\n",
                  count, uart_elapsed, process_elapsed, publish_elapsed, total_loop_time, 1000.0 / loop_period, Serial2.available());
    total_loop_time = millis();

    delay(30);
}

// void setup()
// {
//     Serial.begin(115200); // Main serial monitor
//     delay(1000);
//     Serial.println("Initializing Serial2...");

//     Serial2.begin(460800, SERIAL_8N1, 16, 17); // GPIO16 = RX, GPIO17 = TX
//     Serial.println("Serial2 started. Sending RPLIDAR reset command...");

//     uint8_t resetCommand[] = {0xA5, 0x40}; // Reset command for RPLIDAR
//     Serial2.write(resetCommand, sizeof(resetCommand));
//     Serial.println("Reset command sent.");
// }

// void loop()
// {
//     if (Serial2.available())
//     {
//         uint8_t b = Serial2.read();
//         Serial.print("Received: 0x");
//         if (b < 0x10)
//             Serial.print("0");
//         Serial.println(b, HEX);
//     }
// }

// ///////////////////////

// #include <Arduino.h>

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting...");

//   const uint8_t rxPin = 10;
//   const uint8_t txPin = 11;

//   // RPLIDAR on Serial2 at 460800bps
//   Serial2.begin(460800, SERIAL_8N1, rxPin, txPin);
//   while(!Serial2);

//   Serial.println("Serial2 initialized, getting device info...");

//   // Build GET_INFO request packet:
//   // [Start Flag][Cmd=0x50][Payload Size=0][Checksum = 0xA5 ^ 0x50]
//   uint8_t getInfoCmd[4];
//   getInfoCmd[0] = 0xA5;
//   getInfoCmd[1] = 0x50;   // GET_INFO
//   getInfoCmd[2] = 0x00;   // no payload
//   getInfoCmd[3] = getInfoCmd[0] ^ getInfoCmd[1] ^ getInfoCmd[2]; // 0xF5 :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}

//   // Send request
//   Serial2.write(getInfoCmd, sizeof(getInfoCmd));

//   // --- STEP 1: read response descriptor (7 bytes) ---
//   // Format: 0xA5,0x5A, <len low>, <len mid>, <len high + mode bits>, <data type>
//   const uint8_t DESC_LEN = 7;
//   uint8_t desc[DESC_LEN];
//   // Wait until we have at least 7 bytes
//   while (Serial2.available() < DESC_LEN) {
//     Serial.print(Serial2.available());
//     delay(100);
//   }
//   Serial2.readBytes(desc, DESC_LEN);

//   // Verify descriptor header
//   if (desc[0] != 0xA5 || desc[1] != 0x5A) {
//     Serial.println("Invalid response descriptor header");
//     return;
//   }

//   // Compute data length (little-endian)
//   uint32_t dataLen = (uint32_t)desc[2]
//                    | (uint32_t)desc[3] << 8
//                    | (uint32_t)desc[4] << 16;
//   Serial.print("Incoming device-info packet length: ");
//   Serial.println(dataLen);

//   // --- STEP 2: read the actual Device Info payload ---
//   if (dataLen != 20) {
//     Serial.print("Unexpected payload size: ");
//     Serial.println(dataLen);
//     return;
//   }
//   uint8_t payload[20];
//   while (Serial2.available() < dataLen) { /* spin */ }
//   Serial2.readBytes(payload, dataLen);

//   // --- STEP 3: parse and print ---
//   uint8_t model          = payload[0];
//   uint8_t firmware_minor = payload[1];
//   uint8_t firmware_major = payload[2];
//   uint8_t hardware       = payload[3];
//   // serial number is 16 bytes, LSB first
//   Serial.println("=== RPLIDAR Device Info ===");
//   Serial.print("Model ID: 0x");
//   Serial.println(model, HEX);
//   Serial.print("Firmware: ");
//   Serial.print(firmware_major);
//   Serial.print('.');
//   Serial.println(firmware_minor);
//   Serial.print("Hardware Rev: ");
//   Serial.println(hardware);

//   // Serial number as hex string
//   Serial.print("Serial #: ");
//   for (int i = 4; i < 20; i++) {
//     if (payload[i] < 0x10) Serial.print('0');
//     Serial.print(payload[i], HEX);
//   }
//   Serial.println();
// }

// void loop()
// {

// }