#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <Adafruit_BNO08x.h>
#include <AS5600.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h> //a verigier sarah 

#include "I2C_mutex.h"
#include "MotorController.h"
#include "credentials.h"
#include "I2C_wire.h"
#include "RplidarC1.h"
#include "ImuSensor.h"
#include "UltraSonicSensor.h"
#include "AS5600Encoder.h"
#include "DMS15.h"
#include "NEO6M.h" 

#define TEST_IMU             0
#define TEST_ULTRASONIC      1
#define TEST_ENCODER         0
#define TEST_SERVO_DIR       1
#define TEST_SERVO_LID       0
#define TEST_SERVO_ANGLE_PUB 0
#define TEST_GPS             0
#define TEST_LIDAR           0
#define TEST_MOTOR           1

#define ESC_PIN 15

// Micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_publisher_t publisher;
rcl_node_t      node;
// Lidar
RplidarC1 lidar;
// IMU
ImuSensor       imuSensor;
rcl_publisher_t imu_publisher;

// Ultrasonic sensor
UltraSonicSensor ultrasonic(10, 11);
rcl_publisher_t  range_publisher;

volatile float last_dist = 999.0f ; //arbitrary value just to init
volatile bool g_obstacle = false;
constexpr int CLEAR_CYCLES = 5;
static int clear_timer = CLEAR_CYCLES;
rcl_publisher_t estop_pub;
std_msgs__msg__Bool estop_msg;

// Encoder
AS5600Encoder   encoder;
rcl_publisher_t encoder_publisher;
//Servo
DMS15                servo_dir(6); // servo used for the direcition of the car
rcl_subscription_t   servo_dir_subscriber;
std_msgs__msg__Int32 servo_dir_angle_msg;
DMS15                servo_lid(7); // servo used for tilting the lidar
rcl_subscription_t   servo_lid_subscriber;
std_msgs__msg__Int32 servo_lid_angle_msg;
rcl_publisher_t      servo_angle_publisher;
std_msgs__msg__Int32 servo_angle_msg;
//gps
NEO6M                       gps(Serial2, 9600, 18, 17); 
rcl_publisher_t             gps_publisher;
sensor_msgs__msg__NavSatFix gps_msg;
// Motor
rcl_subscription_t  motor_cmd_subscriber;
std_msgs__msg__Int8 motor_cmd_msg;
MotorController     motor(ESC_PIN);

rclc_executor_t   executor;
SemaphoreHandle_t ros_publish_mutex;
QueueHandle_t     servoAngleQ;
SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();

#define RCCHECK(fn, msg)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}return temp_rc;}

void connect_wifi(){
    WiFi.disconnect(true); // Reset Wi-Fi
    WiFi.mode(WIFI_STA); // Set to Station mode
    WiFi.begin(ssid, pass); // ssid and pass are defined in credentials.h
        while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

//Servo
void servo_dir_callback(const void* msgin) {
const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
servo_dir.setAngle(msg->data);
}

void servo_lid_callback(const void* msgin) {
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    servo_lid.setAngle(msg->data);
    Serial.print("Received angle: ");
    Serial.println(msg->data);
}

void motor_callback(const void* msgin) {
    const auto *cmd = static_cast<const std_msgs__msg__Int8*>(msgin);
    motor.setTargetPercent(float(cmd->data) * 0.2);
}

rcl_ret_t init_ros() {
    // Micro-ROS initialization
    rcl_ret_t ret;
    struct my_micro_ros_agent_locator {
        IPAddress address;
        int port;
    } static locator;
    locator.address = ros2_agent_ipa; // ros2_agent_ipa and ros2_agent_port are defined in credentials.h
    locator.port = ros2_agent_port;

    printf("rmw_uros_set_custom_transport...\r\n");
    ret=rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );

    if (RCL_RET_OK != ret){
        printf("rmw_uros_set_custom_transport error=%d\r\n",ret);
        return ret;
    }

    allocator = rcl_get_default_allocator();
    
    printf("rclc_support_init...\r\n");
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (RCL_RET_OK != ret){
        printf("rclc_support_init error=%d\r\n",ret);
        return ret;
    }
    printf("rclc_node_init_default...\r\n");
    ret = rclc_node_init_default(&node, "lidar_node", "", &support);
    if (RCL_RET_OK != ret){
        printf("rclc_node_init_default error=%d\r\n",ret);
        return ret;
    }

    // Lidar
    #if TEST_LIDAR
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
    #endif

    // IMU
    #if TEST_IMU
        ret = rclc_publisher_init_default(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu"
        );
        if (RCL_RET_OK != ret){
            printf("imu publisher init error=%d\r\n", ret);
            return ret;
        }
    #endif

    // US sensor
    #if TEST_ULTRASONIC
        ret = rclc_publisher_init_default(
            &range_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            "/ultrasonic"
        );
        if (ret != RCL_RET_OK) {
            printf("ultrasonic publisher init failed: %d\n", ret);
            return ret;
        }
        ret = rclc_publisher_init_default(
            &estop_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/emergency_stop"
        );
        if (ret != RCL_RET_OK) {
            printf("emergency stop publisher init failed: %d\n", ret);
            return ret;
        }


    #endif

    // Encoder
    #if TEST_ENCODER
        ret = rclc_publisher_init_default(
            &encoder_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/encoder"
        );
        if (ret != RCL_RET_OK) {
            printf("encoder publisher init failed: %d\n", ret);
            return ret;
        }
    #endif

    // GPS
    #if TEST_GPS
        ret = rclc_publisher_init_default(
            &gps_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
            "/gps"
        );
        if (ret != RCL_RET_OK) {
            printf("gps publisher init failed: %d\n", ret);
            return ret;
        }
    #endif

    // Servo_lid puvlisher to get the angle
    #if TEST_SERVO_ANGLE_PUB
        ret = rclc_publisher_init_default(
            &servo_angle_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/lidar_servo_angle"
        );
        if (ret != RCL_RET_OK) {
            printf("Failed to create servo_angle publisher: %d\n", ret);
            return ret;
        }
    #endif

    //Servo dir
    #if TEST_SERVO_DIR
    ret = rclc_subscription_init_default(
        &servo_dir_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/servo_dir/angle"
    );
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to create servo_dir subscriber");
        esp_restart();
    }
    #endif

    //Servo lid
    #if TEST_SERVO_LID
        ret = rclc_subscription_init_default(
            &servo_lid_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/servo_lid/angle"
        );
        if (ret != RCL_RET_OK) {
            Serial.println("Failed to create servo_lid subscriber");
            esp_restart();
        }
    #endif

    #if TEST_MOTOR
        // Motor control subscriber
        ret = rclc_subscription_init_default(
            &motor_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            "/motor_cmd"
        );
        if (ret != RCL_RET_OK) {
            Serial.println("Failed to create motor_cmd subscriber");
            esp_restart();
        }
    #endif

    // Executor
    ret = rclc_executor_init(&executor, &support.context, 3, &allocator); // 3 = number of handles (subscribers) (motor and 2 servos)
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize executor");
        return ret;
    }
    
    // Add subscriptions to executor
    //Motor
    #if TEST_MOTOR 
        ret = rclc_executor_add_subscription(
        &executor,
        &motor_cmd_subscriber,
        &motor_cmd_msg,
        &motor_callback,
        ON_NEW_DATA
        );
        if (ret != RCL_RET_OK) {
            printf("Failed to add motor callback to executor: %d\n", ret);
            return ret;
        }
    #endif

    //Servo dir
    #if TEST_SERVO_DIR
        ret = rclc_executor_add_subscription(
            &executor,
            &servo_dir_subscriber,
            &servo_dir_angle_msg,
            &servo_dir_callback,
            ON_NEW_DATA
        );
            if (ret != RCL_RET_OK) {
                printf("Failed to add servo_dir callback to executor: %d\n", ret);
            return ret;
        }
    #endif

    // Servo lid
    #if TEST_SERVO_LID
        ret = rclc_executor_add_subscription(
            &executor,
            &servo_lid_subscriber,
            &servo_lid_angle_msg,
            &servo_lid_callback,
            ON_NEW_DATA
        );
        if (ret != RCL_RET_OK) {
            printf("Failed to add servo_lid callback to executor: %d\n", ret);
            return ret;
        }
    #endif

    return RCL_RET_OK;
}

void imuTask(void *parameter);
void encoderTask(void *parameter);
void ultrasonicTask(void *parameter);
void servoLidTask(void *parameter);
void executorTask(void *parameter);
void wifiMonitorTask(void *parameter);
void servoPublisherTask(void *parameter);
void lidarTask(void *parameter);
void motorTask(void *parameter);
void gpsTask(void *parameter);

// Setup function
void setup() {
    Serial.begin(115200);
    Serial.println("Starting up...");
    I2C_wire.begin(8, 9);
    connect_wifi();
    if(RCL_RET_OK != init_ros()){
        printf("init_ros error. Rebooting ...\r\n");
        esp_restart();
    }

    // if(!servo_dir.begin()) {
    // Serial.println("Servo_dir failed to initialize, rebooting...");
    // esp_restart();
    // }

    ros_publish_mutex = xSemaphoreCreateMutex();
    servoAngleQ = xQueueCreate(1, sizeof(int));

    #if TEST_IMU
        if (!imuSensor.begin()) {
            Serial.println("IMU failed to initialize, rebooting...");
            esp_restart();
        }
        BaseType_t ` = xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 5, NULL, 1);
        if (imuTaskCreated != pdPASS) {
            Serial.println("Failed to create IMU Task");
            esp_restart();
        }
    #endif

    #if TEST_ENCODER
        if (!encoder.begin()) {
            Serial.println("Encoder failed to initialize, rebooting...");
            esp_restart();
        }
        Serial.println("Encoder initialized");
        BaseType_t encoderTaskCreated = xTaskCreatePinnedToCore(encoderTask, "Encoder Task", 2048, NULL, 4, NULL, 0);
        if (encoderTaskCreated != pdPASS) {
            Serial.println("Failed to create Encoder Task");
            esp_restart();
        }
    #endif

    #if TEST_ULTRASONIC
        if (!ultrasonic.begin()) {
            Serial.println("Ultrasonic sensor failed to initialize, rebooting...");
            esp_restart();
        }
        BaseType_t ultrasonicTaskCreated = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic Task", 2048, NULL, 4, NULL, 1);
        if (ultrasonicTaskCreated != pdPASS) {
            Serial.println("Failed to create Ultrasonic Task");
            esp_restart();
        }
    #endif

    #if TEST_SERVO_LID
        if(!servo_lid.begin()) {
            Serial.println("Servo_lib failed to initialize, rebooting...");
            esp_restart();
        }
        BaseType_t servoLidTaskCreated = xTaskCreatePinnedToCore(servoLidTask, "Servo Lid Task", 2048, NULL, 5, NULL, 1);
        if (servoLidTaskCreated != pdPASS) {
            Serial.println("Failed to create Servo Lid Task");
            esp_restart();
        }
    #endif

    #if TEST_SERVO_DIR
        if(!servo_dir.begin()) {
            Serial.println("Servo_dir failed to initialize, rebooting...");
            esp_restart();
        }
        // servo_dir.setAngle(90); // angle to 90 (center position)
        // BaseType_t servoDirTaskCreated = xTaskCreatePinnedToCore(servoDirTask, "Servo Dir Task", 2048, NULL, 5, NULL, 1);
        // if (servoDirTaskCreated != pdPASS) {
        //     Serial.println("Failed to create Servo Dir Task");
        //     esp_restart();
        // }
    #endif

    #if TEST_GPS
        if (!gps.begin()) {
            Serial.println("GPS init failed!");
            esp_restart();
        }
        BaseType_t gpsTaskCreated = xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 3, NULL, 0);
        if (gpsTaskCreated != pdPASS) {
            Serial.println("Failed to create GPS Task");
            esp_restart();
        }
    #endif

    #if TEST_SERVO_ANGLE_PUB
        BaseType_t servoTaskCreated = xTaskCreatePinnedToCore(servoPublisherTask, "ServoPub", 2048, NULL, 4, NULL, 1);
        if (servoTaskCreated != pdPASS) {
        Serial.println("Failed to create Servo Publisher Task");
        esp_restart();
        }
    #endif

    #if TEST_LIDAR
        lidar.begin();
        delay(1000);
        lidar.resetLidar();
        delay(800);
        lidar.startLidar();
        BaseType_t lidarTaskCreated = xTaskCreatePinnedToCore(lidarTask, "Lidar Task", 4096, NULL, 5, NULL, 0);
        if (lidarTaskCreated != pdPASS) {
            Serial.println("Failed to create Lidar Task");
            esp_restart();
        }
    #endif

    #if TEST_MOTOR
        if (!motor.begin()) {
            Serial.println("Motor failed to initialize, rebooting...");
            esp_restart();
        }
        BaseType_t motorTaskCreated = xTaskCreatePinnedToCore(motorTask, "Motor Task", 2048, NULL, 5, NULL, 1);
        if (motorTaskCreated != pdPASS) {
            Serial.println("Failed to create Motor Task");
            esp_restart();
        }
    #endif


    BaseType_t executorTaskCreated = xTaskCreatePinnedToCore(executorTask, "Executor Task", 4096, NULL, 4, NULL, 0); // Higher priority
    if (executorTaskCreated != pdPASS) {
        Serial.println("Failed to create Executor Task");
        esp_restart();
    }

    BaseType_t wifiMonitorTaskCreated = xTaskCreatePinnedToCore(wifiMonitorTask, "WiFi Monitor", 2048, NULL, 1, NULL, 0);
    if (wifiMonitorTaskCreated != pdPASS) {
        Serial.println("Failed to create WiFi Monitor Task");
        esp_restart();
    }
}

void imuTask(void *parameter) {
    while (true) {
        imuSensor.readAndUpdate();
        if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
            if (ret != RCL_RET_OK) {
                printf("rcl_publish IMU error=%d\r\n", ret);
            }
            xSemaphoreGive(ros_publish_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // 200 Hz
    }
}

void encoderTask(void *parameter) {
    while (true) {
        std_msgs__msg__Int32 angle_msg = encoder.update();
        if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t ret = rcl_publish(&encoder_publisher, &angle_msg, NULL);
            if (ret != RCL_RET_OK) {
                printf("rcl_publish encoder error=%d\r\n", ret);
            }
            xSemaphoreGive(ros_publish_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}

void ultrasonicTask(void *parameter) {
    while (true) {
        float dist = ultrasonic.readDistance();
        if (dist > 0) {
            ultrasonic.range_msg.range = dist;
            ultrasonic.range_msg.header.stamp.sec = millis() / 1000;
            ultrasonic.range_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

            if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
                rcl_ret_t ret = rcl_publish(&range_publisher, &ultrasonic.range_msg, NULL);
                if (ret != RCL_RET_OK) {
                    printf("rcl_publish ultrasonic error=%d\r\n", ret);
                }
                xSemaphoreGive(ros_publish_mutex);
            }

            g_obstacle = (dist < UltraSonicSensor::STOP_THRESHOLD);
            estop_msg.data = g_obstacle;
            if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
                rcl_ret_t ret = rcl_publish(&estop_pub, &estop_msg, NULL);
                if (ret != RCL_RET_OK) {
                    printf("rcl_publish emergency stop error=%d\r\n", ret);
                }
                xSemaphoreGive(ros_publish_mutex);
            }

        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

void servoLidTask(void *parameter) {
    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        // compute next angle
        servo_lid.tiltLidar(60, 120, 8000);
        int angle = servo_lid.getAngle();
        xQueueOverwrite(servoAngleQ, &angle);
        vTaskDelayUntil(&lastWake, period);
    }
}

// void servoDirTask(void *parameter) {
//     const TickType_t period = pdMS_TO_TICKS(20);
//     TickType_t lastWake = xTaskGetTickCount();
//     while (true) {
//         ???
//     }
// }

void servoPublisherTask(void *parameter) {
    std_msgs__msg__Int32 msg;
    int angle = 0;
    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        int latest;
        if (xQueuePeek(servoAngleQ, &latest, 0) == pdTRUE) {
            angle = latest;
        }
        msg.data = angle;
        if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t ret = rcl_publish(&servo_angle_publisher, &msg, nullptr);
            if (ret != RCL_RET_OK) {
                printf("rcl_publish servo_lid error=%d\r\n", ret);
            }
            xSemaphoreGive(ros_publish_mutex);
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void executorTask(void *parameter) {
    while (true) {
        if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            if (ret != RCL_RET_OK) {
                printf("rclc_executor_spin_some error=%d\r\n", ret);
            }
            xSemaphoreGive(ros_publish_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

void wifiMonitorTask(void *parameter) {
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Wi-Fi disconnected, reconnecting...");
            connect_wifi();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void gpsTask(void*) {
    while (true) {
        if (gps.read()) {
            gps_msg.header.stamp.sec = millis() / 1000;
            gps_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
            gps.populateNavSatFix(gps_msg);
            rcl_ret_t ret = rcl_publish(&gps_publisher, &gps_msg, nullptr);
            if (ret != RCL_RET_OK) {
                printf("rclc_publish gps error=%d\r\n", ret);
            }
        }
    } 
}

void lidarTask(void *parameter) {
    const TickType_t period = pdMS_TO_TICKS(100);  // 100 ms → 10 Hz
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        int count = lidar.uartRx();
        lidar.processFrame(count);

        if (xSemaphoreTake(ros_publish_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
            if (ret_pub != RCL_RET_OK)
            {
                printf("rcl_publish returned %d\r\n", ret_pub);
                esp_restart();
            }
            xSemaphoreGive(ros_publish_mutex);
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void motorTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(UPDATE_PERIOD_MS);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
         if (g_obstacle){
            motor.emergencyStop();
            clear_timer = CLEAR_CYCLES;
         } else if (clear_timer){
            --clear_timer;
         }

        motor.update();
        xTaskDelayUntil(&lastWake, period);
    }
}

void loop() {

}

// #include <Arduino.h>
// #include <WiFi.h>
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <sensor_msgs/msg/laser_scan.h>

// #include "credentials.h"
// #include "RplidarC1.h"

// #include <ESP32Servo.h>                 // Arduino-compatible servo PWM library for ESP32
// #include <sensor_msgs/msg/joy.h>        // ROS2 Joy message type
// #include <std_msgs/msg/int32.h>        // ROS2 Int32 message type
// #include <std_msgs/msg/float32.h>

// #define ESC_PWM_PIN    15    // GPIO15 for throttle ESC signal
// #define SERVO_PWM_PIN   6    // GPIO6 for steering servo signal

// Servo escServo;
// Servo steeringServo;

// // Subscriptions
// auto joy_sub       = rcl_get_zero_initialized_subscription();
// auto accel_sub     = rcl_get_zero_initialized_subscription();
// auto decel_sub     = rcl_get_zero_initialized_subscription();
// auto joyx_sub      = rcl_get_zero_initialized_subscription();
// auto joyy_sub      = rcl_get_zero_initialized_subscription();

// sensor_msgs__msg__Joy         joy_msg;
// std_msgs__msg__Float32        accel_msg;
// std_msgs__msg__Float32        decel_msg;
// std_msgs__msg__Float32        joyx_msg;
// std_msgs__msg__Float32        joyy_msg;

// void joyCallback(const void * msgin) {
//   Serial.println("[JOY] joyCallback()");  
//   const sensor_msgs__msg__Joy * joy = (const sensor_msgs__msg__Joy *)msgin;

//   // Read axes from Joy message
//   float steer_axis  = joy->axes.data[0];   // left/right stick for steering
//   float brake_axis  = joy->axes.data[2];   // LT trigger for brake/reverse
//   float throttle_axis = joy->axes.data[5]; // RT trigger for throttle

//   Serial.printf("[JOY] steer=%.2f, throttle=%.2f, brake=%.2f\n",
//     steer_axis, throttle_axis, brake_axis);

//     // TODO: CAREFUL, the following code is likely to crash the ESP32, was not tested
// //   // Apply small deadzone for steering to avoid jitter around 0
// //   if (fabs(steer_axis) < 0.05f) {  // deadzone threshold ~5%
// //     steer_axis = 0.0f;
// //   }

// //   // Map steering axis (-1 to +1) to servo pulse width (1000 to 2000 µs)
// //   int steering_us = (int)(1500 + steer_axis * 500);  // centered at 1500 µs, ±500 µs range
// //   steering_us = constrain(steering_us, 1400, 1600);  // clamp to [1000,2000] just in case
// //   steeringServo.writeMicroseconds(steering_us);

// //   // Determine throttle/reverse for ESC. Neutral is 1500 µs.
// //   float forward = 0.0f;
// //   float reverse = 0.0f;
// //   // Some controllers report triggers 0.0 to 1.0, others -1.0 to 1.0. Adjust if needed:
// //   if (throttle_axis < -0.1f || brake_axis < -0.1f) {
// //     // If axes are in [-1,1] range, normalize them to [0,1]
// //     forward = (throttle_axis + 1.0f) / 2.0f;
// //     reverse = (brake_axis   + 1.0f) / 2.0f;
// //   } else {
// //     // Assume already 0 to 1
// //     forward = (throttle_axis > 0.0f) ? throttle_axis : 0.0f;
// //     reverse = (brake_axis   > 0.0f) ? brake_axis   : 0.0f;
// //   }

// //   int esc_us = 1500; // start at neutral
// //   if (reverse > 0.05f) {
// //     // Brake/Reverse active – map to 1500 down to 1000 µs
// //     esc_us = (int)(1500 - reverse * 500);   // full reverse = ~1000 µs
// //   } else if (forward > 0.05f) {
// //     // Throttle active – map to 1500 up to 2000 µs
// //     esc_us = (int)(1500 + forward * 500);   // full throttle = ~2000 µs
// //   } 
// //   esc_us = constrain(esc_us, 1000, 2000);
// //   escServo.writeMicroseconds(esc_us);
// }

// void accelCallback(const void * msgin) {
// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
// Serial.printf("[ACCEL] data=%.3f\r\n", msg->data);
// }

// void decelCallback(const void * msgin) {
// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
// Serial.printf("[DECEL] data=%.3f\r\n", msg->data);
// }

// void joyxCallback(const void * msgin) {
// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
// Serial.printf("[JOYX] data=%.3f\r\n", msg->data);
// }

// void joyyCallback(const void * msgin) {
// const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
// Serial.printf("[JOYY] data=%.3f\r\n", msg->data);
// }

// // Micro-ROS variables
// rcl_allocator_t allocator;
// rclc_support_t support;
// rcl_publisher_t publisher;
// rcl_node_t node;
// rclc_executor_t executor;

// #define RCCHECK(fn, msg)                           \
//     {                                              \
//         rcl_ret_t temp_rc = fn;                    \
//         if ((temp_rc != RCL_RET_OK))               \
//         {                                          \
//             printf("err=%d %s\r\n", temp_rc, msg); \
//         }                                          \
//         return temp_rc;                            \
//     }
// #define RCSOFTCHECK(fn, msg)                       \
//     {                                              \
//         rcl_ret_t temp_rc = fn;                    \
//         if ((temp_rc != RCL_RET_OK))               \
//         {                                          \
//             printf("err=%d %s\r\n", temp_rc, msg); \
//         }                                          \
//         return temp_rc;                            \
//     }

// RplidarC1 lidar;

// void connect_wifi()
// {
//     WiFi.disconnect(true);  // Reset Wi-Fi
//     WiFi.mode(WIFI_STA);    // Set to Station mode
//     WiFi.begin(ssid, pass); // ssid and pass are defined in credentials.h
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(1000);
//         Serial.print(".");
//     }
//     Serial.println("\nConnected to Wi-Fi");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());
// }

// rcl_ret_t init_ros()
// {
//     // Micro-ROS initialization

//     rcl_ret_t ret;

//     struct my_micro_ros_agent_locator
//     {
//         IPAddress address;
//         int port;
//     } static locator;
//     locator.address = ros2_agent_ipa; // ros2_agent_ipa and ros2_agent_port are defined in credentials.h
//     locator.port = ros2_agent_port;

//     printf("rmw_uros_set_custom_transport...\r\n");
//     ret = rmw_uros_set_custom_transport(
//         false,
//         (void *)&locator,
//         platformio_transport_open,
//         platformio_transport_close,
//         platformio_transport_write,
//         platformio_transport_read);
//     if (RCL_RET_OK != ret)
//     {
//         printf("rmw_uros_set_custom_transport error=%d\r\n", ret);
//         return ret;
//     }

//     allocator = rcl_get_default_allocator();

//     printf("rclc_support_init...\r\n");
//     ret = rclc_support_init(&support, 0, NULL, &allocator);
//     if (RCL_RET_OK != ret)
//     {
//         printf("rclc_support_init error=%d\r\n", ret);
//         return ret;
//     }
//     printf("rclc_node_init_default...\r\n");
//     ret = rclc_node_init_default(&node, "lidar_node", "", &support);
//     if (RCL_RET_OK != ret)
//     {
//         printf("rclc_node_init_default error=%d\r\n", ret);
//         return ret;
//     }
//     // printf("rclc_publisher_init_default...\r\n");
//     // ret =
//     //     rclc_publisher_init_default(
//     //         &publisher,
//     //         &node,
//     //         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
//     //         "/scan");
//     // if (RCL_RET_OK != ret)
//     // {
//     //     printf("rclc_publisher_init_default error=%d\r\n", ret);
//     //     return ret;
//     // }

//     //     // create 20 msecs timer
//     //     // printf("create lidar timer...\r\n");
//     //     // const unsigned int lidar_timer_timeout = 200;
//     //     // ret = rclc_timer_init_default(
//     //     //   &timer,
//     //     //   &support,
//     //     //   RCL_MS_TO_NS(lidar_timer_timeout),
//     //     //   lidar_timer_callback);
//     //     // if(ret != RCL_RET_OK){
//     //     //   printf("rclc_timer_init_default lidar error=%d",ret);
//     //     //   return ret;
//     //     // }

//     //     // // create executor
//     //     // printf("create executor...\r\n");
//     //     // ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
//     //     // if(ret != RCL_RET_OK){
//     //     //   printf("rclc_executor_init error=&d\r\n",ret);
//     //     //   return ret;
//     //     // }

//     //     // printf("add  time to executor...\r\n");
//     //     // ret = rclc_executor_add_timer(&executor, &timer);
//     //     // if(RCL_RET_OK !=ret){
//     //     //   printf("rclc_executor_add_timer error=%d\r\n",ret);
//     //     //   return ret;
//     //     // }

//     return RCL_RET_OK;
// }

// // Setup function
// void setup()
// {
//     Serial.begin(115200); // Initialize Serial for debugging

//     connect_wifi();

//     if (RCL_RET_OK != init_ros())
//     {
//         printf("init_ros error. Rebooting ...\r\n");
//         esp_restart();
//     }

//     rclc_executor_init(&executor, &support.context, 5, &allocator);

//     // Joy subscription
//     rclc_subscription_init_default(
//         &joy_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
//         "/joy");
//     rclc_executor_add_subscription(&executor, &joy_sub, &joy_msg, &joyCallback, ON_NEW_DATA);

//     // Accel subscription
//     rclc_subscription_init_default(
//         &accel_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/accel");
//     rclc_executor_add_subscription(&executor, &accel_sub, &accel_msg, &accelCallback, ON_NEW_DATA);

//     // Decel subscription
//     rclc_subscription_init_default(
//         &decel_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/decel");
//     rclc_executor_add_subscription(&executor, &decel_sub, &decel_msg, &decelCallback, ON_NEW_DATA);

//     // JoyX subscription
//     rclc_subscription_init_default(
//         &joyx_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/joyx");
//     rclc_executor_add_subscription(&executor, &joyx_sub, &joyx_msg, &joyxCallback, ON_NEW_DATA);

//     // JoyY subscription
//     rclc_subscription_init_default(
//         &joyy_sub,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//         "/joyy");
//     rclc_executor_add_subscription(&executor, &joyy_sub, &joyy_msg, &joyyCallback, ON_NEW_DATA);

//     // // 4) Servo & ESC setup… // TODO: CAREFUL, was not tested (chatgpt generated)
//     // steeringServo.setPeriodHertz(50);
//     // steeringServo.attach(SERVO_PWM_PIN, 1000, 2000);
//     // escServo.setPeriodHertz(50);
//     // escServo.attach(ESC_PWM_PIN, 1000, 2000);
//     // escServo.writeMicroseconds(1500);
//     // delay(100);

//     // lidar.begin();
//     // delay(1000);
//     // lidar.resetLidar();
//     // delay(800);
//     // lidar.startLidar();
// }

// // rcl_ret_t ret;
// int main_loop_count = 0;
// unsigned long mil = 0L;
// // main timer callback
// // unsigned long start_uart;
// // unsigned long start_publishing;
// // unsigned long start_processing;
// unsigned long total_loop_time = 0L;
// float loop_period = 0.0;
// // int loop_count = 0;

// void loop_simple()
// {
//     int count = lidar.uartRx();
//     Serial.printf("got %d points\r\n", count);
//     // lidar.processFrame(count);
// }

// void loop()
// {
//     // Check Wi-Fi connection
//     if (WiFi.status() != WL_CONNECTED)
//     {
//         Serial.println("Wi-Fi disconnected, reconnecting...");
//         connect_wifi();
//     }

//     // unsigned long uart_elapsed = millis();
//     // int count = lidar.uartRx();
//     // uart_elapsed = millis() - uart_elapsed;

//     // unsigned long process_elapsed = millis();
//     // lidar.processFrame(count);
//     // process_elapsed = millis() - process_elapsed;

//     // unsigned long publish_elapsed = millis();
//     // rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
//     // publish_elapsed = millis() - publish_elapsed;

//     // if (ret_pub != RCL_RET_OK)
//     // {
//     //     printf("rcl_publish returned %d\r\n", ret_pub);
//     //     esp_restart();
//     // }

//     // // calculate loop period
//     // total_loop_time = millis() - total_loop_time;
//     // float total_loop_time_f = (float)total_loop_time;
//     // loop_period = loop_period * 0.9 + total_loop_time_f * 0.1;

//     // Serial.printf("got %d points in %lu ms. Frame Processing in %lu. Frame publishing in %lu. Total loop in %lu ms. Freq=%.1f Hz Serial2.available=%d\r\n",
//     //               count, uart_elapsed, process_elapsed, publish_elapsed, total_loop_time, 1000.0 / loop_period, Serial2.available());
//     // total_loop_time = millis();

//     rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
//     if (rc != RCL_RET_OK) {
//         printf("rclc_executor_spin_some error=%d\r\n", rc);
//         // esp_restart();
//     }
//     // delay(30); // TODO: should we put delay ???
// }