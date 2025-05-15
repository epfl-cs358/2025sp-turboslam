#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <ICM_20948.h>
#include <AS5600.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int8.h>

#include "MotorController.h"

#include "credentials.h"
//#include "RplidarC1.h"

#include "ImuSensor.h"

#include "UltraSonicSensor.h"

#include "AS5600Encoder.h"

#include <std_msgs/msg/int32.h>
#include "DMS15.h"

#define TEST_IMU         1
#define TEST_ULTRASONIC  1
#define TEST_ENCODER     1
#define TEST_SERVO_DIR   1
#define TEST_SERVO_LID   1

// ESC signal pins
constexpr int escSignalPin = 15;

// Micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_node_t node;

rcl_subscription_t motor_cmd_subscriber;
std_msgs__msg__Int8 motor_cmd_msg;

// Motor controller
MotorController motor(escSignalPin);

// IMU
ImuSensor imuSensor;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

// Ultrasonic sensor
UltraSonicSensor ultrasonic(25, 12);  
rcl_publisher_t range_publisher;

// Encoder
AS5600Encoder encoder;
rcl_publisher_t encoder_publisher;

//Servo
//To test run : $ ros2 topic pub /servo_lid/angle std_msgs/msg/Int32 "{data: 90}" 
DMS15 servo_dir(26); // servo used for the direcition of the car
rcl_subscription_t servo_dir_subscriber;
std_msgs__msg__Int32 servo_dir_angle_msg;
 
DMS15 servo_lid(27); // servo used for tilting the lidar
rcl_subscription_t servo_lid_subscriber;
std_msgs__msg__Int32 servo_lid_angle_msg;
// Servo_lid angle publisher
// To test run : $ ros2 topic echo /lidar_servo_angle
rcl_publisher_t servo_angle_publisher;
std_msgs__msg__Int32 servo_angle_msg;

rclc_executor_t executor;

QueueHandle_t servoAngleQ;

#define RCCHECK(fn, msg)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}return temp_rc;}
#define RCSOFTCHECK(fn, msg) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("err=%d %s\r\n",temp_rc,msg);}return temp_rc;}

void connect_wifi(){
    WiFi.disconnect(true);   // Reset Wi-Fi
    WiFi.mode(WIFI_STA);     // Set to Station mode
    WiFi.begin(ssid, pass);  // ssid and pass are defined in credentials.h
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

// Motor callback
void motor_callback(const void * msgin) {
    auto cmd = static_cast<const std_msgs__msg__Int8*>(msgin);
    motor.command(cmd->data, 200);
    printf("Motor cmd: %d\n", cmd->data);
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
    printf("rmw_uros_set_custom_transport ret=%d\r\n",ret);
    if (RCL_RET_OK != ret){
      printf("rmw_uros_set_custom_transport error=%d\r\n",ret);
      return ret;
    }
    
    allocator = rcl_get_default_allocator();

    printf("rclc_support_init...\r\n");
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    printf("rclc_support_init ret=%d\n", ret);
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

    // IMU
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

    // US sensor
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

    // Encoder 
    ret = rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/encoder/angle"
    );
    if (ret != RCL_RET_OK) {
        printf("encoder publisher init failed: %d\n", ret);
        return ret;
    }

    // Servo_lid puvlisher to get the angle
    ret = rclc_publisher_init_default(
        &servo_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "lidar_servo_angle"
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create servo_angle publisher: %d\n", ret);
        return ret;
    }

    //Servo dir
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

    //Servo lid
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

    ret = rclc_executor_init(&executor, &support.context, 3, &allocator);  // 2 = number of handles (subscribers)
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize executor");
        return ret;
    }

    // Add subscriptions to executor
    //Servo dir
    ret = rclc_executor_add_subscription(
        &executor,
        &servo_dir_subscriber,
        &servo_dir_angle_msg,
        &servo_dir_callback,
        ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        printf("Failed to add servo_dir callback to executor: %d\n", ret);
        return ret;
    }

    // Servo lid
    ret = rclc_executor_add_subscription(
        &executor,
        &servo_lid_subscriber,
        &servo_lid_angle_msg,
        &servo_lid_callback,
        ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        printf("Failed to add servo_lid callback to executor: %d\n", ret);
        return ret;
    }

    // Motor control subscriber
    ret = rclc_subscription_init_default(
        &motor_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/motor_cmd"
    );

    if (ret) return ret;

    // Add it to the executor
    // ret = rclc_executor_add_subscription(
    //     &executor,
    //     &motor_cmd_subscriber,
    //     &motor_cmd_msg,
    //     &motor_callback,
    //     ON_NEW_DATA);
    // if (ret != RCL_RET_OK) {
    //     printf("Failed to add motor callback to executor: %d\n", ret);
    //     return ret;
    // }
    RCCHECK(
        rclc_executor_add_subscription(
            &executor,
            &motor_cmd_subscriber,
            &motor_cmd_msg,
            &motor_callback,
            ON_NEW_DATA),
        "Failed to add motor callback to executor");
  
    return RCL_RET_OK;
}

void imuTask(void *parameter);
void encoderTask(void *parameter);
void ultrasonicTask(void *parameter);
void servoLidTask(void *parameter);
void executorTask(void *parameter);
void wifiMonitorTask(void *parameter);
void servoPublisherTask(void *parameter);

// Setup function
void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    motor.begin();
    WiFi.localIP();
    connect_wifi();
    printf("Free heap: %d\n", esp_get_free_heap_size());

    if (!motor.begin()) {
        Serial.println("Motor failed to initialize, rebooting...");
        esp_restart();
    }


   
    if (RCL_RET_OK != init_ros()) {
        printf("init_ros failed. Rebooting ...\r\n");
        esp_restart();
    } else {
        Serial.println("init_ros succeeded");
    }

    // if(!servo_dir.begin()) {
    //     Serial.println("Servo_dir failed to initialize, rebooting...");
    //     esp_restart();
    // }

    servoAngleQ = xQueueCreate(1, sizeof(int));

    #if TEST_IMU
        BaseType_t imuTaskCreated = xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 1, NULL, 1);
        if (imuTaskCreated != pdPASS) {
            Serial.println("Failed to create IMU Task");
            esp_restart();
        }
        if (!imuSensor.begin()) {
            Serial.println("IMU failed to initialize, rebooting...");
            esp_restart();
        }
    #endif

    #if TEST_ENCODER
        BaseType_t encoderTaskCreated = xTaskCreatePinnedToCore(encoderTask, "Encoder Task", 4096, NULL, 1, NULL, 1);
        if (encoderTaskCreated != pdPASS) {
            Serial.println("Failed to create Encoder Task");
            esp_restart();
        }
        if (!encoder.begin()) {
            Serial.println("Encoder failed to initialize, rebooting...");
            esp_restart();
        }
    #endif

    #if TEST_ULTRASONIC
        BaseType_t ultrasonicTaskCreated = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic Task", 4096, NULL, 1, NULL, 1); 
        if (ultrasonicTaskCreated != pdPASS) {
            Serial.println("Failed to create Ultrasonic Task");
            esp_restart();
        }
        if (!ultrasonic.begin()) {
            Serial.println("Ultrasonic sensor failed to initialize, rebooting...");
            esp_restart();
        }
    #endif
    
    #if TEST_SERVO_LID
        BaseType_t servoLidTaskCreated = xTaskCreatePinnedToCore(servoLidTask, "Servo Lid Task", 4096, NULL, 3, NULL, 1);
        if (servoLidTaskCreated != pdPASS) {
            Serial.println("Failed to create Servo Lid Task");
            esp_restart();
        }
        if(!servo_lid.begin()) {
            Serial.println("Servo_lib failed to initialize, rebooting...");
            esp_restart();
        }
    #endif

    BaseType_t servoTaskCreated = xTaskCreatePinnedToCore(servoPublisherTask, "ServoPub", 4096, NULL, 1, NULL, 0);
    if (servoTaskCreated != pdPASS) {
        Serial.println("Failed to create Servo Publisher Task");
        esp_restart();
    }

    BaseType_t executorTaskCreated = xTaskCreatePinnedToCore(executorTask, "Executor Task", 4096, NULL, 2, NULL, 0);  // Higher priority
    if (executorTaskCreated != pdPASS) {
        Serial.println("Failed to create Executor Task");
        esp_restart();
    }
    BaseType_t wifiMonitorTaskCreated = xTaskCreatePinnedToCore(wifiMonitorTask, "WiFi Monitor", 2048, NULL, 1, NULL, 1);
    if (wifiMonitorTaskCreated != pdPASS) {
        Serial.println("Failed to create WiFi Monitor Task");
        esp_restart();
    }
}

void imuTask(void *parameter) {
    while (true) {
        imuSensor.readAndUpdate();
        rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("rcl_publish IMU error=%d\r\n", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz
    }
    uxTaskGetStackHighWaterMark(NULL);
}

void encoderTask(void *parameter) {
    while (true) {
        encoder.update();
        rcl_ret_t ret = rcl_publish(&encoder_publisher, &encoder.angle_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("rcl_publish encoder error=%d\r\n", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
    uxTaskGetStackHighWaterMark(NULL);
}

void ultrasonicTask(void *parameter) {
    while (true) {
        float dist = ultrasonic.readDistance();
        if (dist > 0) {
            ultrasonic.range_msg.range = dist;
            ultrasonic.range_msg.header.stamp.sec = millis() / 1000;
            ultrasonic.range_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
            rcl_ret_t ret = rcl_publish(&range_publisher, &ultrasonic.range_msg, NULL);
            if (ret != RCL_RET_OK) {
                printf("rcl_publish ultrasonic error=%d\r\n", ret);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
    }
    uxTaskGetStackHighWaterMark(NULL);
}

void servoLidTask(void *parameter) {
    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t lastWake = xTaskGetTickCount();
  
    while (true) {
      // compute next angle
      servo_lid.tiltLidar(70, 110, 1000);
      int angle = servo_lid.getAngle();
  
      xQueueOverwrite(servoAngleQ, &angle);
      vTaskDelayUntil(&lastWake, period);
    }
}

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
        rcl_ret_t ret = rcl_publish(&servo_angle_publisher, &msg, nullptr);
        if (ret != RCL_RET_OK) {
            printf("rcl_publish servo_lid error=%d\r\n", ret);
        }

        vTaskDelayUntil(&lastWake, period);
    }
    uxTaskGetStackHighWaterMark(NULL);
}

void executorTask(void *parameter) {
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
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
    uxTaskGetStackHighWaterMark(NULL);
}

void loop() {
    // Empty loop
}




