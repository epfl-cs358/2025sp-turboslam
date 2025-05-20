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

#include <sensor_msgs/msg/joy.h>        // ROS2 Joy message type
#include <std_msgs/msg/int32.h>        // ROS2 Int32 message type
#include <std_msgs/msg/float32.h>

#define TEST_IMU         0
#define TEST_ULTRASONIC  0
#define TEST_ENCODER     0
#define TEST_SERVO_DIR   0
#define TEST_SERVO_LID   0

// ESC signal pins
constexpr int escSignalPin = 15;
constexpr int steeringServoPin = 26;

// Subscriptions
auto joy_sub       = rcl_get_zero_initialized_subscription();
auto accel_sub     = rcl_get_zero_initialized_subscription();
auto decel_sub     = rcl_get_zero_initialized_subscription();
auto joyx_sub      = rcl_get_zero_initialized_subscription();
auto joyy_sub      = rcl_get_zero_initialized_subscription();

// sensor_msgs__msg__Joy         joy_msg;
// std_msgs__msg__Float32        accel_msg;
// std_msgs__msg__Float32        decel_msg;
// std_msgs__msg__Float32        joyx_msg;
// std_msgs__msg__Float32        joyy_msg;

void joyCallback(const void * msgin) {
// //   Serial.println("[JOY] joyCallback()");  
// //   const sensor_msgs__msg__Joy * joy = (const sensor_msgs__msg__Joy *)msgin;
//     const auto *joy = static_cast<const sensor_msgs__msg__Joy *>(msgin);



//   // Read axes from Joy message
//   float steer_axis  = joy->axes.data[0];   // left/right stick for steering
//   float brake_axis  = joy->axes.data[2];   // LT trigger for brake/reverse
//   float throttle_axis = joy->axes.data[5]; // RT trigger for throttle

//   Serial.printf("[JOY] steer=%.2f, throttle=%.2f, brake=%.2f\n",
//     steer_axis, throttle_axis, brake_axis);
//     if (fabsf(steer_axis) < 0.05f) {
//         Serial.printf("[JOY] steer=%.2f\n", steer_axis);
//         steer_axis = 0.0f;
//     }

//     int steer_deg = (int)(90 + steer_axis * 10); // Map to servo angle
//     steer_deg = constrain(steer_deg, 85, 95); 
//     servo_dir.setAngle(steer_deg);

//     // motor command
//     float forward = throttle_axis > 0 ? throttle_axis : 0;
//     float reverse = brake_axis > 0 ? brake_axis : 0;
//     float cmd_percent = forward - reverse;
//     cmd_percent = constrain(cmd_percent, -0.3f, 0.3f);

//     motor.setTargetPercent(cmd_percent);

//     Serial.printf(
//     "[JOY] steer=%.2f (deg=%d), throttle=%.2f, brake=%.2f → pct=%.2f\n",
//     steer_axis, steer_deg, throttle_axis, brake_axis, cmd_percent);


//   // Map steering axis (-1 to +1) to servo pulse width (1000 to 2000 µs)
//   int steering_us = (int)(1500 + steer_axis * 500);  // centered at 1500 µs, ±500 µs range
//   steering_us = constrain(steering_us, 1400, 1600);  // clamp to [1000,2000] just in case
//   steeringServo.writeMicroseconds(steering_us);

//   // Determine throttle/reverse for ESC. Neutral is 1500 µs.
//   float forward = 0.0f;
//   float reverse = 0.0f;
//   // Some controllers report triggers 0.0 to 1.0, others -1.0 to 1.0. Adjust if needed:
//   if (throttle_axis < -0.1f || brake_axis < -0.1f) {
//     // If axes are in [-1,1] range, normalize them to [0,1]
//     forward = (throttle_axis + 1.0f) / 2.0f;
//     reverse = (brake_axis   + 1.0f) / 2.0f;
//   } else {
//     // Assume already 0 to 1
//     forward = (throttle_axis > 0.0f) ? throttle_axis : 0.0f;
//     reverse = (brake_axis   > 0.0f) ? brake_axis   : 0.0f;
//   }

//   int esc_us = 1500; // start at neutral
//   if (reverse > 0.05f) {
//     // Brake/Reverse active – map to 1500 down to 1000 µs
//     esc_us = (int)(1500 - reverse * 500);   // full reverse = ~1000 µs
//   } else if (forward > 0.05f) {
//     // Throttle active – map to 1500 up to 2000 µs
//     esc_us = (int)(1500 + forward * 500);   // full throttle = ~2000 µs
//   } 
//   esc_us = constrain(esc_us, 1000, 2000);
//   escServo.writeMicroseconds(esc_us);
}

void accelCallback(const void * msgin) {
const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
Serial.printf("[ACCEL] data=%.3f\r\n", msg->data);
}

void decelCallback(const void * msgin) {
const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
Serial.printf("[DECEL] data=%.3f\r\n", msg->data);
}

void joyxCallback(const void * msgin) {
const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
Serial.printf("[JOYX] data=%.3f\r\n", msg->data);
}

void joyyCallback(const void * msgin) {
const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
Serial.printf("[JOYY] data=%.3f\r\n", msg->data);
}

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
DMS15 servo_dir(steeringServoPin); // servo used for the direcition of the car
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
        printf("Wifi status: %d\n",  WiFi.status());
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
void motor_callback(const void* msgin) {
    const auto *cmd = static_cast<const std_msgs__msg__Int8*>(msgin);
    motor.setTargetPercent(float(cmd->data));
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
void motorTask(void *parameter);


// Setup function
void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    // bool initMotor = motor.begin();
    WiFi.localIP();
    connect_wifi();

    // Initialize the steering servo
    if (!servo_dir.begin()) {
        Serial.println("Servo_dir failed to initialize, rebooting...");
        esp_restart();
    }
    // Set it to center (90 degrees)
    servo_dir.setAngle(90);


   
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

    BaseType_t motor_task = xTaskCreatePinnedToCore(motorTask, "Motor Task", 4096, NULL, 5, NULL, 1);
        if (motor_task != pdPASS) {
            Serial.println("Failed to create IMU Task");
            esp_restart();
        }
        if (!motor.begin()) {
            Serial.println("IMU failed to initialize, rebooting...");
            esp_restart();
        }

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

    // BaseType_t servoTaskCreated = xTaskCreatePinnedToCore(servoPublisherTask, "ServoPub", 4096, NULL, 1, NULL, 0);
    // if (servoTaskCreated != pdPASS) {
    //     Serial.println("Failed to create Servo Publisher Task");
    //     esp_restart();
    // }

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

void motorTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(UPDATE_PERIOD_MS);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        motor.update();
        xTaskDelayUntil(&lastWake, period);
    }
    uxTaskGetStackHighWaterMark(NULL);
}

void loop() {
  
}




// #include <Arduino.h>
// #include "MotorController.h"

// // ESC signal pin
// constexpr int escSignalPin = 15;

// // Create our MotorController
// MotorController motor(escSignalPin);

// void setup() {
//     pinMode(LED_BUILTIN, OUTPUT);

//     Serial.begin(115200);
//     for (int i = 0; i < 6; i++) {
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(200);
//         digitalWrite(LED_BUILTIN, LOW);
//         delay(200);
//     }
//     Serial.println("Hello from setup!");
//     while (!Serial) {}  
//     Serial.println("\nREADY: f=forward, r=reverse, s=stop");

//     // Attach & arm ESC
//     if (!motor.begin()) {
//       Serial.println("ERROR: ESC failed to attach!");
//       while (1) delay(100);
//     }
//     Serial.println("ESC armed (neutral).");
// }

// void loop() {
//     if (!Serial.available()) {
//       delay(10);
//       return;
//     }

//     char c = Serial.read();
//     switch (c) {
//       case 'w': case 'W':
//         Serial.println("→ FORWARD");
//         motor.command(1650);  // 2 ms pulse = full forward
//         break;
//       case 's': case 'S':
//         Serial.println("← REVERSE");
//         motor.command(1350);  // 1 ms pulse = full reverse
//         break;
//       case 'b': case 'B':
//         Serial.println("⏸ STOP");
//         motor.command(1500);  // 1.5 ms pulse = neutral/brake
//         break;
//       default:
//         // ignore any other character
//         break;
//     }
//   }



