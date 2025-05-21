#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <ICM_20948.h>
#include <AS5600.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>        // ROS2 IMU message type
#include <std_msgs/msg/int8.h>          // ROS2 Int8 message type

#include "MotorController.h"

#include "credentials.h"
//#include "RplidarC1.h"

#include "ImuSensor.h"

#include "UltraSonicSensor.h"

#include "AS5600Encoder.h"

#include <std_msgs/msg/int32.h>
#include "DMS15.h"

#include <sensor_msgs/msg/joy.h>        // ROS2 Joy message type
// Removed duplicate #include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define TEST_IMU         0
#define TEST_ULTRASONIC  0
#define TEST_ENCODER     0
#define TEST_SERVO_DIR   0
#define TEST_SERVO_LID   0

// ESC signal pins
constexpr int escSignalPin = 15;
constexpr int steeringServoPin = 6;

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
#define RCSOFTCHECK(fn, msg) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Warning: err=%d %s\r\n",temp_rc,msg);}}

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
   
    struct AgentLocator {
      IPAddress address;
      int port;
    } static locator;
    locator.address = ros2_agent_ipa; // ros2_agent_ipa and ros2_agent_port are defined in credentials.h
    locator.port = ros2_agent_port;

    printf("rmw_uros_set_custom_transport...\r\n");
    RCCHECK(rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    ), "rmw_uros_set_custom_transport error");
    
    printf("rmw_uros_set_custom_transport ret=%d\r\n",ret);
    
    
    allocator = rcl_get_default_allocator();

    printf("rclc_support_init...\r\n");
    // ret = rclc_support_init(&support, 0, NULL, &allocator);
    RCCHECK(rclc_support_init(
        &support,
        0,
        NULL,
        &allocator
    ), "rclc_support_init error");
    printf("rclc_support_init ret=%d\n", ret);
    
    printf("rclc_node_init_default...\r\n");

    RCCHECK(rclc_node_init_default(
        &node, 
        "lidar_node", 
        "", 
        &support), 
        "rclc_node_init_default error");
    

    // IMU
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu"
    ), "imu publisher init error");


    // US sensor
    RCCHECK(rclc_publisher_init_default(
        &range_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonic"
    ), "ultrasonic publisher init failed");
    

    // Encoder 
    RCCHECK(rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/encoder/angle"
    ), "encoder publisher init failed");
    

    // Servo_lid publisher to get the angle
    RCCHECK(rclc_publisher_init_default(
        &servo_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "lidar_servo_angle"
    ), "Failed to create servo_angle publisher");
    

    //Servo dir
    RCCHECK(rclc_subscription_init_default(
        &servo_dir_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/servo_dir/angle"
    ), "Failed to create servo_dir subscriber");
    

    //Servo lid
    RCCHECK(rclc_subscription_init_default(
        &servo_lid_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_lid/angle"
    ), "Failed to create a servo_lid subscriber");
   

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator), "Failed to initialize executor");  // 2 = number of handles (subscribers)
    

    // Add subscriptions to executor
    //Servo dir
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &servo_dir_subscriber,
        &servo_dir_angle_msg,
        &servo_dir_callback,
        ON_NEW_DATA), "Failed to add servo_dir callback to executor");
    

    // Servo lid
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &servo_lid_subscriber,
        &servo_lid_angle_msg,
        &servo_lid_callback,
        ON_NEW_DATA), "Failed to add servo_lid callback to executor");
    

    // Motor control subscriber
    RCCHECK(rclc_subscription_init_default(
        &motor_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/motor_cmd"
    ), "Failed to initialize motor control subscriber");

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
static float targetPercentage  = 0.0f;

// Shared state
static float targetPct  = 0.0f;
static int   steerAngle = 90;

// Serial‐driven commands
void serialTask(void *) {

    // Drive & steer parameters
    static constexpr float DRIVE_POWER = 0.3f;  // 30% throttle/reverse
    static constexpr int   STEER_STEP  = 5;     // degrees per press
    static constexpr int   STEER_MIN   = 45;
    static constexpr int   STEER_MAX   = 135;
    static constexpr int   STEER_CENTRE= 90;
    // State
    float targetPct  = 0.0f;
    int   steerAngle = STEER_CENTRE;

    // Key press tracking
    bool wKeyPressed = false;
    bool sKeyPressed = false;
    unsigned long lastKeyActivity = 0;
    constexpr unsigned long KEY_TIMEOUT_MS = 500; // Time before assuming key was released


    while (true) {
       // 1) Handle Serial input
      if (Serial.available()) {
          char c = tolower(Serial.read());
          lastKeyActivity = millis();

          switch (c) {
              case 'w':
                  wKeyPressed = true;
                  sKeyPressed = false;
                  targetPct = DRIVE_POWER;
                  Serial.println("Forward - holding...");
                  break;
              case 's':
                  sKeyPressed = true;
                  wKeyPressed = false;
                  targetPct = -DRIVE_POWER;
                  Serial.println("Reverse - holding...");
                  break;
              case 'x':
                  wKeyPressed = false;
                  sKeyPressed = false;
                  targetPct = 0.0f;
                  Serial.println("Emergency STOP");
                  break;
              case 'a':
                  steerAngle = max(STEER_MIN, steerAngle - STEER_STEP);
                  servo_dir.setAngle(steerAngle);
                  Serial.printf("Steer LEFT → %d°\n", steerAngle);
                  break;
              case 'd':
                  steerAngle = min(STEER_MAX, steerAngle + STEER_STEP);
                  servo_dir.setAngle(steerAngle);
                  Serial.printf("Steer RIGHT → %d°\n", steerAngle);
                  break;
              default:
                  // ignore
                  break;
        }
        // update motor target
        motor.setTargetPercent(targetPct);
    }

    // 2) Check for key release (timeout since last keypress)
    if ((wKeyPressed || sKeyPressed) && (millis() - lastKeyActivity > KEY_TIMEOUT_MS)) {
        wKeyPressed = false;
        sKeyPressed = false;
        targetPct = 0.0f;
        motor.setTargetPercent(targetPct);
        Serial.println("Key released - stopping");
    }

    // 3) Ramp ESC toward target
    motor.update();

    // 4) Periodic status
    static unsigned long lastStatusTs = 0;
    if (millis() - lastStatusTs > 250) {
        lastStatusTs = millis();
        Serial.printf("Status → drive=%.2f  steer=%d°  esc_us=%dµs\n",
                                    targetPct, steerAngle, motor.currentUs());
    }

    delay(10);
  }
}

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

    BaseType_t serialCreated = xTaskCreatePinnedToCore(
        serialTask, "SerialCmd", 2048, NULL, 3, NULL, 1);
    if (serialCreated != pdPASS) {
        Serial.println("Failed to create serialTask");
        esp_restart();
    }
   
    if (RCL_RET_OK != init_ros()) {
        printf("init_ros failed. Rebooting ...\r\n");
        esp_restart();
    } else {
        Serial.println("init_ros succeeded");
    }

   

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








// #include <Arduino.h>
// #include "MotorController.h"
// #include "DMS15.h"

// // Pins
// constexpr int ESC_PIN      = 15;
// constexpr int STEER_PIN    = 6;

// // Drive & steer parameters
// static constexpr float DRIVE_POWER = 0.3f;  // 30% throttle/reverse
// static constexpr int   STEER_STEP  = 5;     // degrees per press
// static constexpr int   STEER_MIN   = 45;
// static constexpr int   STEER_MAX   = 135;
// static constexpr int   STEER_CENTRE= 90;

// // Objects
// MotorController motor(ESC_PIN);
// DMS15            servo_dir(STEER_PIN);

// // State
// float targetPct  = 0.0f;
// int   steerAngle = STEER_CENTRE;

// // Key press tracking
// bool wKeyPressed = false;
// bool sKeyPressed = false;
// unsigned long lastKeyActivity = 0;
// constexpr unsigned long KEY_TIMEOUT_MS = 500; // Time before assuming key was released

// void setup() {
//     pinMode(LED_BUILTIN, OUTPUT);

//     Serial.begin(115200);
//     while (!Serial) { delay(10); }

//     // LED blink to show startup
//     for (int i = 0; i < 3; i++) {
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(200);
//         digitalWrite(LED_BUILTIN, LOW);
//         delay(200);
//     }

//     Serial.println("=== Motor + Steering Serial Control ===");
//     Serial.println("Hold W: forward (releases to stop)");
//     Serial.println("Hold S: reverse (releases to stop)");
//     Serial.println("X: emergency stop");
//     Serial.println("A: steer left, D: steer right\n");

//     // Motor init
//     if (!motor.begin()) {
//         Serial.println("ERROR: ESC attach failed!");
//         while (1) delay(1000);
//     }
//     Serial.println("Motor ready (neutral).");

//     // Steering init
//     if (!servo_dir.begin()) {
//         Serial.println("ERROR: Steering servo attach failed!");
//         while (1) delay(1000);
//     }
//     servo_dir.setAngle(steerAngle);
//     Serial.println("Steering ready (90°).");
// }

// void loop() {
//     // 1) Handle Serial input
//     if (Serial.available()) {
//         char c = tolower(Serial.read());
//         lastKeyActivity = millis();
        
//         switch (c) {
//             case 'w':
//                 wKeyPressed = true;
//                 sKeyPressed = false;
//                 targetPct = DRIVE_POWER;
//                 Serial.println("Forward - holding...");
//                 break;
//             case 's':
//                 sKeyPressed = true;
//                 wKeyPressed = false;
//                 targetPct = -DRIVE_POWER;
//                 Serial.println("Reverse - holding...");
//                 break;
//             case 'x':
//                 wKeyPressed = false;
//                 sKeyPressed = false;
//                 targetPct = 0.0f;
//                 Serial.println("Emergency STOP");
//                 break;
//             case 'a':
//                 steerAngle = max(STEER_MIN, steerAngle - STEER_STEP);
//                 servo_dir.setAngle(steerAngle);
//                 Serial.printf("Steer LEFT → %d°\n", steerAngle);
//                 break;
//             case 'd':
//                 steerAngle = min(STEER_MAX, steerAngle + STEER_STEP);
//                 servo_dir.setAngle(steerAngle);
//                 Serial.printf("Steer RIGHT → %d°\n", steerAngle);
//                 break;
//             default:
//                 // ignore
//                 break;
//         }
//         // update motor target
//         motor.setTargetPercent(targetPct);
//     }

//     // 2) Check for key release (timeout since last keypress)
//     if ((wKeyPressed || sKeyPressed) && (millis() - lastKeyActivity > KEY_TIMEOUT_MS)) {
//         wKeyPressed = false;
//         sKeyPressed = false;
//         targetPct = 0.0f;
//         motor.setTargetPercent(targetPct);
//         Serial.println("Key released - stopping");
//     }

//     // 3) Ramp ESC toward target
//     motor.update();

//     // 4) Periodic status
//     static unsigned long lastStatusTs = 0;
//     if (millis() - lastStatusTs > 250) {
//         lastStatusTs = millis();
//         Serial.printf("Status → drive=%.2f  steer=%d°  esc_us=%dµs\n",
//                                     targetPct, steerAngle, motor.currentUs());
//     }

//     delay(10);
// }




