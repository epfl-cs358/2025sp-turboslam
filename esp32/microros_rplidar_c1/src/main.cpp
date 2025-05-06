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

#include "credentials.h"
//#include "RplidarC1.h"

#include "ImuSensor.h"

#include "UltraSonicSensor.h"

#include "AS5600Encoder.h"

#include <std_msgs/msg/int32.h>
#include "DMS15.h"

// Micro-ROS variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_node_t node;

// Lidar
// RplidarC1 lidar;

// IMU
ImuSensor imuSensor;
rcl_publisher_t imu_publisher;

// Ultrasonic sensor
UltraSonicSensor ultrasonic(12, 14);  
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
    // printf("rclc_publisher_init_default...\r\n");
    // ret = rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    //     "/scan");  
    // if (RCL_RET_OK != ret){
    //     printf("rclc_publisher_init_default error=%d\r\n",ret);
    //     return ret;        
    // }

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

    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);  // 2 = number of handles (subscribers)
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize executor");
        return ret;
    }

    // Add subscriptions to executor
    // Servo dir
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
  
    return RCL_RET_OK;
}

// Setup function
void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    connect_wifi();
   
    if(RCL_RET_OK != init_ros()){
      printf("init_ros error. Rebooting ...\r\n");
      esp_restart();
    }
    
    // lidar.begin();
    // delay(1000);
    // lidar.resetLidar();
    // delay(800);
    // lidar.startLidar();

    if (!imuSensor.begin()) {
        Serial.println("IMU failed to initialize, rebooting...");
        esp_restart();
    }

    if (!ultrasonic.begin()) {
        Serial.println("Ultrasonic sensor failed to initialize, rebooting...");
        esp_restart();
    }

    if (!encoder.begin()) {
        Serial.println("Encoder failed to initialize, rebooting...");
        esp_restart();
    }

    if(!servo_dir.begin()) {
        Serial.println("Servo_dir failed to initialize, rebooting...");
        esp_restart();
    }

    if(!servo_lid.begin()) {
        Serial.println("Servo_lib failed to initialize, rebooting...");
        esp_restart();
    }
}

//rcl_ret_t ret;
int main_loop_count = 0;
unsigned long mil = 0L;
unsigned long total_loop_time = 0L;
float loop_period = 0.0;
// int loop_count = 0;

// void loop_simple() {
//     int count = lidar.uartRx();
//     Serial.printf("got %d points\r\n", count);
//     //lidar.processFrame(count);
// }

void loop() {
    // Check Wi-Fi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi disconnected, reconnecting...");
        connect_wifi();
    }

    // unsigned long uart_elapsed = millis();
    // int count = lidar.uartRx();
    // uart_elapsed = millis() - uart_elapsed;

    // unsigned long process_elapsed = millis();
    // lidar.processFrame(count);
    // process_elapsed = millis()-process_elapsed;

    // unsigned long publish_elapsed = millis();
    // rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
    // publish_elapsed = millis() - publish_elapsed;

    // if(ret_pub != RCL_RET_OK){
    //     printf("rcl_publish returned %d\r\n", ret_pub);
    //     esp_restart();
    // }

    // calculate loop period  
    // total_loop_time = millis()-total_loop_time;
    // float total_loop_time_f = (float) total_loop_time;
    // loop_period = loop_period*0.9 + total_loop_time_f*0.1;
    
    // Serial.printf("got %d points in %lu ms. Frame Processing in %lu. Frame publishing in %lu. Total loop in %lu ms. Freq=%.1f Hz Serial2.available=%d\r\n",
    //     count, uart_elapsed, process_elapsed, publish_elapsed, total_loop_time, 1000.0/loop_period, Serial2.available() );
    // total_loop_time = millis();

    // Publish IMU data
    imuSensor.readAndUpdate();
    rcl_ret_t ret_imu = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret_imu != RCL_RET_OK) {    // Publish IMU data
        imuSensor.readAndUpdate();
        rcl_ret_t ret_imu = rcl_publish(&imu_publisher, &imu_msg, NULL);
        if (ret_imu != RCL_RET_OK) {
            printf("rcl_publish IMU error=%d\r\n", ret_imu);
        }
    
        //US data
        float dist = ultrasonic.readDistance();
        if (dist > 0) {
            ultrasonic.range_msg.range = dist;
            ultrasonic.range_msg.header.stamp.sec = millis() / 1000;
            ultrasonic.range_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
            rcl_ret_t ret_range = rcl_publish(&range_publisher, &ultrasonic.range_msg, NULL);
            if (ret_range != RCL_RET_OK) {
                printf("rcl_publish range error=%d\r\n", ret_range);
            }
        }
    }

    // Encoder data
    encoder.update();
    rcl_ret_t ret_range = rcl_publish(&encoder_publisher, &encoder.angle_msg, NULL);
    if (ret_range != RCL_RET_OK) {
        printf("rcl_publish encoder error=%d\r\n", ret_range);
    }
    printf("rcl_publish IMU error=%d\r\n", ret_imu);

    //US data
    float dist = ultrasonic.readDistance();
    if (dist > 0) {
        ultrasonic.range_msg.range = dist;
        ultrasonic.range_msg.header.stamp.sec = millis() / 1000;
        ultrasonic.range_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
        rcl_ret_t ret_range = rcl_publish(&range_publisher, &ultrasonic.range_msg, NULL);
        if (ret_range != RCL_RET_OK) {
            printf("rcl_publish range error=%d\r\n", ret_range);
        }
    }

    // Encoder data
    encoder.update();
    ret_range = rcl_publish(&encoder_publisher, &encoder.angle_msg, NULL);
    if (ret_range != RCL_RET_OK) {
        printf("rcl_publish encoder error=%d\r\n", ret_range);
    }

    // Servo_lid tilting call
    servo_lid.tiltLidar(70, 110, 4000);
    servo_angle_msg.data = servo_lid.getAngle();
    ret_range = rcl_publish(&servo_angle_publisher, &servo_angle_msg, NULL);
    if (ret_range != RCL_RET_OK) {
        printf("rcl_publish servo_lid error=%d\r\n", ret_range);
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    delay(30);
}

