#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "credentials.h"
#include "RplidarC1.h"

#include <ESP32Servo.h>                 // Arduino-compatible servo PWM library for ESP32
#include <sensor_msgs/msg/joy.h>        // ROS2 Joy message type
#include <std_msgs/msg/int32.h>        // ROS2 Int32 message type
#include <std_msgs/msg/float32.h>

#define ESC_PWM_PIN    15    // GPIO15 for throttle ESC signal
#define SERVO_PWM_PIN   6    // GPIO6 for steering servo signal

Servo escServo;
Servo steeringServo;

// Subscriptions
auto joy_sub       = rcl_get_zero_initialized_subscription();
auto accel_sub     = rcl_get_zero_initialized_subscription();
auto decel_sub     = rcl_get_zero_initialized_subscription();
auto joyx_sub      = rcl_get_zero_initialized_subscription();
auto joyy_sub      = rcl_get_zero_initialized_subscription();

sensor_msgs__msg__Joy         joy_msg;
std_msgs__msg__Float32        accel_msg;
std_msgs__msg__Float32        decel_msg;
std_msgs__msg__Float32        joyx_msg;
std_msgs__msg__Float32        joyy_msg;

void joyCallback(const void * msgin) {
  Serial.println("[JOY] joyCallback()");  
  const sensor_msgs__msg__Joy * joy = (const sensor_msgs__msg__Joy *)msgin;

  // Read axes from Joy message
  float steer_axis  = joy->axes.data[0];   // left/right stick for steering
  float brake_axis  = joy->axes.data[2];   // LT trigger for brake/reverse
  float throttle_axis = joy->axes.data[5]; // RT trigger for throttle

  Serial.printf("[JOY] steer=%.2f, throttle=%.2f, brake=%.2f\n",
    steer_axis, throttle_axis, brake_axis);

    // TODO: CAREFUL, the following code is likely to crash the ESP32, was not tested
//   // Apply small deadzone for steering to avoid jitter around 0
//   if (fabs(steer_axis) < 0.05f) {  // deadzone threshold ~5%
//     steer_axis = 0.0f;
//   }

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
rclc_executor_t executor;

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
    // printf("rclc_publisher_init_default...\r\n");
    // ret =
    //     rclc_publisher_init_default(
    //         &publisher,
    //         &node,
    //         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    //         "/scan");
    // if (RCL_RET_OK != ret)
    // {
    //     printf("rclc_publisher_init_default error=%d\r\n", ret);
    //     return ret;
    // }

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

    rclc_executor_init(&executor, &support.context, 5, &allocator);

    // Joy subscription
    rclc_subscription_init_default(
        &joy_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "/joy");
    rclc_executor_add_subscription(&executor, &joy_sub, &joy_msg, &joyCallback, ON_NEW_DATA);

    // Accel subscription
    rclc_subscription_init_default(
        &accel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/accel");
    rclc_executor_add_subscription(&executor, &accel_sub, &accel_msg, &accelCallback, ON_NEW_DATA);

    // Decel subscription
    rclc_subscription_init_default(
        &decel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/decel");
    rclc_executor_add_subscription(&executor, &decel_sub, &decel_msg, &decelCallback, ON_NEW_DATA);

    // JoyX subscription
    rclc_subscription_init_default(
        &joyx_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/joyx");
    rclc_executor_add_subscription(&executor, &joyx_sub, &joyx_msg, &joyxCallback, ON_NEW_DATA);

    // JoyY subscription
    rclc_subscription_init_default(
        &joyy_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/joyy");
    rclc_executor_add_subscription(&executor, &joyy_sub, &joyy_msg, &joyyCallback, ON_NEW_DATA);

    // // 4) Servo & ESC setup… // TODO: CAREFUL, was not tested (chatgpt generated)
    // steeringServo.setPeriodHertz(50);
    // steeringServo.attach(SERVO_PWM_PIN, 1000, 2000);
    // escServo.setPeriodHertz(50);
    // escServo.attach(ESC_PWM_PIN, 1000, 2000);
    // escServo.writeMicroseconds(1500);
    // delay(100);

    // lidar.begin();
    // delay(1000);
    // lidar.resetLidar();
    // delay(800);
    // lidar.startLidar();
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

    // unsigned long uart_elapsed = millis();
    // int count = lidar.uartRx();
    // uart_elapsed = millis() - uart_elapsed;

    // unsigned long process_elapsed = millis();
    // lidar.processFrame(count);
    // process_elapsed = millis() - process_elapsed;

    // unsigned long publish_elapsed = millis();
    // rcl_ret_t ret_pub = rcl_publish(&publisher, &lidar.scan_msg, NULL);
    // publish_elapsed = millis() - publish_elapsed;

    // if (ret_pub != RCL_RET_OK)
    // {
    //     printf("rcl_publish returned %d\r\n", ret_pub);
    //     esp_restart();
    // }

    // // calculate loop period
    // total_loop_time = millis() - total_loop_time;
    // float total_loop_time_f = (float)total_loop_time;
    // loop_period = loop_period * 0.9 + total_loop_time_f * 0.1;

    // Serial.printf("got %d points in %lu ms. Frame Processing in %lu. Frame publishing in %lu. Total loop in %lu ms. Freq=%.1f Hz Serial2.available=%d\r\n",
    //               count, uart_elapsed, process_elapsed, publish_elapsed, total_loop_time, 1000.0 / loop_period, Serial2.available());
    // total_loop_time = millis();

    rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (rc != RCL_RET_OK) {
        printf("rclc_executor_spin_some error=%d\r\n", rc);
        // esp_restart();
    }
    // delay(30); // TODO: should we put delay ???
}