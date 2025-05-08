#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>

const int motorPin1 = 5; // motor direction
rcl_subscription_t subscriber;
std_msgs__msg__Int8 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t subscriber;
std_msgs__msg__Int8 msg;

void motor_callback(const void *msgin) {
  const std_msgs__msg__Int8 *cmd = (const std_msgs__msg__Int8 *)msgin;

  if (cmd->data == 1) {
    // forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 200);
  } else if (cmd->data == -1) {
    // backward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, 200);
  } else {
    // stop
    analogWrite(enablePin, 0);
  }
}

void setup() {
  set_microros_transports();
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "motor_cmd");

  rclc_executor_add_subscription(&executor, &subscriber, &msg, &motor_callback, ON_NEW_DATA);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
