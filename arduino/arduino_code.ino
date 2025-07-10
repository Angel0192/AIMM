#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Motor 1
#define DIR1 4  // IN1
#define PWM1 5  // AN1 (PWM)

// Motor 2
#define DIR2 6  // IN2
#define PWM2 9  // AN2 (PWM)

// ROS-related variables
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

#define PWM_MAX 255

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;

  float linear = twist->linear.x;
  float angular = twist->angular.z;

  // Basic differential drive logic
  float left_speed = linear - angular;
  float right_speed = linear + angular;

  // Scale to PWM values
  int left_pwm = min(abs(left_speed * PWM_MAX), PWM_MAX);
  int right_pwm = min(abs(right_speed * PWM_MAX), PWM_MAX);

  // Set motor directions
  digitalWrite(DIR1, left_speed >= 0 ? HIGH : LOW);
  digitalWrite(DIR2, right_speed >= 0 ? HIGH : LOW);

  // Set motor speeds
  analogWrite(PWM1, left_pwm);
  analogWrite(PWM2, right_pwm);
}

void setup() {
  // Motor pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Micro-ROS setup
  set_microros_transports();  // You must define this for your board (Serial, WiFi, etc.)

  delay(2000);  // Let serial start

  allocator = rcl_get_default_allocator();

  // Init support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "motor_controller", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  // Init executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
