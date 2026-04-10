// #include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>  // CHANGED

// CHANGED: MultiArray message + static buffer
#define JOINT_ARRAY_SIZE 12
float joint_data[JOINT_ARRAY_SIZE];            // Static buffer (from reference)

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;          // CHANGED
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#ifdef LED_BUILTIN
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // Same LED logic — lights up if first joint is non-zero
  digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  // CHANGED
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  // ADDED: Point msg to static buffer (reference pattern)
  msg.data.data     = joint_data;
  msg.data.capacity = JOINT_ARRAY_SIZE;
  msg.data.size     = 0;

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),  // CHANGED
    "micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // CHANGED: RCSOFTCHECK
Serial.print("Joints: ");
  for (int i = 0; i < msg.data.size; i++) {
    Serial.print(msg.data.data[i]);
    Serial.print("  ");
  }
  Serial.println();
}
