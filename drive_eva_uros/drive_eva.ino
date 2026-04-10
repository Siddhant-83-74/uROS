// #include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

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

int drive_swedish[4][2] = {
    {3, 2},   // FL: pwmL, pwmR
    {4, 5},   // FR: pwmL, pwmR
    {7, 6},   // BR: pwmL, pwmR
    {1, 0}    // BL: pwmL, pwmR
};
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist =
      (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_PIN, (twist->linear.x == 0) ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "teensy_drive"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  analogWriteResolution(14);
  analogWriteFrequency(1, 9000);
}

int x=0;
int y=0;
int w=0;
int max_pwm=6000;
void drive(int pwmL,int pwmR,int val)
{
  analogWrite(pwmL,val<0?abs(val):0);
  analogWrite(pwmR,val>0?abs(val):0);
}
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  Serial.print("x: "); Serial.print(msg.linear.x);
  Serial.print("  y: "); Serial.print(msg.linear.y);
  Serial.print("  w: "); Serial.println(msg.angular.z);
//------------IK
  x=int(msg.linear.x*128);
  y=int(msg.linear.y*128);
  w=int(msg.angular.z*128);


  drive(drive_swedish[0][0],drive_swedish[0][1],map((x+w/2),-128,128,-max_pwm,max_pwm));
  drive(drive_swedish[1][0],drive_swedish[1][1],map((-y+w/2),-128,128,-max_pwm,max_pwm));
  drive(drive_swedish[2][0],drive_swedish[2][1],map((x-w/2),-128,128,-max_pwm,max_pwm));
  drive(drive_swedish[3][0],drive_swedish[3][1],map((-y-w/2),-128,128,-max_pwm,max_pwm));
//-------------------------------
  delay(20);
}
