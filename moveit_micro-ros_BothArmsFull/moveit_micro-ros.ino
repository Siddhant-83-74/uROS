// #include <micro_ros_arduino.h>
#include "CubeMarsServoDual.h"
#include <Wire.h>
#include "RMCS-220X_Right.h"
#include "RMCS-220X_Left.h"
#include <Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/byte_multi_array.h>   // ← ADDED

// ── Joint array ──────────────────────────────────────────────
#define JOINT_ARRAY_SIZE 12
float joint_data[JOINT_ARRAY_SIZE];

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;

// ── Finger subscriber ────────────────────────────────────────
rcl_subscription_t finger_subscriber;          // ← ADDED
std_msgs__msg__ByteMultiArray finger_msg;       // ← ADDED
uint8_t finger_data_buf[2];                     // ← ADDED
bool finger1 = false;                           // ← ADDED
bool finger2 = false;                           // ← ADDED

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

//-----Fingers
const int NUM_SERVOS = 10;
const int SERVO_PINS[NUM_SERVOS] = {0,2,3,4,5,6,7,8,9,12};
Servo fingers[NUM_SERVOS];

//-----Rhino
#define ADDR 0x08
RMCS220X_Left motor_left;
RMCS220X_Right motor_right;

//------Wrist Servo
int servoPinLeft=10;
Servo wristLeftServo;
int servoPinRight=1;
Servo wristRightServo;

float LeftArmShoulder=0.0;
float LeftArmBicep=0.0;
float LeftArmElbow=0.0;
float RightArm=0.0;
float LeftArmForearm=0.0;
float RightArmBicep=0.0;
float LeftArmwrist=0.0;
float RightArmElbow=0.0;
float RightArmShoulder=0.0;
float RightArmForearm=0.0;
float LeftArm=0.0;
float RightArmwrist=0.0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ── Joint callback ───────────────────────────────────────────
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = 
    (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);
}

// ── Finger callback ──────────────────────────────────────────  ← ADDED
void finger_callback(const void * msgin) {
  const std_msgs__msg__ByteMultiArray * fmsg = 
    (const std_msgs__msg__ByteMultiArray *)msgin;

  if (fmsg->data.size >= 2) {
    finger1 = (bool)fmsg->data.data[0];
    finger2 = (bool)fmsg->data.data[1];
    // Serial.print("Finger1: "); Serial.print(finger1);
    // Serial.print("  Finger2: "); Serial.println(finger2);
  }
}

//---------------CUBEMARS_SETUP---------------
CubeMarsCAN<CAN1> leftBus;
CubeMarsCAN<CAN3> rightBus;

std::vector<CubeMarsMotor> leftArm = {
  CubeMarsMotor(0x01, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x02, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x03, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x04, leftBus, 6, 4000, 6000),
};

std::vector<CubeMarsMotor> rightArm = {
  CubeMarsMotor(0x05, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x06, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x07, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x08, rightBus, 6, 4000, 6000),
};

static constexpr uint32_t CONTROL_LOOP_MS = 10;
static uint32_t lastLoopMs = 0;

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  // ── Joint msg buffer ─────────────────────────────────────
  msg.data.data     = joint_data;
  msg.data.capacity = JOINT_ARRAY_SIZE;
  msg.data.size     = 0;

  // ── Finger msg buffer ────────────────────────────────────  ← ADDED
  finger_msg.data.data     = finger_data_buf;
  finger_msg.data.capacity = 2;
  finger_msg.data.size     = 0;

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // ── Joint subscriber ─────────────────────────────────────
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_subscriber"));

  // ── Finger subscriber ────────────────────────────────────  ← ADDED
  RCCHECK(rclc_subscription_init_default(
    &finger_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ByteMultiArray),
    "micro_ros_finger_status"));

  // ── Executor — count = 2 now ─────────────────────────────  ← CHANGED 1→2
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber,
    &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &finger_subscriber,
    &finger_msg, &finger_callback, ON_NEW_DATA));  // ← ADDED

  //---------CUBEMARS
  leftBus.begin(1000000);
  leftBus.attachMotors(leftArm);
  Serial.println("[CAN1] Left  arm ready — 4 motors");

  rightBus.begin(1000000);
  rightBus.attachMotors(rightArm);
  Serial.println("[CAN2] Right arm ready — 4 motors");

  for (auto &m : leftArm)  { m.setOrigin(0); delay(5); }
  for (auto &m : rightArm) { m.setOrigin(0); delay(5); }
  for (auto &m : leftArm)  m.update();
  for (auto &m : rightArm) m.update();
  delay(500);
  Serial.println("Origin Set");

  //------------Rhino
  motor_left.begin(ADDR);
  motor_left.calibrateEncoderPositionInDegrees(0);
  motor_right.begin(ADDR);
  motor_right.calibrateEncoderPositionInDegrees(0);

  //---------Servo
  wristLeftServo.attach(servoPinLeft);
  wristRightServo.attach(servoPinRight);

  //-----fingers
  for (int i = 0; i < NUM_SERVOS; i++) {
    fingers[i].attach(SERVO_PINS[i]);
  }
}
bool isLeftClose=false;
bool isRightClose=false;
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Serial.print("Joints: ");
  // for (int i = 0; i < msg.data.size; i++) {
  //   Serial.print(msg.data.data[i]);
  //   Serial.print("  ");
  // }
  // Serial.println();

  LeftArmShoulder = msg.data.data[0];
  LeftArmBicep    = -1*(msg.data.data[1]);
  LeftArmElbow    = -1*msg.data.data[2];
  RightArm        = msg.data.data[3];
  LeftArmForearm  = msg.data.data[4];
  RightArmBicep   = -1*(msg.data.data[5]);
  LeftArmwrist    = msg.data.data[6];
  RightArmElbow   = -1*(msg.data.data[7]);
  RightArmShoulder= -1*msg.data.data[8];
  RightArmForearm = msg.data.data[9];
  LeftArm         = -1*msg.data.data[10];
  RightArmwrist   = msg.data.data[11];

  Serial.print("LShoulder: "); Serial.print(LeftArmShoulder, 2);  Serial.print("° ");
  Serial.print("LBicep: ");    Serial.print(LeftArmBicep, 2);     Serial.print("° ");
  Serial.print("LElbow: ");    Serial.print(LeftArmElbow, 2);     Serial.print("° ");
  Serial.print("RArm: ");      Serial.print(RightArm, 2);         Serial.print("° ");
  Serial.print("LForearm: ");  Serial.print(LeftArmForearm, 2);   Serial.print("° ");
  Serial.print("RBicep: ");    Serial.print(RightArmBicep, 2);    Serial.print("° ");
  Serial.print("LWrist: ");    Serial.print(LeftArmwrist, 2);     Serial.print("° ");
  Serial.print("RElbow: ");    Serial.print(RightArmElbow, 2);    Serial.print("° ");
  Serial.print("RShoulder: "); Serial.print(RightArmShoulder, 2); Serial.print("° ");
  Serial.print("RForearm: ");  Serial.print(RightArmForearm, 2);  Serial.print("° ");
  Serial.print("LArm: ");      Serial.print(LeftArm, 2);          Serial.print("° ");
  Serial.print("RWrist: ");    Serial.print(RightArmwrist, 2);
  Serial.println("°");

// // ── Print Finger Status ──────────────────────────────────────
// Serial.print("Finger1: "); Serial.print(finger1 ? "CLOSED" : "OPEN");
// Serial.print("  Finger2: "); Serial.println(finger2 ? "CLOSED" : "OPEN");

  //------CUBEMARS
  rightArm[0].setTargetPositionDeg(RightArm);
  rightArm[1].setTargetPositionDeg(RightArmShoulder);
  rightArm[2].setTargetPositionDeg(RightArmBicep);
  rightArm[3].setTargetPositionDeg(RightArmElbow);

  leftArm[0].setTargetPositionDeg(LeftArm);
  leftArm[1].setTargetPositionDeg(LeftArmShoulder);
  leftArm[2].setTargetPositionDeg(LeftArmBicep);
  leftArm[3].setTargetPositionDeg(LeftArmElbow);

  for(int i = 0; i < 4; i++) {
    rightArm[i].update();
    leftArm[i].update();
  }

  motor_right.goToPositionInDegrees(RightArmForearm);
  motor_left.goToPositionInDegrees(LeftArmForearm);

 RightArmwrist=RightArmwrist<-20?-20:RightArmwrist;
 RightArmwrist=RightArmwrist>20?20:RightArmwrist;
 LeftArmwrist=LeftArmwrist<-20?-20:LeftArmwrist;
 LeftArmwrist=LeftArmwrist>20?20:LeftArmwrist;
 RightArmwrist=RightArmwrist+90;
 LeftArmwrist=LeftArmwrist+90;

 Serial.println(RightArmwrist);
  Serial.println(LeftArmwrist);

  // wristRightServo.write(RightArmwrist );
  // wristLeftServo.write(LeftArmwrist );

  // ── Finger servos ────────────────────────────────────────  ← ADDED
  // for (int i = 0; i < 6; i++) {
  //   fingers[i].write(finger1 ? 90 : 0);   // Left hand (servos 0-5)
  // }
  // for (int i = 6; i < 12; i++) {
  //   fingers[i].write(finger2 ? 90 : 0);   // Right hand (servos 6-11)
  // }
//   if(isLeftClose && !finger2)//open
// {
//   fingers[0].write(0);
//   fingers[2].write(180);
//   fingers[5].write(180);
//   fingers[3].write(0);
//   fingers[4].write(0);
//   isLeftClose=!isLeftClose;

//   Serial.println("Left is Opened");

// }
// if(!isLeftClose && finger2)//close
// {
//   fingers[0].write(180);
//   fingers[2].write(0);
//   fingers[5].write(0);
//   fingers[3].write(180);
//   fingers[4].write(180);
//   isLeftClose=!isLeftClose;

//   Serial.println("Left is Closed");
// }
// if(isRightClose && !finger1)//op3n
// {
//   fingers[6].write(180);
//   fingers[7].write(0);
//   fingers[8].write(180);
//   fingers[9].write(0);
//   fingers[11].write(180);
//   isRightClose=!isRightClose;

//   Serial.println("Right is Opened");
// }

// if(!isRightClose && finger1)//close
// {
//   fingers[6].write(0);
//   fingers[7].write(180);
//   fingers[8].write(0);
//   fingers[9].write(180);
//   fingers[11].write(0);
//   isRightClose=!isRightClose;

//   Serial.println("Right is Closed");
// }
  delay(20);
}
