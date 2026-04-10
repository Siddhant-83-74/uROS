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

//-----Rhino

#define ADDR 0x08
RMCS220X_Left motor_left;
RMCS220X_Right motor_right;

//----------------

//------Wrist Servo

int servoPinLeft=10;
Servo wristLeftServo;

int servoPinRight=1;
Servo wristRightServo;

//---------

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

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // Same LED logic — lights up if first joint is non-zero
  digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  // CHANGED


}

//---------------CUBEMARS_SETUP---------------

// ── CAN bus instances ────────────────────────────────────────
CubeMarsCAN<CAN1> leftBus;
CubeMarsCAN<CAN3> rightBus;

// ── Motor lists ──────────────────────────────────────────────
//   CubeMarsMotor(motorId, busRef, controlMode, speedErpm, accelErpm2)
std::vector<CubeMarsMotor> leftArm = {
  CubeMarsMotor(0x01, leftBus, 6, 4000, 6000),   // shoulder pitch
  CubeMarsMotor(0x02, leftBus, 6, 4000, 6000),   // shoulder roll
  CubeMarsMotor(0x03, leftBus, 6, 4000, 6000),
  CubeMarsMotor(0x04, leftBus, 6, 4000, 6000),   // elbow
};

std::vector<CubeMarsMotor> rightArm = {
  CubeMarsMotor(0x05, rightBus, 6, 4000, 6000),  // shoulder pitch
  CubeMarsMotor(0x06, rightBus, 6, 4000, 6000),  // shoulder roll
  CubeMarsMotor(0x07, rightBus, 6, 4000, 6000),
  CubeMarsMotor(0x08, rightBus, 6, 4000, 6000),  // elbow
};

// ── Control-loop timing ──────────────────────────────────────
static constexpr uint32_t CONTROL_LOOP_MS = 10;   // 100 Hz
static uint32_t lastLoopMs = 0;

//-----------------------CUUBEMARS_SETUP----!!!!!!!!!!!


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

  //---------CUBEMARS

leftBus.begin(1000000);
  leftBus.attachMotors(leftArm);
  Serial.println("[CAN1] Left  arm ready — 4 motors");

  rightBus.begin(1000000);
  rightBus.attachMotors(rightArm);
  Serial.println("[CAN2] Right arm ready — 4 motors");

  // ── Optional: zero both arms at startup ──
  // Zero both arms
  for (auto &m : leftArm)  { m.setOrigin(0); delay(5); }
  for (auto &m : rightArm) { m.setOrigin(0); delay(5); }


    for (auto &m : leftArm)  m.update();
    for (auto &m : rightArm) m.update();
  delay(500);  // let origin commands settle

    
  // delay(500);
Serial.println("Origin Set");
  //---------CUBEMARS!!!!!!!!!

//------------Rhino

 motor_left.begin(ADDR);
 motor_left.calibrateEncoderPositionInDegrees(0);

  motor_right.begin(ADDR);
 motor_right.calibrateEncoderPositionInDegrees(0);
 //------------

 //---------Servo

wristLeftServo.attach(servoPinLeft);
wristRightServo.attach(servoPinRight);




}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // CHANGED: RCSOFTCHECK
  Serial.print("Joints: ");
  for (int i = 0; i < msg.data.size; i++) {
    Serial.print(msg.data.data[i]);
    Serial.print("  ");


  }
    Serial.println();

LeftArmShoulder=msg.data.data[0];
LeftArmBicep=-1*(msg.data.data[1]);
LeftArmElbow=-1*msg.data.data[2];
RightArm=msg.data.data[3];
LeftArmForearm=msg.data.data[4];
RightArmBicep=-1*(msg.data.data[5]);
LeftArmwrist=msg.data.data[6];
RightArmElbow=-1*(msg.data.data[7]);
RightArmShoulder=-1*msg.data.data[8];
RightArmForearm=msg.data.data[9]; 
LeftArm=-1*msg.data.data[10];
RightArmwrist=msg.data.data[11];






// Print YOUR final variables (not array indices)
Serial.print("LShoulder: "); Serial.print(LeftArmShoulder, 2); Serial.print("° ");
Serial.print("LBicep: ");     Serial.print(LeftArmBicep, 2);    Serial.print("° ");
Serial.print("LElbow: ");     Serial.print(LeftArmElbow, 2);    Serial.print("° ");
Serial.print("RArm: ");       Serial.print(RightArm, 2);        Serial.print("° ");
Serial.print("LForearm: ");   Serial.print(LeftArmForearm, 2);  Serial.print("° ");
Serial.print("RBicep: ");     Serial.print(RightArmBicep, 2);   Serial.print("° ");
Serial.print("LWrist: ");     Serial.print(LeftArmwrist, 2);    Serial.print("° ");
Serial.print("RElbow: ");     Serial.print(RightArmElbow, 2);   Serial.print("° ");
Serial.print("RShoulder: ");  Serial.print(RightArmShoulder, 2);Serial.print("° ");
Serial.print("RForearm: ");   Serial.print(RightArmForearm, 2);  Serial.print("° ");
Serial.print("LArm: ");       Serial.print(LeftArm, 2);         Serial.print("° ");
Serial.print("RWrist: ");     Serial.print(RightArmwrist, 2);
Serial.println("°");



  // for (uint8_t i = 0; i < NUM_MOTORS; i++) {

  //------CUBEMARS
  rightArm[0].setTargetPositionDeg(RightArm);
  rightArm[1].setTargetPositionDeg(RightArmShoulder);
  rightArm[2].setTargetPositionDeg(RightArmBicep);
  rightArm[3].setTargetPositionDeg(RightArmElbow);

  leftArm[0].setTargetPositionDeg(LeftArm);
  leftArm[1].setTargetPositionDeg(LeftArmShoulder);
  leftArm[2].setTargetPositionDeg(LeftArmBicep);
  leftArm[3].setTargetPositionDeg(LeftArmElbow);


  // }
for(int i=0;i<4;i++)
{
  rightArm[i].update();
  leftArm[i].update();
}
  //-----------------


  motor_right.goToPositionInDegrees(RightArmForearm);
  motor_left.goToPositionInDegrees(LeftArmForearm);

  wristRightServo.write(RightArmwrist+90);
  wristLeftServo.write(LeftArmwrist+90);

  // CubeMarsCAN::update();
  delay(20);

}
