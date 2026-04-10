// #include <micro_ros_arduino.h>
#include "CubeMarsServo.h"
#include <Wire.h>
#include "RMCS-220X.h"
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
RMCS220X motor;

//----------------

//------Wrist Servo

int servoPin=8;
Servo wristServo;

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
CubeMarsMotor motors[] = {
  CubeMarsMotor(0x05, 6, 4000, 6000),//LeftArm
  CubeMarsMotor(0x06, 6, 4000, 6000),//LeftArmShoulder
  CubeMarsMotor(0x07, 6, 4000, 6000),//LeftArmBicep
  CubeMarsMotor(0x08, 6, 4000, 6000),//LeftArmElbow

};

const uint8_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// ---- Sweep configuration --------------------------------------------------

const float    SWEEP_POSITIONS[] = { 0.0f, 90.0f, 180.0f, -90.0f, 0.0f };
const uint8_t  NUM_POSITIONS     = sizeof(SWEEP_POSITIONS) / sizeof(SWEEP_POSITIONS[0]);
const uint32_t HOLD_MS           = 3000;  // Hold each waypoint for 3 seconds

// Per-motor state
uint8_t  sweepIndex[NUM_MOTORS]  = { 0, 1, 2 };  // staggered start positions
uint32_t lastChangeMs[NUM_MOTORS] = { 0, 0, 0 };

// ---------------------------------------------------------------------------

// CubeMarsCAN::attachMotors() expects a std::vector reference.
// We wrap our array into one here — this is the ONLY place vector is used.
std::vector<CubeMarsMotor> motorVec;

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

    CubeMarsCAN::begin(1000000);

  // Wrap plain array into vector so attachMotors() can dispatch CAN callbacks
  motorVec.reserve(NUM_MOTORS);
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motorVec.push_back(motors[i]);
  }
  CubeMarsCAN::attachMotors(motorVec);

  delay(200);

  // Set temporary origin on all motors
  // Serial.println("Setting temporary origins...");
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].setOrigin(0);  // 0=temp, 1=permanent, 2=restore
    delay(50);
  }

  delay(500);
  //---------CUBEMARS!!!!!!!!!

//------------Rhino

 motor.begin(ADDR);
 motor.calibrateEncoderPositionInDegrees(0);

 //------------

 //---------Servo

wristServo.attach(servoPin);

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
LeftArmBicep=(msg.data.data[1]);
LeftArmElbow=msg.data.data[2];
RightArm=msg.data.data[3];
LeftArmForearm=msg.data.data[4];
RightArmBicep=-1*(msg.data.data[5]);
LeftArmwrist=msg.data.data[6];
RightArmElbow=-1*(msg.data.data[7]);
RightArmShoulder=msg.data.data[8];
RightArmForearm=msg.data.data[9]; 
LeftArm=msg.data.data[10];
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
  motors[0].setTargetPositionDeg(RightArm);
  motors[1].setTargetPositionDeg(RightArmShoulder);
  motors[2].setTargetPositionDeg(RightArmBicep);
  motors[3].setTargetPositionDeg(RightArmElbow);
  // }
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
  motors[i].update();
  }
  //-----------------


  motor.goToPositionInDegrees(RightArmForearm);

  wristServo.write(RightArmwrist);

  // CubeMarsCAN::update();
  delay(20);

}
