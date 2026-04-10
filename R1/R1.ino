#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

//#include <Wire.h>
//#include <Adafruit_BNO055.h>
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#include <Encoder.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <USBHost_t36.h>

// ========= servo limits ========
int gripper_min = 50;
int gripper_max = 150;
int center_min = 40;
int center_max = 150;

// ARM
Encoder armEnc(33, 32); 

int armRPWM = 15;
int armLPWM = 14;

float armKP = 17.0;
float armKI = 0.0;
float armKD = 0.0;

float armErr = 0, armPrevErr = 0, armInteg = 0;
float armPID = 0;

long armSP_ticks = 0;
float ticks_per_deg = 17500.0 / 360.0;

//arm limits
float ARM_MAX_DEG = 0.0;
float ARM_MIN_DEG = -65.0;

long ARM_MAX_TICKS = ARM_MAX_DEG * ticks_per_deg;   // 0
long ARM_MIN_TICKS = ARM_MIN_DEG * ticks_per_deg;   // negative

// SERVOS
Servo centerServo, gripperServo, shaft, servo1, grip;
int centerServoPin = 0;
int gripperServoPin = 0;
int shaftServoPin = 29;
int gripPin = 9;
int servo1pin = 8;

int centerPos = 90;
int gripperPos = 90;

// CONVEYOR

int convRPWM = 22;
int convLPWM = 23;
int conveyorSpeed = 9000;

//Pnematic
int solenoid_gripper = 31;
bool gripper_state = false; //Closed
bool lastButtonGripper = false;



int climber_solenoid = 32;
bool climber_state = false; //Closed
bool servo_state = false;
bool lastButtonExtension = false;


//#define RADIUS 0.063      //in m  
//float ROBOT_RADIUS[3] = {0.28, 0.3, 0.3};

#define RADIUS 6.3                // in cm
float ROBOT_RADIUS[3] = {28, 30, 30};

int pwmL_pin[3] = {0, 4, 12};
int pwmR_pin[3] = {1, 5, 13};
int max_rpm = 200;

Encoder m[3] = { Encoder(20, 21), Encoder(27, 26), Encoder(40, 41) };

// === MICRO-ROS VARIABLES ===
rcl_subscription_t tuning_subscriber;
rcl_publisher_t state_publisher;

std_msgs__msg__Float32MultiArray tuning_msg;  // From Python node - [ps4 + accl + pid constants]
std_msgs__msg__Float32MultiArray state_msg;   // To Python node - [actual and desired velocity]

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Message buffers
#define TUNING_ARRAY_SIZE 32    // [ps4_lx, ps4_ly, ps4_rx, ps4_ry, phone_ax, phone_ay, gyro_z, Kp_vx, Ki_vx, Kd_vx, Kp_vy, Ki_vy, Kd_vy, Kp_z, Ki_z, Kd_z, btn]  
#define STATE_ARRAY_SIZE 4     // [vx, vy, pid_vx, pid_vy] 

float tuning_data[TUNING_ARRAY_SIZE];
float state_data[STATE_ARRAY_SIZE];

// === CONTROL VARIABLES ===
float ps4_lx = 0, ps4_ly = 0, ps4_rx = 0, ps4_ry = 0;
float phone_ax = 0, phone_ay = 0, gyro_z = 0;
//bool ps4_btn[12] = {0};
bool ps4_btn[16] = {0};
bool last_ps4_btn[16] = {0};
bool ps4_btn_pulse[16] = {0};


const char* ps4_btn_name[16] = {
  "X", "O", "SQUARE", "TRIANGLE",
  "L1", "R1", "L2", "R2",
  "SHARE", "OPTIONS", "PS", "L3",
  "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT"
};


// === JOYSTICK RATE LIMITING ===

float previous_ps4_lx = 0.0f;
float previous_ps4_ly = 0.0f;
float previous_ps4_rx = 0.0f;

float raw_ps4_lx = 0.0f;
float raw_ps4_ly = 0.0f;
float raw_ps4_rx = 0.0f;

  
// VELOCITY PID 
float Kp_vx = 0.0, Ki_vx = 0.0, Kd_vx = 0.0;
float Kp_vy = 0.0, Ki_vy = 0.0, Kd_vy = 0.0;
float Kp_z = 0.0, Ki_z = 0.0, Kd_z = 0.0;

float dt = 0.001;
float error_vx, eDer_vx, eInt_vx, error_vy, eDer_vy, eInt_vy; 
float error_z, eDer_z, eInt_z; 
float pid_vx, pid_vy, pid_z;
float lastError_vx, lastError_vy, lastError_z;
float ax = 0, ay = 0, z = 0;
float vx = 0, vy = 0;
float target_vx, target_vy, target_w;

// RPM PID 
volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };

volatile float kp[] = { 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };

volatile long oldPosition[3] = { 0, 0, 0 };
volatile long count[3] = { 0, 0, 0 };
volatile long newPosition[3] = { 0, 0, 0 };

volatile float rpm_rt[3] = { 0, 0, 0 };
//float cpr[] = {1300.0, 700.0, 700.0};
float cpr[] = {3550.0, 3550.0, 3550.0};

// Publishing timing
unsigned long last_publish_time = 0;
const unsigned long PUBLISH_INTERVAL = 50; // ms (20Hz to match Python)

// === ETHERNET CONFIGURATION ===
#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
}

// === MESSAGE INITIALIZATION ===
void setup_messages() {
  // Initialize tuning message (from Jetson)
  tuning_msg.data.data = tuning_data;
  tuning_msg.data.capacity = TUNING_ARRAY_SIZE;
  tuning_msg.data.size = 0;  // Will be set when message received
  
  // Initialize state message (to Jetson) 
  state_msg.data.data = state_data;
  state_msg.data.capacity = STATE_ARRAY_SIZE;
  state_msg.data.size = STATE_ARRAY_SIZE;
}

// === TUNING SUBSCRIPTION CALLBACK ===
void tuning_callback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* tuning_cmd =
      (const std_msgs__msg__Float32MultiArray*)msgin;

  if (tuning_cmd->data.size != 32) return;   // SAFETY CHECK

  raw_ps4_lx = tuning_cmd->data.data[0] * 375;
  raw_ps4_ly = tuning_cmd->data.data[1] * 375;
  raw_ps4_rx = tuning_cmd->data.data[2] * 5;
  ps4_ry     = tuning_cmd->data.data[3];

  phone_ax = tuning_cmd->data.data[4];
  phone_ay = tuning_cmd->data.data[5];
  gyro_z   = tuning_cmd->data.data[6];


  Kp_vx = tuning_cmd->data.data[7];
  Ki_vx = tuning_cmd->data.data[8];
  Kd_vx = tuning_cmd->data.data[9];
  Kp_vy = tuning_cmd->data.data[10];
  Ki_vy = tuning_cmd->data.data[11];
  Kd_vy = tuning_cmd->data.data[12];
  Kp_z  = tuning_cmd->data.data[13];
  Ki_z  = tuning_cmd->data.data[14];
  Kd_z  = tuning_cmd->data.data[15];


  for (int i = 0; i < 16; i++) {
    bool current = (bool)tuning_cmd->data.data[16 + i];
  
    ps4_btn_pulse[i] = current && !last_ps4_btn[i];
 
    ps4_btn[i] = current;
  

//    if (ps4_btn_pulse[i]) {
//      Serial.print("BUTTON PULSE -> index: ");
//      Serial.print(i);
//      Serial.print(" name: ");
//      Serial.println(ps4_btn_name[i]);
//    }
  
    last_ps4_btn[i] = current;
  }
  unsigned long last_debug_time = 0;


  unsigned long now = millis();
  if (now - last_debug_time > 200) {  // print at 5 Hz
//    Serial.print("[AXIS] ");
//    Serial.print("LX: "); Serial.print(raw_ps4_lx);
//    Serial.print(" LY: "); Serial.print(raw_ps4_ly);
// 
//    Serial.println();
    last_debug_time = now;
  }


}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void inverseKinematics(float vy, float vx, float omega, float* rpms) {
  float w1 = (-sin(0)*vx + cos(0)*vy + ROBOT_RADIUS[0]*omega)/RADIUS;
  float w2 = (-sin(2*PI/3)*vx + cos(2*PI/3)*vy + ROBOT_RADIUS[1]*omega)/RADIUS;
  float w3 = (-sin(4*PI/3)*vx + cos(4*PI/3)*vy + ROBOT_RADIUS[2]*omega)/RADIUS;

  // rad/s --> RPM
  rpms[0] = w1*50.0/(2*PI);
  rpms[1] = w2*60.0/(2*PI);
  rpms[2] = w3*60.0/(2*PI);

  for(int i = 0; i<3; i++){
    rpms[i] = constrain(rpms[i], -max_rpm, max_rpm);
  }

//  rpms[0] = map(x + 0.3*w, -175, 175, max_rpm, -max_rpm);
//  rpms[1] = map(-0.5 * x - 0.852 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);
//  rpms[2] = map(-0.5 * x + 0.860 * y + 0.3*w, -175, 175, max_rpm, -max_rpm);
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin){
  analogWrite(pwmRPin, (pwm_val <= 0 ? -1*pwm_val : 0));
  analogWrite(pwmLPin, (pwm_val >= 0 ? pwm_val : 0));
}

IntervalTimer motorTimer;
IntervalTimer velTimer;
IntervalTimer armTimer;

float rpm_cmd[3];

const float MAX_DELTA_V_PER_MS = 2.0f;  // Maximum velocity change per millisecond

//float raw_ps4_lx = 0, raw_ps4_ly = 0, raw_ps4_rx = 0;

float limited_ps4_lx = 0, limited_ps4_ly = 0, limited_ps4_rx = 0;

float limit_joystick_rate_simple(float target_val, float current_val) {
    float delta = target_val - current_val;
    float max_delta = MAX_DELTA_V_PER_MS;  // Since we call this every 1ms
    
    if (abs(delta) > max_delta) {
        delta = (delta > 0) ? max_delta : -max_delta;
    }
    
    return current_val + delta;
}

void vel_update(){
//  ps4_lx = limit_joystick_rate_simple(ps4_lx, previous_ps4_lx);
//  ps4_ly = limit_joystick_rate_simple(ps4_ly, previous_ps4_ly);
//  ps4_rx = limit_joystick_rate_simple(ps4_rx, previous_ps4_rx);
//  
//  previous_ps4_lx = ps4_lx;
//  previous_ps4_ly = ps4_ly;
//  previous_ps4_rx = ps4_rx;

  limited_ps4_lx = limit_joystick_rate_simple(raw_ps4_lx, limited_ps4_lx);
  limited_ps4_ly = limit_joystick_rate_simple(raw_ps4_ly, limited_ps4_ly);
  limited_ps4_rx = limit_joystick_rate_simple(raw_ps4_rx, limited_ps4_rx);
  
  // Use the RATE-LIMITED values for control
  target_vx = limited_ps4_lx;        
  target_vy = limited_ps4_ly;        
  target_w = limited_ps4_rx;
  
  // Debug output to see both raw and limited values
//  Serial.printf("Raw: (%.1f, %.1f) Limited: (%.1f, %.1f)", 
//                raw_ps4_lx, raw_ps4_ly, limited_ps4_lx, limited_ps4_ly);
//  Serial.println();

//  Serial.printf("Rate Limited - LX:%.1f, LY:%.1f, RX:%.1f", limited_ps4_lx, limited_ps4_ly, limited_ps4_rx);
//  Serial.println();

  
    ax = phone_ax;
    ay = phone_ay;
    z = gyro_z;
  
    vx = vx + ax * dt*100;    // actual vel in x in cm/s
    vy = vy + ay * dt*100;    // actual vel in y
  
    vx =vx* 0.95;
    vy =vy* 0.95;

    // Serial.printf("vx:%0.2f   vy:%0.2f   ",vx ,vy);
  
//    target_vx = ps4_lx;        
//    target_vy = ps4_ly;        
//    target_w = ps4_rx;  
  
    // VELOCITY PID
    error_vx = target_vx - vx;
    eDer_vx = (error_vx - lastError_vx)/dt;
    eInt_vx = eInt_vx + error_vx * dt;
    pid_vx = Kp_vx * error_vx + Ki_vx * eInt_vx + Kd_vx * eDer_vx;
    lastError_vx = error_vx;
  
    error_vy = target_vy - vy;
    eDer_vy = (error_vy - lastError_vy)/dt;
    eInt_vy = eInt_vy + error_vy * dt;
    pid_vy = Kp_vy * error_vy + Ki_vy * eInt_vy + Kd_vy * eDer_vy;
    lastError_vy = error_vy;

    error_z = target_w - z;
    eDer_z = (error_z - lastError_z)/dt;
    eInt_z = eInt_z + error_z * dt;
    pid_z = Kp_z * error_z + Ki_z * eInt_z + Kd_z * eDer_z;
    lastError_z = error_z;

    
  
//    float rpm_cmd[3];
//    inverseKinematics(pid_vx, pid_vy, pid_z, rpm_cmd); 
  
//    Serial.printf("setpoint_vx:%.2f, solenoid_grippersetpoint_vy:%.2f, setpoint_z:%.2f, actual_vx:%.2f, actual_vy:%.2f, actual_z:%.2f", 
//                    target_vx, target_vy, target_w, vx, vy, z);
//    Serial.println();
  
}

void motor_update(){

    long pos = armEnc.read();
//  Serial.print("Arm SP: ");
//  Serial.print(armSP_ticks);
//  Serial.print(" | Ticks:    ");
//  Serial.println(pos);

  armErr = armSP_ticks - pos;
  armInteg = armInteg + (armErr * 0.075);
  float der1 = (armErr - armPrevErr) / 0.075;

  armPID = (armKP * armErr) + (armKI * armInteg) + (armKD * der1);
  armPID = constrain(armPID, -3000, 3000);
  armPrevErr = armErr;

  runMotor(armPID, armLPWM, armRPWM);
  
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
//     Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//     Serial.println();
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }

//  
//
//  float rpm_cmd[3];
  inverseKinematics(pid_vx, pid_vy, pid_z, rpm_cmd); 
//
//  Serial.printf("setpoint_vx:%.2f, setpoint_vy:%.2f, actual_vx:%.2f, actual_vy:%.2f", 
//                  target_vx, target_vy, vx, vy);
//  Serial.println();

//   float rpm_cmd[3];
//   inverseKinematics(target_vx, target_vy, target_w, rpm_cmd);

  // RPM PID
  for (int i = 0; i < 3; i++) {
    error[i] = rpm_cmd[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    pwm_pid[i] = constrain(pwm_pid[i], -15000, 15000);
    
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

    lastError[i] = error[i];
//     Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
//     Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_cmd[i]);
  }

  // Update state data for publishing (4 elements as expected by Python)
  state_data[0] = vx;        // Actual velocity X
  state_data[1] = vy;        // Actual velocity Y  
  state_data[2] = pid_vx;    // PID output for VX
  state_data[3] = pid_vy;    // PID output for VY
}


void armPIDLoop() {

  long pos = armEnc.read();
//  Serial.print("Arm SP: ");
//  Serial.print(armSP_ticks);
//  Serial.print(" | Ticks:    ");
//  Serial.println(pos);

  armErr = armSP_ticks - pos;
  armInteg = armInteg + (armErr * 0.075);
  float der1 = (armErr - armPrevErr) / 0.075;

  armPID = (armKP * armErr) + (armKI * armInteg) + (armKD * der1);
  armPID = constrain(armPID, -3000, 3000);
  armPrevErr = armErr;

  runMotor(armPID, armLPWM, armRPWM);
}



// === SETUP ===
void setup(){
//  pinMode(13,OUTPUT); digitalWrite(13,HIGH);
  Serial.begin(115200); delay(2000);
  analogWriteResolution(14);

  // Ethernet setup
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 100);

  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);
//  while (!Serial) delay(10);

//  Serial.println("Teensy Tuning Control Node Starting...");

  /* Initialize the sensor */
  // Commented out for now as per your code
  // if (!bno.begin()) {
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1);
  // }
  // delay(1000);

  // Micro-ROS initialization
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // Setup messages
  setup_messages();

  // Create subscriber for tuning commands from Python
  RCCHECK(rclc_subscription_init_default(
    &tuning_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "tuning_commands"));  // Match Python publisher topic

  // Create publisher for state (to Python)
  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "teensy_state"));  // Match Python subscriber topic

  // Executor setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &tuning_subscriber, &tuning_msg, &tuning_callback, ON_NEW_DATA));

  pinMode(armRPWM, OUTPUT);
  pinMode(armLPWM, OUTPUT);

  pinMode(convRPWM, OUTPUT);
  pinMode(convLPWM, OUTPUT);

  centerServo.attach(centerServoPin);
  gripperServo.attach(gripperServoPin);
  shaft.attach(shaftServoPin);
  grip.attach(gripPin);
  servo1.attach(servo1pin);

  centerServo.write(90);
  gripperServo.write(90);
  shaft.write(0);
  grip.write(60);
  servo1.write(0);
  //Pnematics
  pinMode(solenoid_gripper, OUTPUT);
  pinMode(climber_solenoid, OUTPUT);
  digitalWrite(climber_solenoid, LOW);
  digitalWrite(solenoid_gripper, LOW);



  velTimer.priority(0);
  motorTimer.priority(1);  
//  armTimer.priority(0);

  velTimer.begin(vel_update, 1000); // every 1ms
  motorTimer.begin(motor_update, 75*1000); // every 75ms

//  armTimer.begin(armPIDLoop, 10000);
  

 
  
  
//  Serial.println("Teensy Tuning Node Ready!");
//  Serial.println("Listening: tuning_commands (12 floats)");
//  Serial.println("Publishing: teensy_state (4 floats)");
//  Serial.println("Control Sources: PS4 sticks + Phone accelerometer");
//  Serial.println("PID Parameters: Received via ROS from Python");
}

//int currTIme = 0;
//int lastTime = 0;

static int debug_counter = 0;

enum PS4Button {
  BTN_X = 0,
  BTN_O,
  BTN_SQUARE,
  BTN_TRIANGLE,
  BTN_L1,
  BTN_R1,
  BTN_L2,
  BTN_R2,
  BTN_SHARE,
  BTN_OPTIONS,
  BTN_PS,
  BTN_L3,

  BTN_DPAD_UP,
  BTN_DPAD_DOWN,
  BTN_DPAD_LEFT,
  BTN_DPAD_RIGHT
};


int currentAngle = 0;
int currentAngle2 = 60;

void handleMechanisms() {

  /* ================= ARM PRESETS ================= */

  // UP
  if (ps4_btn[BTN_TRIANGLE]) {
      currentAngle += 1;
      if (currentAngle > 100) currentAngle = 100;
      shaft.write(currentAngle);
//      delay(10);
  }
  
  // DOWN
  if (ps4_btn[BTN_X]) {
      currentAngle -= 1;
      if (currentAngle < 0) currentAngle = 0;
      shaft.write(currentAngle);
//      delay(10);
  }


  /* ================= ARM STEP ================= */

  if (ps4_btn_pulse[BTN_DPAD_UP]) {
    armSP_ticks += 10 * ticks_per_deg;
  }

  if (ps4_btn_pulse[BTN_DPAD_DOWN]) {
    armSP_ticks -= 10 * ticks_per_deg;
  }

  /* ================= GRIPPER SERVO ================= */

  // UP
  if (ps4_btn[BTN_SQUARE]) {
      currentAngle2 += 1;
      if (currentAngle2 > 150) currentAngle2 = 150;
      grip.write(currentAngle2);
//      delay(10);
  }
  
  // DOWN
  if (ps4_btn[BTN_O]) {
      currentAngle2 -= 1;
      if (currentAngle2 < 40) currentAngle2 = 40;
      grip.write(currentAngle2);
//      delay(10);
  }

  /* ================= PNEUMATICS (EDGE TRIGGERED) ================= */

 if (ps4_btn_pulse[BTN_R2]) {
  servo_state = !servo_state;
  if(servo_state) servo1.write(60);
  else servo1.write(0);
}

if (ps4_btn_pulse[BTN_L2]) {
  climber_state = !climber_state;
  digitalWrite(climber_solenoid, climber_state);
//  if(climber_state) servo1.write(60);
//  else servo1.write(0);
}


  /* ================= CENTER SERVO ================= */

  if (ps4_btn_pulse[BTN_L1]) {
    centerPos -= 10;
    centerPos = constrain(centerPos, center_min, center_max);
    centerServo.write(centerPos);
  }

  if (ps4_btn_pulse[BTN_R1]) {
    centerPos += 10;
    centerPos = constrain(centerPos, center_min, center_max);
    centerServo.write(centerPos);
  }

  /* ================= CONVEYOR ================= */

  if (ps4_btn[BTN_DPAD_RIGHT]) {
    runMotor(conveyorSpeed, convLPWM, convRPWM);
  }
  else if (ps4_btn[BTN_DPAD_LEFT]) {
    runMotor(-conveyorSpeed, convLPWM, convRPWM);
  }
  else {
    runMotor(0, convLPWM, convRPWM);
  }
}


// === MAIN LOOP ===
void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  handleMechanisms();
  
  // Periodic publishing at ~20Hz to match Python
  unsigned long current_time = millis();
  if (current_time - last_publish_time >= PUBLISH_INTERVAL) {
    RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
    
//    Serial.printf("vx:%.2f, vy:%.2f, pid_vx:%.2f, pid_vy:%.2f", 
//                  state_data[0], state_data[1], state_data[2], state_data[3]);
//    Serial.println();
    
    last_publish_time = current_time;
  } 

//  unsigned long currTime = millis();
//  if (currTime - lastTime >= 1) {  
//      float dt = (currTime - lastTime) / 1000.0; 
//      
//        ax = phone_ax;
//        ay = phone_ay;
//      
//        vx =vx+ ax * dt*100;    // actual vel in x in cm/s
//        vy = vy+ay * dt*100;    // actual vel in y
//      
//        vx =vx* 0.95;
//        vy =vy* 0.95;
//            
//      lastTime = currTime;
//  }

//  if (debug_counter++ % 100 == 0) {  // Print every 100ms
//    Serial.printf("Rate Limited - LX:%.1f, LY:%.1f, RX:%.1f", ps4_lx, ps4_ly, ps4_rx);
//    Serial.println();
//}

  

  
}