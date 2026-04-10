#include "CubeMarsServoDual.h"
#include <Wire.h>
#include "RMCS-220X_Right.h"
#include "RMCS-220X_Left.h"
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/byte_multi_array.h>
#include <std_msgs/msg/string.h>

// ===================== PCA9685 =====================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire2);

#define SERVO_MIN 150
#define SERVO_MAX 600

int angleToPulse(float angle)
{
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// ===================== HEAD SERVO =====================
Servo servo1;

float currentAngle = 90;
float startAngle   = 90;
float targetAngle  = 90;

bool          moving    = false;
unsigned long startTime = 0;
unsigned long duration  = 1500;

// ===================== SPEAK =====================
// speaking is purely flag-driven — no timer.
// Set true on "talking", false on "not_talking".
bool speaking = false;

// ===================== BLINK =====================
bool          blinking   = false;
unsigned long blinkTimer = 0;
unsigned long blinkSpeed = 70;
int           blinkState = 0;

// ===================== EASING =====================
float easeOutQuart(float x)
{
  return 1 - pow(1 - x, 4);
}

// ===================== CORE MOVEMENT =====================
void moveTo(float angle)
{
  startAngle  = currentAngle;
  targetAngle = angle;
  startTime   = millis();
  moving      = true;
}

void easingUpdate()
{
  if (!moving) return;

  float t = (millis() - startTime) / (float)duration;
  if (t >= 1.0f) { t = 1.0f; moving = false; }

  float e     = easeOutQuart(t);
  float angle = startAngle + (targetAngle - startAngle) * e;

  servo1.write(angle);
  currentAngle = angle;
}

// ===================== PCA SERVO =====================
void moveServoEase(int ch, float startA, float endA, unsigned long dur)
{
  unsigned long st = millis();
  while (millis() - st < dur)
  {
    float t   = (millis() - st) / (float)dur;
    float e   = easeOutQuart(t);
    float ang = startA + (endA - startA) * e;
    pca.setPWM(ch, 0, angleToPulse(ang));
  }
  pca.setPWM(ch, 0, angleToPulse(endA));
}

// ===================== EYES =====================
void eyesClosed()
{
  pca.setPWM(2, 0, angleToPulse(110));
  pca.setPWM(3, 0, angleToPulse(50));
  pca.setPWM(4, 0, angleToPulse(45));
  pca.setPWM(5, 0, angleToPulse(110));
}

void eyesOpen()
{
  pca.setPWM(2, 0, angleToPulse(45));
  pca.setPWM(3, 0, angleToPulse(75));
  pca.setPWM(4, 0, angleToPulse(90));
  pca.setPWM(5, 0, angleToPulse(60));
}

// ===================== TALK =====================
void talkStep()
{
  int openAmount          = random(170, 180);
  unsigned long speedTime = random(100, 140);

  moveServoEase(14, 170, openAmount, speedTime);
  moveServoEase(14, openAmount, 170, speedTime);

  int leftOpen = map(openAmount, 170, 180, 0, 10);
  moveServoEase(10, 0, leftOpen, speedTime);
  moveServoEase(10, leftOpen, 0, speedTime);
}

void stopTalking()
{
  // Return jaw and lip servos to closed/rest position
  pca.setPWM(14, 0, angleToPulse(180));
  pca.setPWM(15, 0, angleToPulse(0));
}

// speakUpdate() keeps running talkStep() as long as speaking == true.
// No timeout — only stops when "not_talking" is received.
void speakUpdate()
{
  if (!speaking) return;
  talkStep();
}

// ===================== BLINK =====================
// blinkState == 1: waiting for next blink (random 3-5 s interval)
// blinkState == 2: eyes closed, waiting 70 ms to reopen
void blinkUpdate()
{
  if (!blinking) return;

  if (blinkState == 1)
  {
    // waiting for next blink
    if (millis() - blinkTimer > blinkSpeed)
    {
      eyesClosed();
      blinkTimer = millis();
      blinkSpeed = 70;          // stay closed 70 ms
      blinkState = 2;
    }
  }
  else if (blinkState == 2)
  {
    // eyes are closed — reopen after 70 ms
    if (millis() - blinkTimer > blinkSpeed)
    {
      eyesOpen();
      blinkTimer = millis();
      blinkSpeed = random(3000, 5000);  // next blink in 3-5 s
      blinkState = 1;
    }
  }
}

// ===================== COMMAND HANDLER =====================

void executeCommand(const char* cmd)
{
  Serial.print("[CMD] executeCommand: \"");
  Serial.print(cmd);
  Serial.println("\"");

  if      (strcmp(cmd, "left")        == 0) { Serial.println("[CMD] → moveTo(45)");  moveTo(45); }
  else if (strcmp(cmd, "right")       == 0) { Serial.println("[CMD] → moveTo(135)"); moveTo(135); }
  else if (strcmp(cmd, "front")       == 0) { Serial.println("[CMD] → moveTo(100)"); moveTo(100); }

  else if (strcmp(cmd, "talking")     == 0)
  {
    Serial.println("[CMD] → speaking = true");
    speaking = true;
  }

  else if (strcmp(cmd, "not talking") == 0)
  {
    Serial.println("[CMD] → speaking = false, stopTalking()");
    speaking = false;
    stopTalking();
  }

  else if (strcmp(cmd, "blink")       == 0)
  {
    Serial.println("[CMD] → auto-blink started (random 3-5 s)");
    blinking   = true;
    blinkState = 1;
    blinkSpeed = random(3000, 5000);  // first interval
    blinkTimer = millis();
  }
  else if (strcmp(cmd, "stop blink")       == 0)
  {
    Serial.println("[CMD] → blink triggered");
    blinking   = false;
    blinkState = 1;
    blinkTimer = millis();
  }
  else
  {
    Serial.print("[CMD] UNKNOWN command: \"");
    Serial.print(cmd);
    Serial.println("\"");
  }
}

// ===================== micro-ROS objects =====================
rcl_subscription_t    talk_subscriber;
std_msgs__msg__String talk_msg;
char                  talk_buf[32];
volatile bool         is_talking = false;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// ===================== SUBSCRIBER CALLBACK =====================
void talk_state_callback(const void* msgin)
{
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  if (msg->data.data == NULL) {
    Serial.println("[CALLBACK] talk_state_callback: msg->data.data is NULL, ignoring");
    return;
  }

  size_t len = msg->data.size < (sizeof(talk_buf) - 1)
                 ? msg->data.size
                 : (sizeof(talk_buf) - 1);
  strncpy(talk_buf, msg->data.data, len);
  talk_buf[len] = '\0';

  Serial.print("[CALLBACK] /talk_state received: \"");
  Serial.print(talk_buf);
  Serial.print("\"  (size=");
  Serial.print(msg->data.size);
  Serial.print(", capacity=");
  Serial.print(msg->data.capacity);
  Serial.println(")");

  executeCommand(talk_buf);
}

// ===================== micro-ROS error loop =====================
void error_loop()
{
  Serial.println("[ERROR] micro-ROS init failed — stuck in error_loop. Check agent on Jetson.");
  while (1) { delay(100); }
}

#define RCCHECK(fn)     { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { Serial.print("[RCCHECK FAIL] "); Serial.print(#fn); Serial.print(" returned "); Serial.println(rc); error_loop(); } else { Serial.print("[RCCHECK OK]  "); Serial.println(#fn); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { Serial.print("[RCSOFTCHECK WARN] "); Serial.print(#fn); Serial.print(" returned "); Serial.println(rc); } }

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  delay(500);  // let Serial settle
  Serial.println("\n========== SETUP START ==========");

  // Head servo
  Serial.println("[SETUP] Attaching head servo on pin 15...");
  servo1.attach(15);
  servo1.write(90);
  Serial.println("[SETUP] Head servo attached, set to 90°");

  // PCA9685
  Serial.println("[SETUP] Initialising Wire / Wire2...");
  Wire.begin();
  Wire2.begin();   // PCA9685 is on Wire2 (0x40)
  Serial.println("[SETUP] Wire & Wire2 up");

  Serial.println("[SETUP] Starting PCA9685...");
  pca.begin();
  pca.setPWMFreq(50);
  Serial.println("[SETUP] PCA9685 ready at 50 Hz");

  randomSeed(analogRead(0));
  eyesOpen();
  stopTalking();
  Serial.println("[SETUP] Eyes open, jaw closed");

  // ---- micro-ROS init ----
  Serial.println("[SETUP] Setting micro-ROS transports...");
  set_microros_transports();  // swap for set_microros_wifi_transports() if using WiFi
  Serial.println("[SETUP] Transport set. Waiting 2 s for agent...");
  delay(2000);

  Serial.println("[SETUP] rcl_get_default_allocator...");
  allocator = rcl_get_default_allocator();

  Serial.println("[SETUP] rclc_support_init...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("[SETUP] rclc_node_init_default → node: robot_head_node...");
  RCCHECK(rclc_node_init_default(&node, "robot_head_node", "", &support));

  // IMPORTANT: point String msg at our static buffer BEFORE subscribing
  talk_msg.data.data     = talk_buf;
  talk_msg.data.size     = 0;
  talk_msg.data.capacity = sizeof(talk_buf);

  Serial.println("[SETUP] rclc_subscription_init_default → topic: /talk_state...");
  RCCHECK(rclc_subscription_init_default(
    &talk_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "talk_state"
  ));

  Serial.println("[SETUP] rclc_executor_init (1 handle)...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  Serial.println("[SETUP] rclc_executor_add_subscription...");
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &talk_subscriber,
    &talk_msg,
    &talk_state_callback,
    ON_NEW_DATA
  ));

  Serial.println("[SETUP] === micro-ROS ready. Node: robot_head_node | Topic: /talk_state ===");
  Serial.println("========== SETUP DONE ==========\n");
}

// ===================== LOOP =====================
static unsigned long lastStatusPrint = 0;

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  easingUpdate();
  speakUpdate();   // runs continuously as long as speaking == true
  blinkUpdate();

  // Print state once per second so Serial isn't flooded
  if (millis() - lastStatusPrint >= 1000)
  {
    lastStatusPrint = millis();
    Serial.print("[LOOP] speaking=");  Serial.print(speaking);
    Serial.print(" blinking=");        Serial.print(blinking);
    Serial.print(" moving=");          Serial.print(moving);
    Serial.print(" currentAngle=");    Serial.print(currentAngle, 1);
    Serial.print(" blinkState=");      Serial.println(blinkState);
  }

  delay(10);
}
