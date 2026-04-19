// drive_eva_odom.ino — EVA Drive Base
//
// Subscribes: /teensy_drive  (geometry_msgs/Twist) → mecanum wheel commands
// Publishes:  /wheel_odom   (geometry_msgs/Twist) @ 50 Hz
//   angular.z = yaw rate (rad/s) — BNO055 gyro-z, raw (not integrated)
//   linear.x  = 0  (not used yet)
//   linear.y  = 0  (not used yet)

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ═══════════════════════════════════════════════════════════════
//  CONSTANTS
// ═══════════════════════════════════════════════════════════════

// ── Motor PWM pins ───────────────────────────────────────────
// Exact copy from drive_eva_uros. Each entry: [pwmL, pwmR].
int drive_swedish[4][2] = {
    {3, 2},   // FL: pwmL, pwmR
    {4, 5},   // FR: pwmL, pwmR
    {7, 6},   // BR: pwmL, pwmR
    {1, 0}    // BL: pwmL, pwmR
};
int max_pwm = 6000;

// ── Safety ───────────────────────────────────────────────────
static const uint32_t CMD_TIMEOUT_MS = 500;  // ms without /teensy_drive → stop motors

// ── Publish rate ─────────────────────────────────────────────
static const unsigned int ODOM_PERIOD_MS = 20;  // 50 Hz

// ── BNO055 ───────────────────────────────────────────────────
// I2C address 0x28, Wire (Teensy 4.1: SDA=18, SCL=19)
// Change to 0x29 if ADR pin is pulled HIGH.
static const uint8_t BNO_I2C_ADDR = 0x28;

// ═══════════════════════════════════════════════════════════════
//  micro-ROS handles
// ═══════════════════════════════════════════════════════════════
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rclc_executor_t  executor;

rcl_subscription_t        sub_drive;
geometry_msgs__msg__Twist msg_drive;

rcl_publisher_t           pub_odom;
geometry_msgs__msg__Twist msg_odom;

rcl_timer_t               timer_odom;

// ═══════════════════════════════════════════════════════════════
//  IMU
// ═══════════════════════════════════════════════════════════════
Adafruit_BNO055 bno(55, BNO_I2C_ADDR, &Wire1);

// ═══════════════════════════════════════════════════════════════
//  Watchdog
// ═══════════════════════════════════════════════════════════════
uint32_t last_cmd_ms = 0;

// ═══════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════
#define RCCHECK(fn)     { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = (fn); (void)_rc; }

void error_loop() {
    while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ── Motor helpers — exact copy from drive_eva_uros ───────────
void drive(int pwmL, int pwmR, int val) {
    analogWrite(pwmL, val < 0 ? abs(val) : 0);
    analogWrite(pwmR, val > 0 ? abs(val) : 0);
}

void stop_all() {
    for (int i = 0; i < 4; i++)
        drive(drive_swedish[i][0], drive_swedish[i][1], 0);
}

void apply_twist(int x, int y, int w) {
    drive(drive_swedish[0][0], drive_swedish[0][1], map((x + w/2), -128, 128, -max_pwm, max_pwm));
    drive(drive_swedish[1][0], drive_swedish[1][1], map((-y + w/2), -128, 128, -max_pwm, max_pwm));
    drive(drive_swedish[2][0], drive_swedish[2][1], map((x - w/2), -128, 128, -max_pwm, max_pwm));
    drive(drive_swedish[3][0], drive_swedish[3][1], map((-y - w/2), -128, 128, -max_pwm, max_pwm));
}

// ═══════════════════════════════════════════════════════════════
//  /teensy_drive subscriber callback
// ═══════════════════════════════════════════════════════════════
void drive_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *t = (const geometry_msgs__msg__Twist *)msgin;
    last_cmd_ms = millis();
    int x = int(t->linear.x  * 128);
    int y = int(t->linear.y  * 128);
    int w = int(t->angular.z * 128);
    apply_twist(x, y, w);
}

// ═══════════════════════════════════════════════════════════════
//  50 Hz timer callback — read BNO055 gyro-z, publish /wheel_odom
// ═══════════════════════════════════════════════════════════════
void timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    msg_odom.linear.x  = 0.0;
    msg_odom.linear.y  = 0.0;
    msg_odom.angular.z = gyro.z();  // rad/s, raw — Jetson integrates to get yaw

    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// ═══════════════════════════════════════════════════════════════
//  setup
// ═══════════════════════════════════════════════════════════════
void setup() {
    
    Serial.begin(115200);
    Serial.println("hiiiii");

    set_microros_transports();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    analogWriteResolution(14);
    analogWriteFrequency(1, 9000);

    stop_all();

    // ── BNO055 ────────────────────────────────────────────────
    if (!bno.begin()) {
        Serial.println("[BNO055] INIT FAILED — check SDA/SCL wiring and I2C address");
        error_loop();
    }
    bno.setExtCrystalUse(true);
    Serial.println("[BNO055] OK");

    delay(2000);

    // ── micro-ROS ─────────────────────────────────────────────
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "eva_drive", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &sub_drive, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "teensy_drive"));

    RCCHECK(rclc_publisher_init_default(
        &pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "wheel_odom"));

    RCCHECK(rclc_timer_init_default(
        &timer_odom, &support,
        RCL_MS_TO_NS(ODOM_PERIOD_MS),
        timer_callback));

    // 2 handles: 1 subscription + 1 timer
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_drive, &msg_drive, &drive_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));

    Serial.println("[micro-ROS] node eva_drive ready");
}

// ═══════════════════════════════════════════════════════════════
//  loop
// ═══════════════════════════════════════════════════════════════
void loop() {
    if (last_cmd_ms != 0 && (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
        stop_all();
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
