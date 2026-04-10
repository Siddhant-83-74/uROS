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
#include <std_msgs/msg/u_int16.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  Robot Head Firmware — micro-ROS / PlatformIO
//  Subscribers:
//    /talk_state   std_msgs/msg/String
//    /neck_angle   std_msgs/msg/UInt16  (0–180 deg, direct to servo)
// ═══════════════════════════════════════════════════════════════════════════════
// ── Camera config ──────────────────────────────────────────────────────────────
#define FRAME_WIDTH       640
#define FRAME_CENTER_X    (FRAME_WIDTH / 2.0f)   // 320.0 px

// ── PCA9685 channels ───────────────────────────────────────────────────────────
#define CH_EYE_TL         2
#define CH_EYE_TR         3
#define CH_EYE_BL         4
#define CH_EYE_BR         5
#define CH_JAW            14
#define CH_LIP            15

// ── Servo pulse constants ──────────────────────────────────────────────────────
#define SERVO_MIN         150
#define SERVO_MAX         600
#define PCA_FREQ          50

// ── Neck servo ─────────────────────────────────────────────────────────────────
#define NECK_PIN          15
#define NECK_PIN_Y        14
#define NECK_MIN_DEG      0
#define NECK_MAX_DEG      180
#define NECK_CENTER_DEG   100.0f
#define NECK_LEFT_DEG     135.0f
#define NECK_RIGHT_DEG    45.0f
#define NECK_MAX_DEFLECT  60.0f
#define NECK_DEAD_BAND    15.0f

// ── Eye positions ──────────────────────────────────────────────────────────────
#define EYE_OPEN_TICK     ((SERVO_MIN + SERVO_MAX) / 2 + 80)
#define EYE_CLOSED_TICK   SERVO_MIN

// ── Jaw / Lip positions ────────────────────────────────────────────────────────
#define JAW_REST_TICK     SERVO_MIN
#define JAW_OPEN_TICK     (SERVO_MIN + 120)
#define LIP_REST_TICK     SERVO_MIN
#define LIP_OPEN_TICK     (SERVO_MIN + 80)

// ── Timing ─────────────────────────────────────────────────────────────────────
#define EASE_STEPS        40
#define EASE_STEP_MS      10
#define BLINK_CLOSE_MS    80
#define BLINK_OPEN_MS     60

// ── Talk oscillator bounds ──────────────────────────────────────────────────────
#define TALK_FREQ_MIN     1.2f   // Hz — slow open vowel
#define TALK_FREQ_MAX     2.8f   // Hz — fast consonant burst  (realistic: 1–3 syl/s)
#define TALK_AMP_MIN      0.12f  // barely open
#define TALK_AMP_MAX      0.50f  // widest open (was 1.0 — now half travel)
#define TALK_VAR_MIN_MS   280    // ms between syllable variations
#define TALK_VAR_MAX_MS   520    // ms between syllable variations
#define TALK_AMP_LERP     0.08f  // per-frame blend toward target amp (smooths jumps)

// ── Objects ────────────────────────────────────────────────────────────────────
Servo                    neckServo;
Servo                    neckServoY;
Adafruit_PWMServoDriver  pca = Adafruit_PWMServoDriver(0x40, Wire2);

// ── micro-ROS handles ──────────────────────────────────────────────────────────
rcl_node_t               node;
rcl_subscription_t       talk_sub;
rcl_subscription_t       offset_sub;
rclc_executor_t          executor;
rclc_support_t           support;
rcl_allocator_t          allocator;

#define CMD_BUF_MAX  32
char                          cmd_buf[CMD_BUF_MAX];
std_msgs__msg__String         talk_msg;
std_msgs__msg__UInt16         angle_msg;

// ── Neck easing state ──────────────────────────────────────────────────────────
float    g_neckCurrent = NECK_CENTER_DEG;
float    g_neckTarget  = NECK_CENTER_DEG;
float    g_neckStart   = NECK_CENTER_DEG;
bool     g_neckMoving  = false;
int      g_easeStep    = 0;
uint32_t g_lastStepMs  = 0;

// ── Talk state ─────────────────────────────────────────────────────────────────
//  Jaw is driven by a rectified sine oscillator:
//    openness(t) = |sin(phase)| * amplitude
//  Frequency and amplitude are re-randomised every TALK_VAR_{MIN,MAX}_MS to
//  simulate the natural rhythm of syllables in human speech.
bool     g_talking        = false;
float    g_talkPhase      = 0.0f;   // oscillator phase [0, 2π)
float    g_talkFreq       = 2.0f;   // current jaw-cycle frequency, Hz
float    g_talkAmp        = 0.3f;   // smoothed amplitude (lerped each frame)
float    g_talkAmpTarget  = 0.3f;   // target amplitude (randomised per syllable)
uint32_t g_talkLastMs     = 0;      // timestamp of last updateTalkAnimation call
uint32_t g_talkNextVarMs  = 0;      // when to next randomise freq & amp

// ── Blink state ────────────────────────────────────────────────────────────────
enum BlinkState { BLINK_IDLE, BLINK_CLOSING, BLINK_OPENING };
BlinkState g_blinkState = BLINK_IDLE;
uint32_t   g_blinkMs    = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════════

void pcaWrite(uint8_t ch, int tick) {
    pca.setPWM(ch, 0, constrain(tick, SERVO_MIN, SERVO_MAX));
}

float easeOutQuart(float t) {
    float u = 1.0f - t;
    return 1.0f - u * u * u * u;
}

void setNeckImmediate(float deg) {
    deg = constrain(deg, (float)NECK_MIN_DEG, (float)NECK_MAX_DEG);
    neckServo.write((int)deg);
    g_neckCurrent = deg;
}

void startNeckMove(float targetDeg) {
    targetDeg = constrain(targetDeg, (float)NECK_MIN_DEG, (float)NECK_MAX_DEG);
    if (fabsf(targetDeg - g_neckTarget) < 0.5f && g_neckMoving) return;
    g_neckStart  = g_neckCurrent;
    g_neckTarget = targetDeg;
    g_easeStep   = 0;
    g_neckMoving = true;
    g_lastStepMs = millis();
}

void updateNeckEasing() {
    if (!g_neckMoving) return;
    uint32_t now = millis();
    if (now - g_lastStepMs < EASE_STEP_MS) return;
    g_lastStepMs = now;
    g_easeStep++;
    if (g_easeStep >= EASE_STEPS) {
        setNeckImmediate(g_neckTarget);
        g_neckMoving = false;
        return;
    }
    float t   = (float)g_easeStep / (float)EASE_STEPS;
    float deg = g_neckStart + (g_neckTarget - g_neckStart) * easeOutQuart(t);
    setNeckImmediate(deg);
}

void setEyes(int tick) {
    pcaWrite(CH_EYE_TL, tick);
    pcaWrite(CH_EYE_TR, tick);
    pcaWrite(CH_EYE_BL, tick);
    pcaWrite(CH_EYE_BR, tick);
}

// Snap jaw/lip to fully open or rest (used by not_talking command)
void setJawLip(bool open) {
    pcaWrite(CH_JAW, open ? JAW_OPEN_TICK : JAW_REST_TICK);
    pcaWrite(CH_LIP, open ? LIP_OPEN_TICK : LIP_REST_TICK);
}

// Smooth jaw/lip drive — openness in [0.0, 1.0]
void setJawLipF(float openness) {
    openness = constrain(openness, 0.0f, 1.0f);
    pcaWrite(CH_JAW, JAW_REST_TICK + (int)(openness * (JAW_OPEN_TICK - JAW_REST_TICK)));
    pcaWrite(CH_LIP, LIP_REST_TICK + (int)(openness * (LIP_OPEN_TICK - LIP_REST_TICK)));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════════════════════

void talk_state_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    if (!msg->data.data || msg->data.size == 0) return;
    const char *cmd = msg->data.data;

    if (strncmp(cmd, "talking", 7) == 0) {
        if (!g_talking) {               // fresh start — reset oscillator
            g_talkPhase      = 0.0f;
            g_talkFreq       = 2.0f;
            g_talkAmp        = 0.3f;
            g_talkAmpTarget  = 0.3f;
            g_talkLastMs     = millis();
            g_talkNextVarMs  = millis();
        }
        g_talking = true;

    } else if (strncmp(cmd, "not_talking", 11) == 0) {
        g_talking = false;
        setJawLip(false);

    } else if (strncmp(cmd, "blink", 5) == 0) {
        if (g_blinkState == BLINK_IDLE) {
            g_blinkState = BLINK_CLOSING;
            g_blinkMs    = millis();
            setEyes(EYE_CLOSED_TICK);
        }

    } else if (strncmp(cmd, "left", 4) == 0) {
        startNeckMove(NECK_LEFT_DEG);

    } else if (strncmp(cmd, "right", 5) == 0) {
        startNeckMove(NECK_RIGHT_DEG);

    } else if (strncmp(cmd, "front", 5) == 0) {
        startNeckMove(NECK_CENTER_DEG);
    }
}

void neck_angle_callback(const void *msgin) {
    const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;
    uint16_t angle = msg->data;

    Serial.print("[NECK] received angle="); Serial.println(angle);
    startNeckMove((float)angle);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Tick functions
// ═══════════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────────────────────
//  updateTalkAnimation()
//
//  Drives jaw + lip with a rectified-sine oscillator:
//
//    phase(t)     += 2π · freq · Δt          (phase accumulator)
//    openness(t)   = |sin(phase)| · amplitude (always ≥ 0, peaks at amplitude)
//
//  Every 120–320 ms the frequency and amplitude are re-randomised to
//  approximate the variable intensity of real syllables:
//    freq  ∈ [3.0, 6.0] Hz  →  slow open vowels … fast consonant bursts
//    amp   ∈ [0.35, 1.0]    →  quiet murmur … wide-open speech
// ─────────────────────────────────────────────────────────────────────────────
void updateTalkAnimation() {
    if (!g_talking) return;

    uint32_t now = millis();

    // Δt in seconds; clamp to avoid a large jump after a pause
    float dt = (now - g_talkLastMs) * 0.001f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.016f;
    g_talkLastMs = now;

    // Re-randomise freq & amp target at irregular intervals → syllable rhythm
    if (now >= g_talkNextVarMs) {
        g_talkFreq      = TALK_FREQ_MIN
                        + random(0, (int)((TALK_FREQ_MAX - TALK_FREQ_MIN) * 10)) * 0.1f;
        g_talkAmpTarget = TALK_AMP_MIN
                        + random(0, (int)((TALK_AMP_MAX - TALK_AMP_MIN) * 100)) * 0.01f;
        g_talkNextVarMs = now + TALK_VAR_MIN_MS + random(0, TALK_VAR_MAX_MS - TALK_VAR_MIN_MS);
    }

    // Smooth amplitude toward target — prevents abrupt jumps that look like shivering
    g_talkAmp += (g_talkAmpTarget - g_talkAmp) * TALK_AMP_LERP;

    // Advance oscillator phase
    g_talkPhase += TWO_PI * g_talkFreq * dt;
    if (g_talkPhase >= TWO_PI) g_talkPhase -= TWO_PI;

    // Rectified sine: jaw returns to rest twice per cycle (natural closing)
    float openness = fabsf(sinf(g_talkPhase)) * g_talkAmp;

    setJawLipF(openness);
}

void updateBlink() {
    if (g_blinkState == BLINK_IDLE) return;
    uint32_t now = millis();
    if (g_blinkState == BLINK_CLOSING && now - g_blinkMs >= BLINK_CLOSE_MS) {
        g_blinkState = BLINK_OPENING;
        g_blinkMs    = now;
        setEyes(EYE_OPEN_TICK);
    } else if (g_blinkState == BLINK_OPENING && now - g_blinkMs >= BLINK_OPEN_MS) {
        g_blinkState = BLINK_IDLE;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Error guard
// ═══════════════════════════════════════════════════════════════════════════════
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { errorLoop(); } }
void errorLoop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);    // USB debug — free because micro-ROS uses Ethernet
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("[SETUP] start");

    // ── Ethernet micro-ROS transport ──
    byte     local_mac[] = LOCAL_MAC;
    IPAddress local_ip   = LOCAL_IP;
    IPAddress agent_ip   = AGENT_IP;
    set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, AGENT_PORT);
    Serial.print("[SETUP] Ethernet transport init — agent ");
    Serial.print(agent_ip);
    Serial.print(":");
    Serial.println(AGENT_PORT);

    // Neck — centre on boot
    neckServo.attach(NECK_PIN);
    neckServoY.attach(NECK_PIN_Y);

    neckServoY.write(80);
    setNeckImmediate(NECK_CENTER_DEG);

    // PCA9685 on Wire2
    Wire2.begin();
    pca.begin();
    pca.setPWMFreq(PCA_FREQ);

    // Boot positions
    setEyes(EYE_OPEN_TICK);
    setJawLip(false);

    // Pre-allocate String buffer for talk_msg
    talk_msg.data.data     = cmd_buf;
    talk_msg.data.size     = 0;
    talk_msg.data.capacity = CMD_BUF_MAX;

    delay(2000);  // wait for micro-ROS agent

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "robot_head_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &talk_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "talk_state"
    ));

    RCCHECK(rclc_subscription_init_default(
        &offset_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
        "neck_angle"
    ));

    // 2 handles — one per subscriber
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &talk_sub,   &talk_msg,   &talk_state_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &offset_sub, &angle_msg,  &neck_angle_callback, ON_NEW_DATA));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    updateNeckEasing();
    updateTalkAnimation();
    updateBlink();
}
