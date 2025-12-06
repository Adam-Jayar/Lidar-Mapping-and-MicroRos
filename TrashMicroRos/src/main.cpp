#include <Arduino.h>
#include <micro_ros_arduino.h> // <--- Check this line
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

// --- ROS Setup ---
rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;

// --- Motor & Encoder Setup ---
// Motor A (Right)
#define EN_A 12
#define IN1_A 13
#define IN2_A 14
#define ENC_A_CHA 34
#define ENC_A_CHB 35
volatile long encoder_ticks_R = 0;

// Motor B (Left)
#define EN_B 21
#define IN3_B 22
#define IN4_B 23
#define ENC_B_CHA 25
#define ENC_B_CHB 26
volatile long encoder_ticks_L = 0;

// --- Kinematics ---
const float WHEEL_BASE = 0.30; 

// --- PWM Setup ---
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelA = 0;
const int pwmLedChannelB = 1;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Error: %d\n", (int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- Interrupts ---
void IRAM_ATTR readEncoderRight() {
    if (digitalRead(ENC_A_CHA) == HIGH) {
        if (digitalRead(ENC_A_CHB) == LOW) encoder_ticks_R++; else encoder_ticks_R--;
    } else {
        if (digitalRead(ENC_A_CHB) == HIGH) encoder_ticks_R++; else encoder_ticks_R--;
    }
}

void IRAM_ATTR readEncoderLeft() {
    if (digitalRead(ENC_B_CHA) == HIGH) {
        if (digitalRead(ENC_B_CHB) == HIGH) encoder_ticks_L++; else encoder_ticks_L--;
    } else {
        if (digitalRead(ENC_B_CHB) == LOW) encoder_ticks_L++; else encoder_ticks_L--;
    }
}

void set_motor_speed(int motor, float linear_vel_mps) {
    float max_vel = 0.5; 
    int pwm = (int)(abs(linear_vel_mps) / max_vel * 255);
    pwm = constrain(pwm, 0, 255);

    if (motor == 0) { // Right
        ledcWrite(pwmLedChannelA, pwm);
        if (linear_vel_mps > 0) { digitalWrite(IN1_A, HIGH); digitalWrite(IN2_A, LOW); }
        else if (linear_vel_mps < 0) { digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, HIGH); }
        else { digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, LOW); }
    } else { // Left
        ledcWrite(pwmLedChannelB, pwm);
        if (linear_vel_mps > 0) { digitalWrite(IN3_B, HIGH); digitalWrite(IN4_B, LOW); }
        else if (linear_vel_mps < 0) { digitalWrite(IN3_B, LOW); digitalWrite(IN4_B, HIGH); }
        else { digitalWrite(IN3_B, LOW); digitalWrite(IN4_B, LOW); }
    }
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *twist_ptr = (const geometry_msgs__msg__Twist *)msgin;
    float vr = twist_ptr->linear.x + twist_ptr->angular.z * (WHEEL_BASE / 2.0);
    float vl = twist_ptr->linear.x - twist_ptr->angular.z * (WHEEL_BASE / 2.0);
    set_motor_speed(0, vr);
    set_motor_speed(1, vl);
}

unsigned long last_publish_time = 0;

void setup() {
    Serial.begin(115200);
    pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT);
    pinMode(IN3_B, OUTPUT); pinMode(IN4_B, OUTPUT);
    ledcSetup(pwmLedChannelA, pwmFreq, pwmResolution); ledcAttachPin(EN_A, pwmLedChannelA);
    ledcSetup(pwmLedChannelB, pwmFreq, pwmResolution); ledcAttachPin(EN_B, pwmLedChannelB);
    pinMode(ENC_A_CHA, INPUT); pinMode(ENC_A_CHB, INPUT);
    pinMode(ENC_B_CHA, INPUT); pinMode(ENC_B_CHB, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_A_CHA), readEncoderRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_CHA), readEncoderLeft, CHANGE);

    set_microros_transports(); 
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));
    
    RCCHECK(rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom_raw")); 

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    
    if (millis() - last_publish_time >= 20) {
        last_publish_time = millis();
        unsigned long now = millis();
        odom_msg.header.stamp.sec = now / 1000;
        odom_msg.header.stamp.nanosec = (now % 1000) * 1000000;
        rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
        odom_msg.twist.twist.linear.x = (double)encoder_ticks_R;
        odom_msg.twist.twist.linear.y = (double)encoder_ticks_L;
        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
}