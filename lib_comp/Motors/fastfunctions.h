#ifndef FAST_FUNCTIONS_H
#define FAST_FUNCTIONS_H

#include <Arduino.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"
#include <ESP32Servo.h>

Servo doorServo;
Servo armServo;

static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2400;
static const uint8_t SERVO_DOOR_CLOSED_ANGLE = 90;
static const uint8_t SERVO_DOOR_OPEN_ANGLE = 0;
static const uint8_t SERVO_ARM_HOME_ANGLE = 45;
static const uint8_t SERVO_ARM_DROP_ANGLE = 120;
static const uint8_t SERVO_ARM_LIFT_ANGLE = 180;

//Photoresistor Pinout
#define photo 25

//Servo Pinouts
#define servo_door 23
#define servo_arm  33

//Motor Pinouts
#define M1_FWD  18
#define M1_BKWD 19
#define M2_FWD  16
#define M2_BKWD 17
#define M3_FWD  27
#define M3_BKWD 26
#define M4_FWD  13
#define M4_BKWD 12
#define COLLECTION 21

// This is to mask all the gpio pins simultaneously using bitwise ORs
// exp: GPIO pin 2 needs to be enabled (1), In a 32 bit system : 0010 -pin 2 enabled.
constexpr uint32_t ALL_MOTOR_MASK = (1UL << M1_FWD) | (1UL << M1_BKWD) | (1UL << M2_FWD) | (1UL << M2_BKWD) |
                                    (1UL << M3_FWD) | (1UL << M3_BKWD) | (1UL << M4_FWD) | (1UL << M4_BKWD);

constexpr uint32_t FWD_MASK =       (1UL << M1_FWD) | (1UL << M2_FWD) | (1UL << M3_FWD) | (1UL << M4_FWD);

constexpr uint32_t BKWD_MASK =      (1UL << M1_BKWD) | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_BKWD);

// Servo masking for the pins
constexpr uint64_t SERVO_MASK =      (1UL << servo_door) | (1ULL << servo_arm);
constexpr uint32_t COLLECTION_MASK = (1UL << COLLECTION);

// Holonomic strafing (aligns with original motors_left/motors_right mapping)
constexpr uint32_t STRAFE_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M2_FWD)  | (1UL << M3_FWD)  | (1UL << M4_BKWD);
constexpr uint32_t STRAFE_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_FWD);

// In-place rotation (matches motors_Rleft/motors_Rright)
constexpr uint32_t ROT_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M3_BKWD) | (1UL << M2_FWD)  | (1UL << M4_FWD);
constexpr uint32_t ROT_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M3_FWD)  | (1UL << M2_BKWD) | (1UL << M4_BKWD);

// LEDC PWM setup
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_RES_BITS = 8;
constexpr uint8_t CH_M1_FWD = 8;
constexpr uint8_t CH_M1_BKWD = 9;
constexpr uint8_t CH_M2_FWD = 10;
constexpr uint8_t CH_M2_BKWD = 11;
constexpr uint8_t CH_M3_FWD = 12;
constexpr uint8_t CH_M3_BKWD = 13;
constexpr uint8_t CH_M4_FWD = 14;
constexpr uint8_t CH_M4_BKWD = 15;
static uint8_t motorSpeed = 255;

// Inline lets the functions compile to direct registers instead of pushing arguments or returning (optimized)
inline void motorsSetSpeed(uint8_t speed){
    motorSpeed = speed;
}

inline void motorsOFF(uint32_t waitMs = 0){
    //GPIO.out_w1tc = COLLECTION;
    ledcWrite(CH_M1_FWD, 0);
    ledcWrite(CH_M1_BKWD, 0);
    ledcWrite(CH_M2_FWD, 0);
    ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_FWD, 0);
    ledcWrite(CH_M3_BKWD, 0);
    ledcWrite(CH_M4_FWD, 0);
    ledcWrite(CH_M4_BKWD, 0);
    delay(waitMs);
}
inline void motorsFWD(uint32_t waitMs = 0){
    ledcWrite(CH_M1_BKWD, 0);
    ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_BKWD, 0);
    ledcWrite(CH_M4_BKWD, 0);
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
    delay(waitMs);
}
inline void motorsBKWD(uint32_t waitMs = 0){
    ledcWrite(CH_M1_FWD, 0);
    ledcWrite(CH_M2_FWD, 0);
    ledcWrite(CH_M3_FWD, 0);
    ledcWrite(CH_M4_FWD, 0);
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
    delay(waitMs);
}
inline void motorsSTRAFE_LEFT(uint32_t waitMs = 0){
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
    delay(waitMs);
}
inline void motorsSTRAFE_RIGHT(uint32_t waitMs = 0){
    motorsOFF();
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
    delay(waitMs);
}
inline void motorsROT_LEFT(uint32_t waitMs = 0){
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
    delay(waitMs);
}
inline void motorsROT_RIGHT(uint32_t waitMs = 0){
    motorsOFF();
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
    delay(waitMs);
}

inline void motorsSetWheelPwm(int pwmM1, int pwmM2, int pwmM3, int pwmM4) {
    auto apply = [](int fwdCh, int bkwdCh, int pwm) {
        if (pwm >= 0) {
            ledcWrite(bkwdCh, 0);
            ledcWrite(fwdCh, pwm);
        } else {
            ledcWrite(fwdCh, 0);
            ledcWrite(bkwdCh, -pwm);
        }
    };

    apply(CH_M1_FWD, CH_M1_BKWD, pwmM1);
    apply(CH_M2_FWD, CH_M2_BKWD, pwmM2);
    apply(CH_M3_FWD, CH_M3_BKWD, pwmM3);
    apply(CH_M4_FWD, CH_M4_BKWD, pwmM4);
}

inline void motorsInit(){
    ledcSetup(CH_M1_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M1_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M2_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M2_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M3_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M3_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M4_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M4_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);

    ledcAttachPin(M1_FWD, CH_M1_FWD);
    ledcAttachPin(M1_BKWD, CH_M1_BKWD);
    ledcAttachPin(M2_FWD, CH_M2_FWD);
    ledcAttachPin(M2_BKWD, CH_M2_BKWD);
    ledcAttachPin(M3_FWD, CH_M3_FWD);
    ledcAttachPin(M3_BKWD, CH_M3_BKWD);
    ledcAttachPin(M4_FWD, CH_M4_FWD);
    ledcAttachPin(M4_BKWD, CH_M4_BKWD);
    motorsOFF();
}

inline void collectionInit(){
    gpio_config_t conf = {};
    conf.pin_bit_mask = COLLECTION_MASK;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    GPIO.out_w1tc = COLLECTION;
}

inline void servosInit() {
    doorServo.setPeriodHertz(50);
    armServo.setPeriodHertz(50);
    doorServo.attach(servo_door, SERVO_MIN_US, SERVO_MAX_US);
    armServo.attach(servo_arm, SERVO_MIN_US, SERVO_MAX_US);
    doorServo.write(SERVO_DOOR_CLOSED_ANGLE);
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(250);
}

inline void openDoor(uint32_t time = 0){
    doorServo.write(SERVO_DOOR_OPEN_ANGLE);
    delay(time);
}

inline void closeDoor(uint32_t time = 0){
    doorServo.write(SERVO_DOOR_CLOSED_ANGLE);
    delay(time);
}

inline void dropFlag(uint32_t time = 0){
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(500 + time);
    armServo.write(SERVO_ARM_DROP_ANGLE);
    delay(500 + time);
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(500 + time);
}

inline void liftBin(uint32_t time = 0){
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(time);
    armServo.write(SERVO_ARM_DROP_ANGLE);
    delay(time);
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(time);
}

// Servo diagnostic: cycles both servos through their key positions.
inline void servoRotationTest(uint8_t cycles = 2, uint32_t dwellMs = 400) {
    for (uint8_t i = 0; i < cycles; i++) {
        openDoor(dwellMs);
        closeDoor(dwellMs);

        armServo.write(SERVO_ARM_HOME_ANGLE);
        delay(dwellMs);
        armServo.write(SERVO_ARM_DROP_ANGLE);
        delay(dwellMs);
        armServo.write(SERVO_ARM_LIFT_ANGLE);
        delay(dwellMs);
        armServo.write(SERVO_ARM_HOME_ANGLE);
        delay(dwellMs);
    }
}
#endif