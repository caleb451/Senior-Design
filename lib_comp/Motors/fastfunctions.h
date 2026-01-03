#ifndef FAST_FUNCTIONS_H
#define FAST_FUNCTIONS_H

#include <Arduino.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"

#define M1_FWD  13
#define M1_BKWD 12
#define M2_FWD  14
#define M2_BKWD 27
#define M3_FWD  26
#define M3_BKWD 25
#define M4_FWD  11
#define M4_BKWD 15
#define PWM 10

// This is to mask all the gpio pins simultaneously using bitwise ORs
// exp: GPIO pin 2 needs to be enabled (1), In a 32 bit system : 0010 -pin 2 enabled.
constexpr uint32_t ALL_MOTOR_MASK = (1UL << M1_FWD) | (1UL << M1_BKWD) | (1UL << M2_FWD) | (1UL << M2_BKWD) |
                                    (1UL << M3_FWD) | (1UL << M3_BKWD) | (1UL << M4_FWD) | (1UL << M4_BKWD);

constexpr uint32_t FWD_MASK =       (1UL << M1_FWD) | (1UL << M2_FWD) | (1UL << M3_FWD) | (1UL << M4_FWD);

constexpr uint32_t BKWD_MASK =      (1UL << M1_BKWD) | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_BKWD);

// Inline lets the functions compile to direct registers instead of pushing arguments or returning (optimized)
inline void motorsOFF(){
    GPIO.out_w1tc = ALL_MOTOR_MASK; 
}
inline void motorsFWD(){
    GPIO.out_w1tc = BKWD_MASK;
    GPIO.out_w1ts = FWD_MASK;
}
inline void motorsBKWD(){
    GPIO.out_w1tc = FWD_MASK;
    GPIO.out_w1ts = BKWD_MASK;
}

inline void motorsInit(){
    gpio_config_t conf = {};
    conf.pin_bit_mask = ALL_MOTOR_MASK;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    motorsOFF();
}
#endif