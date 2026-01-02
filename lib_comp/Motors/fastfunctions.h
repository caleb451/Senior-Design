#ifndef FAST_FUNCTIONS_H
#define FAST_FUNCTIONS_H

#include <Arduino.h>
#include "soc/gpio_struct.h"

#define M1_FWD  13
#define M1_BKWD 12
#define M2_FWD  14
#define M2_BKWD 27
#define M3_FWD  26
#define M3_BKWD 25
#define M4_FWD  33
#define M4_BKWD 32
#define PWM 10

//This is to mask all the gpio pins simultaneously 
constexpr uint32_t ALL_MOTOR_PINS = (1UL << M1_FWD) | (1UL << M1_BKWD) | (1UL << M2_FWD) | (1UL << M2_BKWD) |
                                    (1UL << M3_FWD) | (1UL << M3_BKWD) | (1UL << M4_FWD) | (1UL << M4_BKWD);

constexpr uint32_t FWD_PINS =       (1UL << M1_FWD) | (1UL << M2_FWD) | (1UL << M3_FWD) | (1UL << M4_FWD);

constexpr uint32_t BKWD_PINS =      (1UL << M1_BKWD) | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_BKWD);
#endif