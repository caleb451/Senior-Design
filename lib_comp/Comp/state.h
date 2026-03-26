#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "Motors/fastfunctions.h"

// State IDs for competition tasks
enum CompState {
    STATE_FLAG = 1,
    STATE_SWEEP = 2,
    STATE_BIN1 = 3,
    STATE_BIN2 = 4,
    STATE_CAVE = 5,
    STATE_CAVE_SWEEP = 6,
    STATE_STOP = 0
};

//Function to run the states
inline void hardcode_state(int state, uint8_t speed) {
    motorsSetSpeed(speed);
    
    switch(state) {
        case STATE_FLAG:
            motorsFWD();
            delay(2000);
            motorsBKWD();
        break;

        case STATE_SWEEP:
        break;

        case STATE_BIN1:
        break;

        case STATE_BIN2:
        break;

        case STATE_CAVE:
        break;

        case STATE_CAVE_SWEEP:
        break;

        case STATE_STOP:
        default:
            motorsOFF();
        break;
    }
}

#endif