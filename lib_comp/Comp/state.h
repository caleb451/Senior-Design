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
            Serial.println("Flag Starting");
            dropFlag(1000);
            motorsFWD(250);
            motorsOFF(10);
            dropFlag(1000);
            motorsROT_LEFT(100);
            motorsOFF(10);
            Serial.println("Flag Stopping");
            delay(1000);
        break;

        case STATE_SWEEP:
            Serial.println("Sweep Starting");
            Serial.println("Sweep Stopping");
        break;

        case STATE_BIN1:
            Serial.println("Bin Starting");
            liftBin(1000);
            Serial.println("Bin Stopping");
        break;

        case STATE_BIN2:
            Serial.println("Bin2 Starting");
            liftBin(1000);
            Serial.println("Bin Stopping");
        break;

        case STATE_CAVE:
            Serial.println("Cave Starting");
            Serial.println("Cave Stopping");
        break;

        case STATE_CAVE_SWEEP:
            Serial.println("Cave Sweep Starting");
            Serial.println("Cave Sweep Stopping");
        break;

        case STATE_STOP:
        default:
            Serial.println("Motors Off");
            GPIO.out_w1tc = (1UL << COLLECTION);
            motorsOFF();
        break;
    }
}

#endif