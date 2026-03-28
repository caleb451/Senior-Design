#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Motors/fastfunctions.h"

// State IDs for competition tasks
enum CompState {
    STATE_BIN1 = 1,
    STATE_SWEEP = 2,
    STATE_CAVE = 3,
    STATE_FLAG = 4,
    STATE_BIN2 = 5,
    STATE_CAVE_SWEEP = 6,
    STATE_STOP = 0
};

//Function to run the states
inline void hardcode_state(int state, uint8_t speed) {
    motorsSetSpeed(speed);
    
    switch(state) {
        case STATE_FLAG:
            Serial.println("Flag Starting");
            motorsFWD(2000);
            motorsOFF(10);
            motorsROT_RIGHT(770);
            motorsOFF(10);
            motorsBKWD(2000);
            motorsOFF(10);
            dropFlag(1000);
            motorsFWD(4000);
            // motorsSTRAFE_LEFT(1500);
            // motorsOFF(10);
            // dropFlag(1000);
            // motorsFWD(200);
            Serial.println("Flag Stopping");
            delay(1000);
        break;

        case STATE_SWEEP:
            Serial.println("Sweep Starting");
            motorsBKWD(2000);
            esp_task_wdt_reset();
            motorsOFF(10);
            motorsROT_RIGHT(500);
            esp_task_wdt_reset();
            motorsOFF(10);
            motorsFWD(5000);
            esp_task_wdt_reset();
            motorsOFF(10);
            motorsBKWD(2000);
            esp_task_wdt_reset();
            motorsOFF(10);
            motorsOFF();
            Serial.println("Sweep Stopping");
        break;

        case STATE_BIN1:
            Serial.println("Bin Starting");
            motorsSTRAFE_RIGHT(2500);
            
            motorsOFF(10);
            motorsFWD(3200);
            motorsOFF(10);
            motorsSTRAFE_LEFT(2800);
            motorsOFF(10);
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
            GPIO.out_w1tc = COLLECTION_MASK;
            motorsOFF();
        break;
    }
}

#endif