#include <Arduino.h>
#include "Motors/fastfunctions.h"
#include "Comp/reads.h"
#include "Comp/state.h"
#include <Wire.h>

// default speed for comp
static const uint8_t DRIVE_SPEED = 170;

int val = 0;
bool START = true;
int state;

void setup() {
    Serial.begin(115200);
    delay(100);

    motorsInit();
    motorsSetSpeed(DRIVE_SPEED);
    collectionInit();
    motorsOFF();
}

void loop() {
    //Starting from beginning
    //Photoresistor start signal (will stay in the loop until photoresistor is read)
    val = analogRead(photo);
    if(!START){
        if (val > 1000){
            serialPrint("Photoresister read");
            START = true;
        }
        else{
            motorsOFF();
            return;
        }
    }

    //Current states run (flag, sweep, bin1, bin2, cave, cave_sweep)
    hardcode_state(state, DRIVE_SPEED);
    delay(100);
}
