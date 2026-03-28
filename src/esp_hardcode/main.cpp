#include <Arduino.h>
#include "Motors/fastfunctions.h"
#include "Comp/reads.h"
#include "Comp/state.h"
#include <Wire.h>

// default speed for comp
static const uint8_t DRIVE_SPEED = 160;

int val = 0;
bool START = false;
int state = 0;
bool servo_test = false;

void setup() {
    Serial.begin(115200);
    delay(100);

    motorsInit();
    motorsSetSpeed(DRIVE_SPEED);
    servosInit();
    collectionInit();
    motorsOFF();
    
}

void loop() {
    // if(servo_test){
    //     openDoor(100);
    //     delay(500);
    //     closeDoor(100);
    //     delay(1000);
    //     dropFlag(200);
    //     delay(500);
    //     liftBin(100);
    //     delay(1000);
    //     return;
    // }
    //Starting from beginning
    //Photoresistor start signal (will stay in the loop until photoresistor val reads light value above 1000)
    Serial.println("Photoresister read:");
    val = analogRead(photo);
    Serial.println(val);
    if(!START){
        if (val >= 1000){
            Serial.println("Photoresister read, light on:");
            Serial.println(val);
            START = true;
            digitalWrite(33, HIGH);
            delay(1000);
        }
        else{
            motorsOFF();
            Serial.println("Photoresister read not in bounds.");
            Serial.println("Photoresister read: ");
            Serial.print(val);
            return;
        }
    }

    //Current states run (flag, sweep, bin1, bin2, cave, cave_sweep)
    if(state < 6) {
        hardcode_state(state++, DRIVE_SPEED);
    } else {
        //lets this run again at the end for a new round
        START = false;
        state = 0;
        GPIO.out_w1tc = COLLECTION_MASK;  // Turn off collection after all states complete
    }
    delay(100);
}