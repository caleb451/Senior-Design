#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Motors/fastfunctions.h"
#include "Comp/reads.h"
#include "Comp/state.h"
#include <Wire.h>

// default speed for comp
static const uint8_t DRIVE_SPEED = 170;

int val = 0;
bool START = false;
int state = 0;
bool servo_test = true;

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
    if(servo_test){
        openDoor(100);
        esp_task_wdt_reset();
        delay(500);
        closeDoor(100);
        esp_task_wdt_reset();
        delay(1000);
        dropFlag(200);
        esp_task_wdt_reset();
        delay(500);
        liftBin(100);
        esp_task_wdt_reset();
        delay(1000);
        return;
    }
    //Starting from beginning
    //Photoresistor start signal (will stay in the loop until photoresistor val reads light value above 1000)
    Serial.println("Photoresister read:");
    val = analogRead(photo);
    Serial.println(val);
    if(!START){
        if (val >= 1000){
            GPIO.out_w1ts = (1UL << COLLECTION);
            Serial.println("Photoresister read, light on:");
            Serial.println(val);
            START = true;
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
    esp_task_wdt_reset();
    hardcode_state(state++, DRIVE_SPEED);
    esp_task_wdt_reset();
    delay(100);
}