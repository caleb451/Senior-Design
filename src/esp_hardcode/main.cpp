#include <Arduino.h>
#include "Motors/fastfunctions.h"
#include "Comp/reads.h"
#include "Comp/state.h"
#include <Wire.h>

// default speed for comp
static const uint8_t DRIVE_SPEED = 170;
static const bool SERVO_TEST_ONLY = true;

int val = 0;
bool START = false;
int state = 0;

void setup() {
    Serial.begin(115200);
    delay(100);

    armServo.write(0);
        delay(800);
        armServo.write(90);
        delay(800);
        armServo.write(180);
        delay(800);
        armServo.write(0);
        delay(800);

    doorServo.write(0);
        delay(800);
        doorServo.write(90);
        delay(800);
        doorServo.write(180);
        delay(800);
        doorServo.write(0);
        delay(800);
    motorsInit();
    motorsSetSpeed(DRIVE_SPEED);
    servosInit();
    collectionInit();
    motorsOFF();
    servoRotationTest();
}

void loop() {
    if (SERVO_TEST_ONLY) {
        do{
        armServo.write(0);
        delay(800);
        armServo.write(90);
        delay(800);
        armServo.write(180);
        delay(800);
        armServo.write(0);
        delay(800);
        }while(true);
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
    hardcode_state(state++, DRIVE_SPEED);
    delay(100);
}