// This is the file where the robot will be programmed for its autonomy. The Pi5 for the original computer vision
// vision program broke so everything will be programmed through the esp-32. The esp32 will have encoders to verify
// any sliping movements from the wheels and help with localization so that the robot can move to different points on
// the field. 

// To save time on executing lines of code, the program will be directly calling the GPIO pins from the esp32 allowing for faster speeds

#include <Arduino.h>
#include "Motors/fastfunctions.h"
#include <Wire.h>
#include <AS5600.h>

AS5600 as5600;

// Rotation tracking
int32_t totalRotations = 0;        // Total full revolutions (positive = CW, negative = CCW)
float previousAngleDeg = 0.0;      // Previous reading for wrap detection
bool firstReading = true;

void setup(){
    Serial.begin(115200);
    Wire.begin();

    Serial.println("AS5600 Test");
    if (as5600.isConnected()) {
        Serial.println("AS5600 connected!");
    } else {
        Serial.println("AS5600 not found!");
    }

    //motorsInit();
}

void loop(){
    static uint32_t lastPrint = 0;
    const uint32_t printInterval = 100;  // Print every 100ms
    

    if (millis() - lastPrint >= printInterval) {
        lastPrint = millis();

        // Read raw angle and convert to degrees
        uint16_t rawVal = as5600.rawAngle();
        float currentAngleDeg = rawVal * AS5600_RAW_TO_DEGREES;  // 0.0 to 359.91...

        // === Rotation Counting Logic ===
        if (!firstReading) {
            float delta = currentAngleDeg - previousAngleDeg;

            // Detect wrap-around
            if (delta > 180.0) {              // Crossed from ~359° to ~0° (clockwise wrap)
                delta -= 360.0;
                totalRotations--;
            } else if (delta < -180.0) {      // Crossed from ~0° to ~359° (counter-clockwise wrap)
                delta += 360.0;
                totalRotations++;
            }

            // Optional: Use delta for velocity calculation later
        } else {
            firstReading = false;  // First valid reading, just store it
        }

        previousAngleDeg = currentAngleDeg;

        // === Serial Output ===
        Serial.print("Raw: ");
        Serial.print(rawVal);
        Serial.print(" | Angle: ");
        Serial.print(currentAngleDeg, 2);
        Serial.print("° | Total Rotations: ");
        Serial.print(totalRotations);

        // Magnet status
        if (as5600.detectMagnet()) {
            Serial.println(" | Magnet: OK");
        } else {
            Serial.println(" | Magnet: ISSUE");
        }
    }

    // === Example Motor Test Sequence ===
    // Remove or comment this out once you're done testing!
    // static uint32_t motorTimer = 0;
    // uint32_t now = millis();

    // if (now - motorTimer < 2000) {
    //     motorsFWD();     // Forward for 2 seconds
    // } else if (now - motorTimer < 3000) {
    //     motorsOFF();         // Off for 1 second
    // } else if (now - motorTimer < 5000) {
    //     motorsBKWD();    // Backward for 2 seconds
    // } else if (now - motorTimer < 6000) {
    //     motorsOFF();         // Off for 1 second
    // } else {
    //     motorTimer = now;    // Reset cycle
    // }

    // delay(100);  // Adjust for your loop rate
    // motorsFWD();
    // delay(1000);
    // motorsOFF();
    // delay(500);
    // motorsBKWD();
    // delay(1000);
}