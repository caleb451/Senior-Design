#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <DFRobot_BMI160.h>
#include "Motors/fastfunctions.h"


float diameter = 108; // 4.25in or 108mm

int rotate = 20;

static int32_t rotations[4] = {0};
static uint16_t lastRaw[4] = {0};
static bool firstRead[4] = {true};

static uint32_t lastPrint = 0;
const uint32_t printInterval = 100;

// TCA9548A I2C address (default)
#define TCA_ADDR 0x70

// BMI160 I2C address
const uint8_t bmi_addr = 0x68;
#define BMI160_CHANNEL 4

// Encoder objects (all default address 0x36)
AS5600 encoder1;
AS5600 encoder2;
AS5600 encoder3;
AS5600 encoder4;

AS5600* encoders[4] = {&encoder1, &encoder2, &encoder3, &encoder4};

// BMI160 object
DFRobot_BMI160 bmi160;

// Select TCA9548A channel
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delayMicroseconds(100);  // Settle time for multiplexer
}

void resetEncoder(uint8_t index) {
    rotations[index] = 0;
    firstRead[index] = true;
}

void tcaDisable() {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0);  // Disable all channels
    Wire.endTransmission();
    delayMicroseconds(100);
}

void i2cScanner() {
    Serial.println("\n=== I2C Scanner ===");
    
    // Scan with multiplexer disabled
    tcaDisable();
    delay(50);
    Serial.println("Main I2C Bus:");
    
    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  Found device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            
            // Identify known devices
            if (addr == 0x70) Serial.print(" (TCA9548A)");
            if (addr == 0x68) Serial.print(" (BMI160)");
            if (addr == 0x36) Serial.print(" (AS5600 - shouldn't be here!)");
            
            Serial.println();
            count++;
        }
    }
    if (count == 0) Serial.println("  No devices found");
    
    // Scan each multiplexer channel
    for (uint8_t ch = 0; ch < 8; ch++) {
        tcaSelect(ch);
        delay(10);
        
        Serial.print("Channel ");
        Serial.print(ch);
        Serial.print(": ");
        
        bool found = false;
        for (byte addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.print("0x");
                if (addr < 16) Serial.print("0");
                Serial.print(addr, HEX);
                Serial.print(" ");
                found = true;
            }
        }
        if (!found) Serial.print("(empty)");
        Serial.println();
    }
    
    tcaDisable();
    Serial.println("===================\n");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Robot Sensors Test ((4)AS5600 + BMI160) ===");

    Wire.begin();
    Wire.setClock(100000);  // 400 kHz
    Wire.setTimeOut(10000);

    i2cScanner();

    // Initialize all 4 encoders
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        Serial.print("Encoder ");
        Serial.print(i);
        Serial.print(" initialization ");

        delay(10);

        if (encoders[i]->begin() == 0){
            Serial.println("OK");
            lastRaw[i] = encoders[i] -> rawAngle();
        }
        else
            Serial.println("FAILED " + String(i));
    }
    delay(10);

    tcaSelect(BMI160_CHANNEL);
    delay(100);

    Serial.print("BMI160 initialization (channel ");
    Serial.print(BMI160_CHANNEL);
    Serial.print(") ");
    
    if (bmi160.I2cInit(bmi_addr) == BMI160_OK) {
        Serial.println("OK");
        delay(100);
    } else {
        Serial.println("FAILED");
    }

    tcaDisable();
    delay(50);

    Serial.println("\nReadings: \n");
    motorsInit();
}



void loop() {
    if (millis() - lastPrint >= printInterval) {
        lastPrint = millis();

        // Reading the 4 Encoders
        Serial.println("Encoders:");
        for (uint8_t i = 0; i < 4; i++) {
            tcaSelect(i);

            uint16_t raw = encoders[i] -> rawAngle();
            float deg = raw * (360.0 / 4096.0);

            // Rotation Detection
            if (!firstRead[i]) {
                int32_t delta = (int32_t)raw - (int32_t)lastRaw[i];
                
                // Detect crossing boundary
                if (delta > 2048) 
                    rotations[i]--; // (counter-clockwise)
                else if (delta < -2048) 
                    rotations[i]++; // (clockwise)
            }
            
            lastRaw[i] = raw;
            firstRead[i] = false;

            // Calculate Total Position
            float totalDegrees = (rotations[i] * 360.0) + deg;
            
            // Prints the values of the encoders
            Serial.print("  Encoder ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(deg, 1);
            Serial.print(" degree (");
            Serial.print(raw);
            Serial.print(") | Rotation: ");
            Serial.print(rotations[i]);
            Serial.print(" | Total: ");
            Serial.print(totalDegrees, 1);
            Serial.print(" degree");

            if (encoders[i]->detectMagnet())
                Serial.println("  [OK]");
            else
                Serial.println("  [UNSTABLE]");
        }
        Serial.println();

        // ───── Read BMI160 (channel 4) ─────
        tcaSelect(BMI160_CHANNEL);
        delay(5);

        // motorsFWD();

        // delay(5000);
        // motorsOFF();
        // delay(1000);

        // if((rotate >= rotations[0]) && (rotate >= rotations[1]) && (rotate >= rotations[2]) && (rotate >= rotations[3]))
        //     motorsFWD();
        // else
        //     motorsOFF();


    //───── Read BMI160 ─────
    int16_t data[6] = {0};
    int rslt = bmi160.getAccelGyroData(data);

    if (rslt == BMI160_OK) {
        Serial.println("BMI160:");
        Serial.print("  Gyro X/Y/Z (deg/s): ");
        Serial.print(data[0] / 16.4, 2); Serial.print("  ");
        Serial.print(data[1] / 16.4, 2); Serial.print("  ");
        Serial.print(data[2] / 16.4, 2); Serial.println();

        Serial.print("  Accel X/Y/Z (g):   ");
        Serial.print(data[3] / 16384.0, 3); Serial.print("  ");
        Serial.print(data[4] / 16384.0, 3); Serial.print("  ");
        Serial.print(data[5] / 16384.0, 3); Serial.println();
    } else {
        Serial.print("BMI160 read failed (error ");
        Serial.print(rslt);
        Serial.println(")");
    }

        Serial.println("----------------------------------------");
    }
}