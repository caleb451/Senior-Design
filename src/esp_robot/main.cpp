#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <DFRobot_BMI160.h>
#include "Motors/fastfunctions.h"

float diameter = 108; // 4.25in or 108mm
float wheelCircumference = PI * diameter; // mm

enum MovementMode {
    MOVE_FWD,
    MOVE_BACK,
    STRAFE_LEFT,
    STRAFE_RIGHT,
    ROT_LEFT,
    ROT_RIGHT
};
MovementMode currentMode = MOVE_FWD;

// Expected rotation direction for each wheel in each mode (+1 = forward, -1 = backward, 0 = not used)
const int8_t wheelDirection[6][4] = {
    // M1  M2  M3  M4
    {  1,  1,  1,  1 },  // MOVE_FWD
    { -1, -1, -1, -1 },  // MOVE_BACK
    { -1,  1,  1, -1 },  // STRAFE_LEFT
    {  1, -1, -1,  1 },  // STRAFE_RIGHT
    { -1,  1, -1,  1 },  // ROT_LEFT
    {  1, -1,  1, -1 }   // ROT_RIGHT
};

// Test state variables
bool testActive = false;
float targetRotations = 0.0;
uint32_t testStartTime = 0;
float initialRotations[4] = {0};

static int32_t rotations[4] = {0};
static uint16_t lastRaw[4] = {0};
static bool firstRead[4] = {true};

// Cached encoder data (updated frequently)
static float wheelRotations[4] = {0};   // total rotations including partial
static float wheelDegrees[4] = {0};     // current absolute angle (0-360)
static float wheelDistance[4] = {0};    // mm traveled
static bool  wheelMagnetOK[4] = {false};

// IMU-based movement verification
static float lastAccelMag = 1.0f;       // at rest = 1.0g
static float lastAccelX = 0.0f;         // acceleration X (g)
static float lastAccelY = 0.0f;         // acceleration Y (g)
static bool imuMovementDetected = false;
static float lastGyroX = 0.0f;          // rotation rate X (deg/s)
static float lastGyroY = 0.0f;          // rotation rate Y (deg/s)
static float lastGyroZ = 0.0f;          // rotation rate Z (deg/s)

static uint32_t lastEncoderUpdate = 0;
const uint32_t encoderUpdateInterval = 50; // update encoders every 50 ms

static uint32_t lastValuesPrint = 0;
const uint32_t valuesPrintInterval = 200; // Print values every 200ms

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

// ─────────────────────────────────────────────────
// Update encoder readings and cache computed values
// ─────────────────────────────────────────────────
void updateEncoders() {
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);

        uint16_t raw = encoders[i]->rawAngle();
        float deg = raw * (360.0f / 4096.0f);

        // Rotation Detection
        if (!firstRead[i]) {
            int32_t delta = (int32_t)raw - (int32_t)lastRaw[i];

            if (delta > 2048)
                rotations[i]--;
            else if (delta < -2048)
                rotations[i]++;
        }

        lastRaw[i] = raw;
        firstRead[i] = false;

        // Cache computed values
        wheelDegrees[i] = deg;
        // Invert encoder readings for M1 (wheel 0), M2 (wheel 1), and M3 (wheel 2) - mounted backward
        float actualRotations = rotations[i] + (deg / 360.0f);
        if (i == 0 || i == 1 || i == 2) {
            wheelRotations[i] = -actualRotations;
        } else {
            wheelRotations[i] = actualRotations;
        }
        wheelDistance[i] = wheelRotations[i] * wheelCircumference;
        wheelMagnetOK[i] = encoders[i]->detectMagnet();
    }

    // Also update IMU for movement detection
    tcaSelect(BMI160_CHANNEL);
    delay(2);
    int16_t data[6] = {0};
    int rslt = bmi160.getAccelGyroData(data);
    if (rslt == BMI160_OK) {
        // Gyroscope data (rotation rates in deg/s)
        lastGyroX = data[0] / 16.4f;
        lastGyroY = data[1] / 16.4f;
        lastGyroZ = data[2] / 16.4f;
        
        // Accelerometer data
        lastAccelX = data[3] / 16384.0f;
        lastAccelY = data[4] / 16384.0f;
        float accelZ = data[5] / 16384.0f;
        lastAccelMag = sqrt(lastAccelX*lastAccelX + lastAccelY*lastAccelY + accelZ*accelZ);
        // Movement detected if far from 1.0g (at rest) - looser threshold for ground vibration
        imuMovementDetected = (lastAccelMag > 1.02f || lastAccelMag < 0.95f);
    }

    tcaDisable();
}

inline void applyMovement(MovementMode mode) {
    switch (mode) {
        case MOVE_FWD:      motorsFWD();          break;
        case MOVE_BACK:     motorsBKWD();         break;
        case STRAFE_LEFT:   motorsSTRAFE_LEFT();  break;
        case STRAFE_RIGHT:  motorsSTRAFE_RIGHT(); break;
        case ROT_LEFT:      motorsROT_LEFT();     break;
        case ROT_RIGHT:     motorsROT_RIGHT();    break;
    }
}

void printValues() {
    // Teleplot format: >variableName:value
    
    // Wheel rotations (all on one graph)
    Serial.print(">wheels:");
    Serial.print(wheelRotations[0]);
    Serial.print(":");
    Serial.print(wheelRotations[1]);
    Serial.print(":");
    Serial.print(wheelRotations[2]);
    Serial.print(":");
    Serial.println(wheelRotations[3]);

    // Individual wheel rotations (easier to see per-channel)
    Serial.print(">wheel0:");
    Serial.println(wheelRotations[0]);
    Serial.print(">wheel1:");
    Serial.println(wheelRotations[1]);
    Serial.print(">wheel2:");
    Serial.println(wheelRotations[2]);
    Serial.print(">wheel3:");
    Serial.println(wheelRotations[3]);
    
    // Wheel distances in mm
    Serial.print(">distances:");
    Serial.print(wheelDistance[0]);
    Serial.print(":");
    Serial.print(wheelDistance[1]);
    Serial.print(":");
    Serial.print(wheelDistance[2]);
    Serial.print(":");
    Serial.println(wheelDistance[3]);
    
    // IMU Acceleration
    Serial.print(">accel_magnitude:");
    Serial.println(lastAccelMag);
    
    // IMU Acceleration X and Y
    Serial.print(">accel_x:");
    Serial.println(lastAccelX);
    Serial.print(">accel_y:");
    Serial.println(lastAccelY);
    
    // IMU Gyroscope Z only (rotation about vertical axis, deg/s)
    Serial.print(">gyro_z:");
    Serial.println(lastGyroZ);
    
    // Movement detection
    Serial.print(">imu_moving:");
    Serial.println(imuMovementDetected ? 1 : 0);
    
    // Test state
    if (testActive) {
        Serial.print(">test_target:");
        Serial.println(targetRotations);
        Serial.print(">test_mode:");
        Serial.println(currentMode);
    }
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

// ═══════════════════════════════════════════════════════════════
// TEST FUNCTIONS FOR MOVEMENT VERIFICATION
// ═══════════════════════════════════════════════════════════════

void startMovementTest(float numRotations, MovementMode mode) {
    testActive = true;
    targetRotations = numRotations;
    currentMode = mode;
    testStartTime = millis();

    // Re-zero encoders to start from 0 rotations
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        lastRaw[i] = encoders[i]->rawAngle();
        rotations[i] = 0;
        firstRead[i] = false;
        wheelRotations[i] = 0;
        wheelDistance[i] = 0;
        initialRotations[i] = 0;
    }
    tcaDisable();
    
    Serial.println("\n");
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║     ▶ MOVEMENT TEST STARTED ◀                  ║");
    Serial.print("║  Target: ");
    Serial.print(numRotations);
    Serial.println(" rotations per wheel            ║");
    Serial.print("║  Distance: ");
    Serial.print(numRotations * wheelCircumference, 0);
    Serial.println(" mm                          ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    applyMovement(mode);
}

void stopMovementTest() {
    // Capture final encoder state
    updateEncoders();

    motorsOFF();
    testActive = false;
    
    uint32_t testDuration = millis() - testStartTime;
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║     MOVEMENT TEST COMPLETED                    ║");
    Serial.print("║  Duration: ");
    Serial.print(testDuration);
    Serial.println(" ms                       ║");
    Serial.println("║  SUMMARY:                                      ║");
    
    for (uint8_t i = 0; i < 4; i++) {
        float actualRotations = wheelRotations[i] - initialRotations[i];
        float distance = actualRotations * wheelCircumference;
        
        Serial.print("║  Wheel ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(actualRotations, 2);
        Serial.print(" rot (");
        Serial.print(distance, 0);
        Serial.println(" mm)");
    }
    Serial.println("╚════════════════════════════════════════════════╝\n");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  Robot Movement & Sensor Verification Test     ║");
    Serial.println("║  (4x AS5600 Encoders + BMI160 IMU)             ║");
    Serial.println("╚════════════════════════════════════════════════╝");

    Wire.begin();
    Wire.setClock(100000);  // 100 kHz for stability
    Wire.setTimeOut(10000);

    i2cScanner();

    // Initialize all 4 encoders
    Serial.println("\n[ENCODER INITIALIZATION]");
    for (uint8_t i = 0; i < 4; i++) {
        tcaSelect(i);
        Serial.print("Encoder ");
        Serial.print(i);
        Serial.print(" initialization ");

        delay(10);

        if (encoders[i]->begin() == 0){
            Serial.println("✓ OK");
            lastRaw[i] = encoders[i] -> rawAngle();
        }
        else
            Serial.println("✗ FAILED");
    }

    // Prime cached encoder values
    updateEncoders();

    tcaSelect(BMI160_CHANNEL);
    delay(100);

    Serial.print("\n[BMI160 INITIALIZATION]");
    Serial.print("\nBMI160 on channel ");
    Serial.print(BMI160_CHANNEL);
    Serial.print(" ");
    
    if (bmi160.I2cInit(bmi_addr) == BMI160_OK) {
        Serial.println("✓ OK");
        delay(100);
    } else {
        Serial.println("✗ FAILED");
    }

    tcaDisable();
    delay(50);

    motorsInit();
    delay(100);
    motorsOFF();
    delay(100);

    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║          SYSTEM READY - COMMANDS:                 ║");
    Serial.println("║  M<num>  = Forward (legacy)                       ║");
    Serial.println("║  MF<num> = Forward rotations (e.g. MF5)           ║");
    Serial.println("║  MB<num> = Backward rotations                     ║");
    Serial.println("║  SL<num> = Strafe Left rotations                  ║");
    Serial.println("║  SR<num> = Strafe Right rotations                 ║");
    Serial.println("║  RL<num> = Rotate Left rotations                  ║");
    Serial.println("║  RR<num> = Rotate Right rotations                 ║");
    Serial.println("║  TEST    = Motor diagnostic (check wiring)        ║");
    Serial.println("║  STOP    = Stop test immediately                  ║");
    Serial.println("║  HELP    = Show all available commands            ║");
    Serial.println("╚════════════════════════════════════════════════════╝");
}





void loop() {
    // ─────────────────────────────────────────────────
    // Periodic encoder refresh (for accurate stopping)
    // ─────────────────────────────────────────────────
    if (millis() - lastEncoderUpdate >= encoderUpdateInterval) {
        lastEncoderUpdate = millis();
        updateEncoders();
    }

    // ─────────────────────────────────────────────────
    // HANDLE SERIAL COMMANDS
    // ─────────────────────────────────────────────────
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command.startsWith("MF")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) {
                startMovementTest(rotations_val, MOVE_FWD);
            } else {
                Serial.println("Invalid rotation value\n");
            }
        }
        else if (command.startsWith("MB")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) {
                startMovementTest(rotations_val, MOVE_BACK);
            } else {
                Serial.println("Invalid rotation value\n");
            }
        }
        else if (command.startsWith("M")) {
            // Legacy: M<num> defaults to forward
            float rotations_val = command.substring(1).toFloat();
            if (rotations_val > 0) {
                startMovementTest(rotations_val, MOVE_FWD);
            } else {
                Serial.println("Invalid rotation value. Use format: M5 (for 5 rotations)\n");
            }
        }
        else if (command.startsWith("SL")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) startMovementTest(rotations_val, STRAFE_LEFT);
        }
        else if (command.startsWith("SR")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) startMovementTest(rotations_val, STRAFE_RIGHT);
        }
        else if (command.startsWith("RL")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) startMovementTest(rotations_val, ROT_LEFT);
        }
        else if (command.startsWith("RR")) {
            float rotations_val = command.substring(2).toFloat();
            if (rotations_val > 0) startMovementTest(rotations_val, ROT_RIGHT);
        }
        else if (command == "STOP") {
            if (testActive) {
                stopMovementTest();
            } else {
                motorsOFF();
                Serial.println("No test active.\n");
            }
        }
        else if (command == "TEST") {
            Serial.println("\n=== MOTOR DIAGNOSTIC TEST ===");
            Serial.println("Testing each motor individually for 2 seconds...");
            Serial.println("Watch encoders - all should show POSITIVE rotation!\n");
            
            motorsOFF();
            updateEncoders();
            float testStart[4];
            for (int i=0; i<4; i++) testStart[i] = wheelRotations[i];
            delay(1000);
            
            Serial.println("M1 FWD (Wheel 0)");
            GPIO.out_w1ts = (1UL << M1_FWD);
            delay(2000);
            motorsOFF();
            updateEncoders();
            Serial.print("  Result: ");
            Serial.print(wheelRotations[0] - testStart[0], 2);
            Serial.println(wheelRotations[0] - testStart[0] > 0 ? " rot [OK]" : " rot [WIRED BACKWARD!]");
            for (int i=0; i<4; i++) testStart[i] = wheelRotations[i];
            delay(1000);
            
            Serial.println("M2 FWD (Wheel 1)");
            GPIO.out_w1ts = (1UL << M2_FWD);
            delay(2000);
            motorsOFF();
            updateEncoders();
            Serial.print("  Result: ");
            Serial.print(wheelRotations[1] - testStart[1], 2);
            Serial.println(wheelRotations[1] - testStart[1] > 0 ? " rot [OK]" : " rot [WIRED BACKWARD!]");
            for (int i=0; i<4; i++) testStart[i] = wheelRotations[i];
            delay(1000);
            
            Serial.println("M3 FWD (Wheel 2)");
            GPIO.out_w1ts = (1UL << M3_FWD);
            delay(2000);
            motorsOFF();
            updateEncoders();
            Serial.print("  Result: ");
            Serial.print(wheelRotations[2] - testStart[2], 2);
            Serial.println(wheelRotations[2] - testStart[2] > 0 ? " rot [OK]" : " rot [WIRED BACKWARD!]");
            for (int i=0; i<4; i++) testStart[i] = wheelRotations[i];
            delay(1000);
            
            Serial.println("M4 FWD (Wheel 3)");
            GPIO.out_w1ts = (1UL << M4_FWD);
            delay(2000);
            motorsOFF();
            updateEncoders();
            Serial.print("  Result: ");
            Serial.print(wheelRotations[3] - testStart[3], 2);
            Serial.println(wheelRotations[3] - testStart[3] > 0 ? " rot [OK]" : " rot [WIRED BACKWARD!]");
            
            Serial.println("\n=== TEST COMPLETE ===");
            Serial.println("Swap FWD/BKWD wires for any motor showing BACKWARD!\n");
        }
        else if (command == "TEST") {
            Serial.println("\n=== MOTOR DIAGNOSTIC TEST ===");
            Serial.println("Testing each motor individually for 2 seconds...");
            Serial.println("Expected: All wheels should spin FORWARD\n");
            
            motorsOFF();
            delay(1000);
            
            Serial.println("M1 FWD (Wheel 0) - Should rotate FORWARD");
            GPIO.out_w1ts = (1UL << M1_FWD);
            delay(2000);
            motorsOFF();
            delay(1000);
            
            Serial.println("M2 FWD (Wheel 1) - Should rotate FORWARD");
            GPIO.out_w1ts = (1UL << M2_FWD);
            delay(2000);
            motorsOFF();
            delay(1000);
            
            Serial.println("M3 FWD (Wheel 2) - Should rotate FORWARD");
            GPIO.out_w1ts = (1UL << M3_FWD);
            delay(2000);
            motorsOFF();
            delay(1000);
            
            Serial.println("M4 FWD (Wheel 3) - Should rotate FORWARD");
            GPIO.out_w1ts = (1UL << M4_FWD);
            delay(2000);
            motorsOFF();
            
            Serial.println("\n=== TEST COMPLETE ===");
            Serial.println("If any wheel rotated BACKWARD, that motor is wired incorrectly!");
            Serial.println("Swap FWD/BKWD wires for backward-rotating motors.\n");
        }
        else if (command == "HELP") {
            Serial.println("\n╔════════════════════════════════════════════════╗");
            Serial.println("║        AVAILABLE COMMANDS                      ║");
            Serial.println("╠════════════════════════════════════════════════╣");
            Serial.println("║ M<num>  - Forward (legacy)                    ║");
            Serial.println("║ MF<num> - Move Forward <num> rotations        ║");
            Serial.println("║ MB<num> - Move Backward <num> rotations       ║");
            Serial.println("║ SL<num> - Strafe Left <num> rotations         ║");
            Serial.println("║ SR<num> - Strafe Right <num> rotations        ║");
            Serial.println("║ RL<num> - Rotate Left <num> rotations         ║");
            Serial.println("║ RR<num> - Rotate Right <num> rotations        ║");
            Serial.println("║ TEST    - Run motor diagnostic test           ║");
            Serial.println("║ STOP    - Stop test immediately               ║");
            Serial.println("║ STATUS  - Show current test status            ║");
            Serial.println("║ HELP    - Show this help message              ║");
            Serial.println("╚════════════════════════════════════════════════╝\n");
        }
        else if (command == "STATUS") {
            Serial.println("\n[TEST STATUS]");
            if (testActive) {
                Serial.println("Status: RUNNING");
                Serial.print("Target: ");
                Serial.print(targetRotations);
                Serial.println(" rotations");
            } else {
                Serial.println("Status: IDLE");
            }
            Serial.println();
        }
        else if (command.length() > 0) {
            Serial.println("Unknown command. Type 'HELP' for available commands.\n");
        }
    }

    // ─────────────────────────────────────────────────
    // CHECK TEST COMPLETION
    // ─────────────────────────────────────────────────
    if (testActive) {
        static uint32_t movementStartTime = 0;
        
        // Check if all active wheels have reached target (magnitude-based to tolerate wiring inversions)
        bool encodersReached = true;
        for (uint8_t i = 0; i < 4; i++) {
            int8_t expectedDir = wheelDirection[currentMode][i];
            if (expectedDir == 0) continue; // Wheel not used in this mode
            
            float currentRotations = wheelRotations[i] - initialRotations[i];
            float progressMag = fabs(currentRotations);
            
            if (progressMag < targetRotations - 0.05f) {
                encodersReached = false;
                break;
            }
        }

        // Track when movement first detected for timeout
        if (movementStartTime == 0 && imuMovementDetected) {
            movementStartTime = millis();
        }
        
        bool enoughTimeElapsed = (movementStartTime > 0 && millis() - movementStartTime > 200);
        
        if (encodersReached && (imuMovementDetected || enoughTimeElapsed)) {
            stopMovementTest();
            movementStartTime = 0; // Reset for next test
        }
    }

    // Print sensor values periodically
    if (millis() - lastValuesPrint >= valuesPrintInterval) {
        lastValuesPrint = millis();
        printValues();
    }
}