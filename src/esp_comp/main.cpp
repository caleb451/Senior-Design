#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "Motors/fastfunctions.h"
#include <Adafruit_VL53L0X.h>

// ==================== CONFIGURATION ====================
#define FIELD_LENGTH_MM 2438  // 8 feet in mm
#define FIELD_WIDTH_MM 1219   // 4 feet in mm
#define ROBOT_LENGTH_MM 91    // Robot length
#define ROBOT_WIDTH_MM 150    // Robot width
#define CELL_SIZE_MM 300      // Grid cell size (~12 inches)

// ==================== STATE MACHINE ====================
enum RobotState {
  STATE_INIT,
  STATE_NAVIGATE,
  STATE_COLLECT,
  STATE_RETURN_HOME,
  STATE_IDLE
};

// ==================== SENSOR OBJECTS ====================
DFRobot_BMI160 bmi160;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool bmiOk = false;
bool loxOk = false;

// ==================== POSITION AND ORIENTATION ====================
struct Position {
  float x;      // mm
  float y;      // mm
  float heading; // degrees (0-360)
};

Position robotPos;
Position homePos;
Position targetPos;

// ==================== IMU DATA ====================
float gyroZFilt = 0.0f;
float accelXFilt = 0.0f;
float accelYFilt = 0.0f;
const float FILTER_ALPHA = 0.15f;

// ==================== MOTOR CONTROL ====================
const uint8_t DEFAULT_MOTOR_SPEED = 180;
const uint32_t commandTimeoutMs = 300;
uint32_t lastMotorCommand = 0;

// ==================== NAVIGATION ====================
RobotState currentState = STATE_INIT;
uint32_t stateStartTime = 0;
uint8_t gridRow = 0;
uint8_t gridCol = 0;
const uint8_t GRID_ROWS = (FIELD_LENGTH_MM / CELL_SIZE_MM);
const uint8_t GRID_COLS = (FIELD_WIDTH_MM / CELL_SIZE_MM);

// ==================== MATERIAL COLLECTION ====================
uint32_t collectionTimer = 0;
uint8_t itemCount = 0;
const uint16_t COLLECTION_DURATION_MS = 500; // Collect for 500ms at each position

// ==================== TOF SENSOR READINGS ====================
uint16_t leftDistance = 0;
uint16_t rightDistance = 0;
const uint16_t WALL_THRESHOLD_MM = 600; // Adjust based on field walls

// ==================== TIMING ====================
uint32_t lastStatusPrint = 0;
const uint32_t statusPrintInterval = 500;
uint32_t lastImuRead = 0;
const uint32_t imuReadInterval = 20;
uint32_t lastTofRead = 0;
const uint32_t tofReadInterval = 100;

// ==================== FUNCTION PROTOTYPES ====================
void initializeSensors();
void updateIMU();
void updatePositionFromIMU();
void updateToFSensors();
void activateCollectionMechanism(uint16_t durationMs);
void navigateGrid();
void moveToPosition(Position target);
void rotateToHeading(float targetHeading);
void moveForward(uint16_t distanceMm, uint8_t speed);
void adjustPositionBasedOnWalls();
void printStatus();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ROBOT COMPETITION PROGRAM ===");
  Serial.println("Field Size: 8' x 4' (2438mm x 1219mm)");
  
  // Initialize motor PWM
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
  ledcSetup(5, 20000, 8);
  ledcSetup(6, 20000, 8);
  ledcSetup(7, 20000, 8);
  
  ledcAttachPin(18, 0);
  ledcAttachPin(19, 1);
  ledcAttachPin(16, 2);
  ledcAttachPin(17, 3);
  ledcAttachPin(27, 4);
  ledcAttachPin(26, 5);
  ledcAttachPin(13, 6);
  ledcAttachPin(12, 7);
  
  motorsSetSpeed(DEFAULT_MOTOR_SPEED);
  motorsOFF();
  
  // Initialize sensors
  initializeSensors();
  
  // Set initial state and position
  currentState = STATE_NAVIGATE;
  stateStartTime = millis();
  robotPos.x = 100.0f;
  robotPos.y = 100.0f;
  robotPos.heading = 0.0f;
  homePos.x = 100.0f;
  homePos.y = 100.0f;
  homePos.heading = 0.0f;
  
  Serial.println("Ready to begin navigation...");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t currentTime = millis();
  
  // Update IMU data
  if (currentTime - lastImuRead >= imuReadInterval) {
    updateIMU();
    updatePositionFromIMU();
    lastImuRead = currentTime;
  }
  
  // Update ToF sensors (left and right distances)
  if (currentTime - lastTofRead >= tofReadInterval) {
    updateToFSensors();
    lastTofRead = currentTime;
  }
  
  // State machine
  switch (currentState) {
    case STATE_INIT:
      initializeSensors();
      currentState = STATE_NAVIGATE;
      break;
      
    case STATE_NAVIGATE:
      // ALWAYS move forward and collect at intervals
      moveForward(100, DEFAULT_MOTOR_SPEED);  // Keep moving
      navigateGrid();  // Handle collection timing
      adjustPositionBasedOnWalls();
      break;
      
    case STATE_COLLECT:
      // Keep moving, collection is already active
      moveForward(100, DEFAULT_MOTOR_SPEED);
      if (millis() - collectionTimer >= COLLECTION_DURATION_MS) {
        currentState = STATE_NAVIGATE;
      }
      break;
      
    case STATE_RETURN_HOME:
      motorsOFF();
      currentState = STATE_IDLE;
      break;
      
    case STATE_IDLE:
      motorsOFF();
      break;
  }
  
  // Print status periodically
  if (currentTime - lastStatusPrint >= statusPrintInterval) {
    printStatus();
    lastStatusPrint = currentTime;
  }
}

// ==================== SENSOR INITIALIZATION ====================
void initializeSensors() {
  Serial.println("Initializing sensors...");
  
  // Initialize BMI160 IMU
  delay(200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeOut(10000);
  
  if (bmi160.I2cInit(0x68) == BMI160_OK) {
    Serial.println("BMI160 OK");
    bmiOk = true;
  } else {
    Serial.println("BMI160 FAIL");
    bmiOk = false;
  }
  
  // Initialize VL53L0X ToF sensor - try multiple times
  loxOk = false;
  for (int i = 0; i < 3; i++) {
    if (lox.begin()) {
      Serial.println("ToF OK");
      loxOk = true;
      break;
    }
    delay(100);
  }
  if (!loxOk) {
    Serial.println("ToF FAIL");
  }
}

// ==================== IMU UPDATE ====================
void updateIMU() {
  if (!bmiOk) return;
  
  int16_t data[6] = {0}; // accelX, accelY, accelZ, gyroX, gyroY, gyroZ
  int rslt = bmi160.getAccelGyroData(data);
  
  if (rslt != BMI160_OK) {
    return;
  }
  
  int16_t gyroZ = data[5];
  
  // Apply low-pass filter to gyro Z (heading change)
  gyroZFilt = (gyroZ * FILTER_ALPHA) + (gyroZFilt * (1.0f - FILTER_ALPHA));
}

// ==================== POSITION UPDATE ====================
void updatePositionFromIMU() {
  // This is a simplified position update
  // In practice, you'd want to integrate acceleration over time
  // and use gyro for heading changes
  
  // Update heading from gyro Z (rotation around vertical axis)
  robotPos.heading += (gyroZFilt / 131.0) * 0.02; // Convert to degrees, 20ms interval
  
  // Wrap heading to 0-360
  while (robotPos.heading < 0) robotPos.heading += 360;
  while (robotPos.heading >= 360) robotPos.heading -= 360;
}

// ==================== TOF SENSOR UPDATE ====================
void updateToFSensors() {
  if (!loxOk) return;
  
  VL53L0X_RangingMeasurementData_t measure;
  
  // Read left sensor (you may need to adjust based on actual sensor configuration)
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    leftDistance = measure.RangeMilliMeter;
  }
  
  // Read right sensor (if using i2c multiplexer or second sensor)
  // For now, simplified to single sensor reading
  rightDistance = leftDistance; // Placeholder - update if you have dual sensors
  
  Serial.print("ToF - Left: ");
  Serial.print(leftDistance);
  Serial.print("mm, Right: ");
  Serial.print(rightDistance);
  Serial.println("mm");
}

// ==================== WALL-BASED POSITION ADJUSTMENT ====================
void adjustPositionBasedOnWalls() {
  // Use ToF sensors to detect walls and correct robot position
  // This helps with navigation accuracy on the field
  
  if (leftDistance < WALL_THRESHOLD_MM && leftDistance > 50) {
    // Close to left wall - can use as reference
    Serial.println("Left wall detected");
  }
  
  if (rightDistance < WALL_THRESHOLD_MM && rightDistance > 50) {
    // Close to right wall - can use as reference
    Serial.println("Right wall detected");
  }
}

// ==================== COLLECTION MECHANISM ====================
void activateCollectionMechanism(uint16_t durationMs) {
  // Collection mechanism is always on (sucks automatically)
  // Just track that we're collecting
  Serial.print("Collecting... ");
  Serial.print(durationMs);
  Serial.println("ms");
  
  collectionTimer = millis();
  currentState = STATE_COLLECT;
  itemCount++;
}

// ==================== GRID NAVIGATION ====================
void navigateGrid() {
  static uint32_t lastCollectTime = 0;
  uint32_t currentTime = millis();
  
  // Collect every 2 seconds
  if (currentTime - lastCollectTime > 2000) {
    lastCollectTime = currentTime;
    
    Serial.print("@");
    Serial.print((int)itemCount);
    
    // Start collection
    activateCollectionMechanism(COLLECTION_DURATION_MS);
  }
}

// ==================== MOVEMENT FUNCTIONS ====================
void moveToPosition(Position target) {
  // Non-blocking - just keep moving forward while collecting
  motorsSetSpeed(DEFAULT_MOTOR_SPEED);
  motorsFWD();
}

void rotateToHeading(float targetHeading) {
  float angleDiff = targetHeading - robotPos.heading;
  
  // Normalize angle difference to -180 to 180
  while (angleDiff > 180) angleDiff -= 360;
  while (angleDiff < -180) angleDiff += 360;
  
  if (fabsf(angleDiff) < 5) return; // Already facing correct direction
  
  if (angleDiff > 0) {
    motorsROT_LEFT();
  } else {
    motorsROT_RIGHT();
  }
}

void moveForward(uint16_t distanceMm, uint8_t speed) {
  motorsSetSpeed(speed);
  motorsFWD();
  delay(50);  // Brief movement window
}

// ==================== STATUS PRINTING ====================
void printStatus() {
  Serial.print("HeadingZ:");
  Serial.print((int)robotPos.heading);
  Serial.print(" ToF_L:");
  Serial.print(leftDistance);
  Serial.print(" ToF_R:");
  Serial.print(rightDistance);
  Serial.print(" Items:");
  Serial.print((int)itemCount);
  Serial.print(" State:");
  
  switch (currentState) {
    case STATE_INIT: Serial.print("INIT"); break;
    case STATE_NAVIGATE: Serial.print("NAV"); break;
    case STATE_COLLECT: Serial.print("COL"); break;
    case STATE_RETURN_HOME: Serial.print("HOME"); break;
    case STATE_IDLE: Serial.print("IDLE"); break;
  }
  Serial.println();
}
