#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "Motors/fastfunctions.h"

static uint8_t desiredSpeed = 200;

static float maxLinearMs = 0.5f;
static float maxStrafeMs = 0.5f;
static float maxAngularDegs = 180.0f;

static float velocityDeadbandMs = 0.01f;
static float strafeDeadbandMs = 0.01f;
static float angularDeadbandDegs = 1.0f;

static float halfLengthMm = 45.5f;
static float halfWidthMm = 75.0f;

const uint8_t BMI160_ADDR = 0x68;
const uint8_t GYRO_X_SRC = 0;
const uint8_t GYRO_Y_SRC = 1;
const uint8_t GYRO_Z_SRC = 2;
const int8_t GYRO_X_SIGN = 1;
const int8_t GYRO_Y_SIGN = 1;
const int8_t GYRO_Z_SIGN = 1;

const uint8_t ACC_X_SRC = 0;
const uint8_t ACC_Y_SRC = 1;
const uint8_t ACC_Z_SRC = 2;
const int8_t ACC_X_SIGN = 1;
const int8_t ACC_Y_SIGN = 1;
const int8_t ACC_Z_SIGN = 1;

const float FILTER_ALPHA = 0.2f;

DFRobot_BMI160 bmi160;
bool bmiOk = false;
bool filterInit = false;

float gyroXFilt = 0.0f;
float gyroYFilt = 0.0f;
float gyroZFilt = 0.0f;
float accelXFilt = 0.0f;
float accelYFilt = 0.0f;
float accelZFilt = 0.0f;

uint32_t lastImuPrint = 0;
const uint32_t imuPrintInterval = 100;
const uint32_t commandTimeoutMs = 300;

uint32_t lastCommandMs = 0;
bool hasActiveCommand = false;

float parseTaggedValue(const String &command, char tag) {
    int idx = command.indexOf(tag);
    if (idx < 0) return 0.0f;

    int start = idx + 1;
    while (start < command.length() && command.charAt(start) == ' ') start++;

    int end = start;
    while (end < command.length()) {
        char c = command.charAt(end);
        if (c == 'V' || c == 'Y' || c == 'W') break;
        end++;
    }

    String valueStr = command.substring(start, end);
    valueStr.trim();
    return valueStr.toFloat();
}

bool parseTaggedValueSafe(const String &command, char tag, float &outValue) {
    int idx = command.indexOf(tag);
    if (idx < 0) return false;

    int start = idx + 1;
    while (start < command.length() && command.charAt(start) == ' ') start++;
    if (start >= command.length()) return false;

    int end = start;
    while (end < command.length()) {
        char c = command.charAt(end);
        if (c == 'V' || c == 'Y' || c == 'W') break;
        end++;
    }

    String valueStr = command.substring(start, end);
    valueStr.trim();
    if (valueStr.length() == 0) return false;

    bool hasDigit = false;
    for (int i = 0; i < valueStr.length(); i++) {
        char c = valueStr.charAt(i);
        if (c >= '0' && c <= '9') {
            hasDigit = true;
            break;
        }
    }
    if (!hasDigit) return false;

    outValue = valueStr.toFloat();
    return true;
}

bool hasTag(const String &command, char tag) {
    return command.indexOf(tag) >= 0;
}

void applyVelocityCommand(float linearMs, float strafeMs, float angularDegs) {
    float absV = fabs(linearMs);
    float absS = fabs(strafeMs);
    float absW = fabs(angularDegs);

    if (absV < velocityDeadbandMs && absS < strafeDeadbandMs && absW < angularDeadbandDegs) {
        motorsOFF();
        return;
    }

    float xM = halfLengthMm / 1000.0f;
    float yM = halfWidthMm / 1000.0f;
    float omegaRad = angularDegs * (PI / 180.0f);

    float thetaM1 = atan2(-yM, xM);
    float thetaM2 = atan2(yM, xM);
    float thetaM3 = atan2(-yM, -xM);
    float thetaM4 = atan2(yM, -xM);

    float r = sqrt(xM * xM + yM * yM);

    float w1 = sin(thetaM1) * linearMs + cos(thetaM1) * strafeMs + r * omegaRad;
    float w2 = sin(thetaM2) * linearMs + cos(thetaM2) * strafeMs + r * omegaRad;
    float w3 = sin(thetaM3) * linearMs + cos(thetaM3) * strafeMs + r * omegaRad;
    float w4 = sin(thetaM4) * linearMs + cos(thetaM4) * strafeMs + r * omegaRad;

    float maxOmegaRad = maxAngularDegs * (PI / 180.0f);
    float maxWheel = maxLinearMs + maxStrafeMs + (r * maxOmegaRad);
    if (maxWheel < 0.001f) maxWheel = 0.001f;

    float speedScale = desiredSpeed / 255.0f;
    int pwmM1 = (int)(w1 / maxWheel * 255.0f * speedScale);
    int pwmM2 = (int)(w2 / maxWheel * 255.0f * speedScale);
    int pwmM3 = (int)(w3 / maxWheel * 255.0f * speedScale);
    int pwmM4 = (int)(w4 / maxWheel * 255.0f * speedScale);

    if (pwmM1 > 255) pwmM1 = 255;
    if (pwmM1 < -255) pwmM1 = -255;
    if (pwmM2 > 255) pwmM2 = 255;
    if (pwmM2 < -255) pwmM2 = -255;
    if (pwmM3 > 255) pwmM3 = 255;
    if (pwmM3 < -255) pwmM3 = -255;
    if (pwmM4 > 255) pwmM4 = 255;
    if (pwmM4 < -255) pwmM4 = -255;

    motorsSetWheelPwm(pwmM1, pwmM2, pwmM3, pwmM4);
}

void updateBmiTeleplot() {
    if (millis() - lastImuPrint < imuPrintInterval) return;
    lastImuPrint = millis();

    if (!bmiOk) return;

    int16_t data[6] = {0};
    int rslt = bmi160.getAccelGyroData(data);
    if (rslt != BMI160_OK) {
        Serial.println("BMI160 read error");
        return;
    }

    float accelRaw[3] = {
        data[0] / 16384.0f,
        data[1] / 16384.0f,
        data[2] / 16384.0f
    };
    float gyroRaw[3] = {
        data[3] / 16.4f,
        data[4] / 16.4f,
        data[5] / 16.4f
    };

    float gyroX = GYRO_X_SIGN * gyroRaw[GYRO_X_SRC];
    float gyroY = GYRO_Y_SIGN * gyroRaw[GYRO_Y_SRC];
    float gyroZ = GYRO_Z_SIGN * gyroRaw[GYRO_Z_SRC];
    float accelX = ACC_X_SIGN * accelRaw[ACC_X_SRC];
    float accelY = ACC_Y_SIGN * accelRaw[ACC_Y_SRC];
    float accelZ = ACC_Z_SIGN * accelRaw[ACC_Z_SRC];

    if (!filterInit) {
        gyroXFilt = gyroX;
        gyroYFilt = gyroY;
        gyroZFilt = gyroZ;
        accelXFilt = accelX;
        accelYFilt = accelY;
        accelZFilt = accelZ;
        filterInit = true;
    } else {
        gyroXFilt += FILTER_ALPHA * (gyroX - gyroXFilt);
        gyroYFilt += FILTER_ALPHA * (gyroY - gyroYFilt);
        gyroZFilt += FILTER_ALPHA * (gyroZ - gyroZFilt);
        accelXFilt += FILTER_ALPHA * (accelX - accelXFilt);
        accelYFilt += FILTER_ALPHA * (accelY - accelYFilt);
        accelZFilt += FILTER_ALPHA * (accelZ - accelZFilt);
    }

    Serial.print(">gyro_x:");
    Serial.println(gyroXFilt, 2);
    Serial.print(">gyro_y:");
    Serial.println(gyroYFilt, 2);
    Serial.print(">gyro_z:");
    Serial.println(gyroZFilt, 2);

    Serial.print(">accel_x:");
    Serial.println(accelXFilt, 3);
    Serial.print(">accel_y:");
    Serial.println(accelYFilt, 3);
    Serial.print(">accel_z:");
    Serial.println(accelZFilt, 3);
}

void printUsage() {
    Serial.println("Use only: V<vx> Y<vy> W<w>");
    Serial.println("Example: V0.20 Y0.00 W30");
    Serial.println("Send V0 Y0 W0 to stop");
}

void handleCommand(String command) {
    command.trim();
    command.toUpperCase();

    if (!hasTag(command, 'V') || !hasTag(command, 'Y') || !hasTag(command, 'W')) {
        Serial.println("Invalid command. Use V<vx> Y<vy> W<w>");
        return;
    }

    float vMs = 0.0f;
    float yMs = 0.0f;
    float wDeg = 0.0f;

    bool okV = parseTaggedValueSafe(command, 'V', vMs);
    bool okY = parseTaggedValueSafe(command, 'Y', yMs);
    bool okW = parseTaggedValueSafe(command, 'W', wDeg);
    if (!okV || !okY || !okW) {
        Serial.println("Invalid values. Use V<vx> Y<vy> W<w>");
        return;
    }

    applyVelocityCommand(vMs, yMs, wDeg);
    lastCommandMs = millis();
    hasActiveCommand = true;
}

void setup() {
    Serial.begin(115200);
    delay(200);

    Wire.begin();
    Wire.setClock(100000);
    Wire.setTimeOut(10000);

    if (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK) {
        bmiOk = true;
        Serial.println("BMI160: OK");
    } else {
        bmiOk = false;
        Serial.println("BMI160: INIT FAILED");
    }

    motorsInit();
    delay(100);
    motorsSetSpeed(desiredSpeed);
    motorsOFF();

    printUsage();
}

void loop() {
    updateBmiTeleplot();

    static String commandBuffer;
    static uint32_t lastCharMs = 0;

    if (Serial.available()) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            lastCharMs = millis();
            if (c == '\n' || c == '\r' || c == ';') {
                if (commandBuffer.length() > 0) {
                    handleCommand(commandBuffer);
                    commandBuffer = "";
                }
            } else {
                commandBuffer += c;
                if (commandBuffer.length() > 64) {
                    commandBuffer = "";
                }
            }
        }
    }

    if (commandBuffer.length() > 0 && (millis() - lastCharMs) > 50) {
        handleCommand(commandBuffer);
        commandBuffer = "";
    }

    if (hasActiveCommand && (millis() - lastCommandMs) > commandTimeoutMs) {
        motorsOFF();
        hasActiveCommand = false;
    }
}
