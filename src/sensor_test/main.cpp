#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <Adafruit_VL53L0X.h>

// BMI160 on main I2C bus (no mux)
const uint8_t BMI160_ADDR = 0x68;
const uint8_t TCA_ADDR = 0x70;
const uint8_t TOF_ADDR = 0x29;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_RAW_ANGLE_HI = 0x0C;
const uint8_t encoderChannels[] = {0, 1, 2, 3};
const uint8_t encoderCount = sizeof(encoderChannels) / sizeof(encoderChannels[0]);
const uint8_t TOF_LEFT_CH = 4;
const uint8_t TOF_RIGHT_CH = 5;
const bool USE_FIXED_CHANNELS = true;

// Axis mapping (adjust if axes are flipped)
const uint8_t GYRO_X_SRC = 0; // 0=X, 1=Y, 2=Z
const uint8_t GYRO_Y_SRC = 1;
const uint8_t GYRO_Z_SRC = 2;
const int8_t GYRO_X_SIGN = 1;
const int8_t GYRO_Y_SIGN = 1;
const int8_t GYRO_Z_SIGN = 1;

const uint8_t ACC_X_SRC = 0; // 0=X, 1=Y, 2=Z
const uint8_t ACC_Y_SRC = 1;
const uint8_t ACC_Z_SRC = 2;
const int8_t ACC_X_SIGN = 1;
const int8_t ACC_Y_SIGN = 1;
const int8_t ACC_Z_SIGN = 1;

const float FILTER_ALPHA = 0.2f; // lower = smoother

DFRobot_BMI160 bmi160;
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofRight;
bool bmiOk = false;
bool filterInit = false;
bool muxOk = false;
bool tofLeftOk = false;
bool tofRightOk = false;
int8_t tofLeftCh = -1;
int8_t tofRightCh = -1;

uint16_t encoderLastRaw[4] = {0};
int32_t encoderTurns[4] = {0};
bool encoderFirstRead[4] = {true, true, true, true};

float gyroXFilt = 0.0f;
float gyroYFilt = 0.0f;
float gyroZFilt = 0.0f;
float accelXFilt = 0.0f;
float accelYFilt = 0.0f;
float accelZFilt = 0.0f;

uint32_t lastPrint = 0;
const uint32_t printInterval = 100;
uint32_t lastGoodReadMs = 0;
uint32_t lastReinitMs = 0;
const uint32_t reinitCooldownMs = 1000;
const uint32_t staleReadTimeoutMs = 2000;
uint8_t consecutiveReadErrors = 0;

bool readAs5600Raw(uint16_t &rawAngle) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_HI);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    if (received != 2) return false;

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    rawAngle = ((uint16_t)hi << 8) | lo;
    rawAngle &= 0x0FFF;
    return true;
}

float rawToDegrees(uint16_t rawAngle) {
    return (360.0f * rawAngle) / 4096.0f;
}

float rawToRotations(uint8_t encoderIndex, uint16_t rawAngle) {
    if (encoderFirstRead[encoderIndex]) {
        encoderLastRaw[encoderIndex] = rawAngle;
        encoderFirstRead[encoderIndex] = false;
    } else {
        int32_t delta = (int32_t)rawAngle - (int32_t)encoderLastRaw[encoderIndex];
        if (delta > 2048) {
            encoderTurns[encoderIndex]--;
        } else if (delta < -2048) {
            encoderTurns[encoderIndex]++;
        }
        encoderLastRaw[encoderIndex] = rawAngle;
    }

    return (float)encoderTurns[encoderIndex] + ((float)rawAngle / 4096.0f);
}

bool muxSelect(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    return Wire.endTransmission() == 0;
}

bool muxDisableAll() {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    return Wire.endTransmission() == 0;
}

bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

int8_t findTofOnMux(int8_t excludeCh = -1) {
    for (uint8_t ch = 0; ch < 8; ++ch) {
        if (static_cast<int8_t>(ch) == excludeCh) continue;
        if (!muxSelect(ch)) continue;
        delay(2);
        if (i2cPing(TOF_ADDR)) return static_cast<int8_t>(ch);
    }
    return -1;
}

void initTofSensors() {
    tofLeftOk = false;
    tofRightOk = false;
    tofLeftCh = -1;
    tofRightCh = -1;

    muxOk = i2cPing(TCA_ADDR);
    if (!muxOk) {
        Serial.println("TCA9548A: NOT FOUND");
        return;
    }

    if (USE_FIXED_CHANNELS) {
        tofLeftCh = TOF_LEFT_CH;
        tofRightCh = TOF_RIGHT_CH;
    } else {
        tofLeftCh = findTofOnMux();
        tofRightCh = findTofOnMux(tofLeftCh);
    }

    if (tofLeftCh >= 0 && muxSelect(static_cast<uint8_t>(tofLeftCh))) {
        delay(2);
        tofLeftOk = tofLeft.begin(TOF_ADDR, &Wire);
    }

    if (tofRightCh >= 0 && muxSelect(static_cast<uint8_t>(tofRightCh))) {
        delay(2);
        tofRightOk = tofRight.begin(TOF_ADDR, &Wire);
    }

    muxDisableAll();

    Serial.print("ToF Left ch: ");
    Serial.println(tofLeftCh);
    Serial.println(tofLeftOk ? "ToF Left: OK" : "ToF Left: INIT FAILED");
    Serial.print("ToF Right ch: ");
    Serial.println(tofRightCh);
    Serial.println(tofRightOk ? "ToF Right: OK" : "ToF Right: INIT FAILED");
}

void initI2cBus() {
    Wire.end();
    delay(2);
    Wire.begin();
    Wire.setClock(100000);
    Wire.setTimeOut(20);
}

bool initBmi160() {
    if (bmi160.I2cInit(BMI160_ADDR) == BMI160_OK) {
        filterInit = false;
        consecutiveReadErrors = 0;
        lastGoodReadMs = millis();
        bmiOk = true;
        return true;
    }

    bmiOk = false;
    return false;
}

void tryReinit(const char *reason) {
    uint32_t now = millis();
    if ((now - lastReinitMs) < reinitCooldownMs) return;
    lastReinitMs = now;

    Serial.print("i2c_reinit_reason:");
    Serial.println(reason);

    initI2cBus();
    bool ok = initBmi160();
    initTofSensors();

    Serial.print("i2c_reinit_ok:");
    Serial.println(ok ? 1 : 0);
}

void setup() {
    Serial.begin(115200);
    delay(200);

    initI2cBus();

    Serial.println("\n=== BMI160 TEST MODE ===");

    if (initBmi160()) {
        Serial.println("BMI160: OK");
    } else {
        Serial.println("BMI160: INIT FAILED");
    }

    initTofSensors();

    if (muxOk) {
        for (uint8_t i = 0; i < encoderCount; i++) {
            uint8_t channel = encoderChannels[i];
            if (muxSelect(channel)) {
                delay(2);
                bool found = i2cPing(AS5600_ADDR);
                Serial.print("ENC");
                Serial.print(i);
                Serial.print(" (ch");
                Serial.print(channel);
                Serial.print("): ");
                Serial.println(found ? "FOUND" : "NOT FOUND");

                if (found) {
                    uint16_t raw = 0;
                    if (readAs5600Raw(raw)) {
                        encoderLastRaw[i] = raw;
                        encoderTurns[i] = 0;
                        encoderFirstRead[i] = false;
                    }
                }
            }
        }
        muxDisableAll();
    }
}

void loop() {
    if (millis() - lastPrint < printInterval) return;
    lastPrint = millis();

    if (bmiOk && (millis() - lastGoodReadMs) > staleReadTimeoutMs) {
        tryReinit("stale_timeout");
    }

    // BMI160 data (Teleplot)
    if (bmiOk) {
        int16_t data[6] = {0};
        int rslt = bmi160.getAccelGyroData(data);
        if (rslt == BMI160_OK) {
            lastGoodReadMs = millis();
            consecutiveReadErrors = 0;

            // DFRobot_BMI160 data order can be accel/gyro swapped on some setups.
            // If accel looks like gyro and gyro looks like accel, swap interpretation.
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

            Serial.print("bmi_ok:");
            Serial.println(1);
        } else {
            Serial.println("BMI160 read error");
            consecutiveReadErrors++;
            Serial.print("bmi_ok:");
            Serial.println(0);

            if (consecutiveReadErrors >= 3) {
                tryReinit("consecutive_errors");
            }
        }
    } else {
        Serial.print("bmi_ok:");
        Serial.println(0);
        tryReinit("not_initialized");
    }

    VL53L0X_RangingMeasurementData_t measure;

    if (tofLeftOk && tofLeftCh >= 0 && muxSelect(static_cast<uint8_t>(tofLeftCh))) {
        delay(2);
        tofLeft.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            Serial.print(">tof_left_mm:");
            Serial.println(measure.RangeMilliMeter);
        }
    }

    if (tofRightOk && tofRightCh >= 0 && muxSelect(static_cast<uint8_t>(tofRightCh))) {
        delay(2);
        tofRight.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            Serial.print(">tof_right_mm:");
            Serial.println(measure.RangeMilliMeter);
        }
    }

    for (uint8_t i = 0; i < encoderCount; i++) {
        bool readOk = false;
        uint16_t encRaw = 0;
        uint8_t channel = encoderChannels[i];

        if (muxOk && muxSelect(channel)) {
            delay(2);
            if (readAs5600Raw(encRaw)) {
                readOk = true;
            }
        }

        Serial.print(">enc");
        Serial.print(i);
        Serial.print("_raw:");
        if (readOk) {
            Serial.println(encRaw);
        } else {
            Serial.println(-1);
        }

        Serial.print(">enc");
        Serial.print(i);
        Serial.print("_deg:");
        if (readOk) {
            Serial.println(rawToDegrees(encRaw), 2);
        } else {
            Serial.println(-1);
        }

        Serial.print(">enc");
        Serial.print(i);
        Serial.print("_rot:");
        if (readOk) {
            Serial.println(rawToRotations(i, encRaw), 4);
        } else {
            Serial.println(-1);
        }
    }

    if (muxOk) {
        muxDisableAll();
    }
}
