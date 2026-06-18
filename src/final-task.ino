#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>

/*
 * Robot I2C Sensor Demonstration Code - No LED Version
 *
 * 功能：
 * 1. ADS1115 读取三个 IR 红外传感器
 * 2. BME280 读取温度、湿度、气压
 * 3. MPU-9250 通过原始寄存器读取加速度计和陀螺仪
 * 4. 快速向上抬起小车 -> 轮子开始转动
 * 5. 小车停在空中不动 -> 轮子继续转动
 * 6. 一旦检测到小车开始向下 -> 轮子停止
 * 7. 向下停住后的反弹不会重新触发轮子转动
 * 8. 没有使用任何 LED，引脚 D3 和 D11 空出
 *
 * UNO / Nano:
 * A4 = SDA
 * A5 = SCL
 *
 * L298N:
 * ENA -> D5
 * ENB -> D6
 * IN1 -> D4
 * IN2 -> D7
 * IN3 -> D9
 * IN4 -> D10
 *
 * ADS1115:
 * A0 -> Left IR
 * A1 -> Center IR
 * A2 -> Right IR
 */

/* ===================== L298N 电机接口 ===================== */

const int enablePinRight = 5;   // D5  -> L298N ENA，右电机 PWM
const int enablePinLeft  = 6;   // D6  -> L298N ENB，左电机 PWM

const int in1Pin = 4;           // D4  -> L298N IN1，右电机方向
const int in2Pin = 7;           // D7  -> L298N IN2，右电机方向
const int in3Pin = 9;           // D9  -> L298N IN3，左电机方向
const int in4Pin = 10;          // D10 -> L298N IN4，左电机方向

/* ===================== ADS1115 红外传感器接口 ===================== */

const int IR_LEFT_CH   = 0;     // ADS1115 A0 -> 左红外 AO
const int IR_CENTER_CH = 1;     // ADS1115 A1 -> 中红外 AO
const int IR_RIGHT_CH  = 2;     // ADS1115 A2 -> 右红外 AO

/* ===================== I2C 地址 ===================== */

const int ADS1115_ADDR = 0x48;
const int BME280_ADDR1 = 0x76;
const int BME280_ADDR2 = 0x77;
const int MPU_ADDR     = 0x68;

/* ===================== MPU 寄存器地址 ===================== */

const byte MPU_REG_SMPLRT_DIV     = 0x19;
const byte MPU_REG_CONFIG         = 0x1A;
const byte MPU_REG_GYRO_CONFIG    = 0x1B;
const byte MPU_REG_ACCEL_CONFIG   = 0x1C;
const byte MPU_REG_ACCEL_CONFIG2  = 0x1D;
const byte MPU_REG_PWR_MGMT_1     = 0x6B;
const byte MPU_REG_PWR_MGMT_2     = 0x6C;
const byte MPU_REG_WHO_AM_I       = 0x75;

const byte MPU_REG_ACCEL_XOUT_H   = 0x3B;
const byte MPU_REG_GYRO_XOUT_H    = 0x43;

/* ===================== 可调参数 ===================== */

// 如果某一侧电机方向反了，把 false 改成 true，或者用串口命令 reverseR / reverseL 调试
bool reverseRightMotor = false;
bool reverseLeftMotor  = false;

// 抬起后电机前进速度，范围 0~255
int testForwardSpeed = 130;

// IR 黑线判断阈值
int irBlackThreshold = 10000;

// MPU 量程：加速度 ±2g，陀螺仪 ±250 deg/s
const float MPU_ACCEL_SCALE = 16384.0;
const float MPU_GYRO_SCALE  = 131.0;

// 抬起 / 下降判断阈值
float liftDiffG = 0.20;       // 检测向上动作
float downDiffG = -0.15;      // 检测开始下降

// 抬起确认次数
int liftConfirmNeed = 1;

// 向上触发后，短暂忽略“下降信号”
unsigned long liftSettleIgnoreDownMs = 1000;

// 下降停止后，屏蔽抬起触发，防止反弹误触发
unsigned long lowerToLiftLockoutMs = 1000;

// 时间控制
const unsigned long RECORD_INTERVAL_MS = 100;
const unsigned long PRINT_INTERVAL_MS  = 500;
const unsigned long MOTION_INTERVAL_MS = 50;

/* ===================== 传感器对象 ===================== */

Adafruit_ADS1115 ads;
Adafruit_BME280 bme;

/* ===================== 设备状态变量 ===================== */

bool adsOK = false;
bool bmeOK = false;
bool mpuOK = false;
byte mpuWhoAmI = 0x00;

/* ===================== MPU 基准变量 ===================== */

float baseAx = 0.0;
float baseAy = 0.0;
float baseAz = 0.0;

float baseNorm = 1.0;
float unitGx = 0.0;
float unitGy = 0.0;
float unitGz = 1.0;

/* ===================== 抬起 / 下降状态变量 ===================== */

int liftConfirmCount = 0;
int lowerConfirmCount = 0;

bool liftedRunningState = false;

unsigned long liftStartTime = 0;
unsigned long liftBlockUntilTime = 0;

/* ===================== 调试状态变量 ===================== */

bool debugPrintEnabled = true;
bool manualMotorMode = false;

int manualLeftSpeed = 0;
int manualRightSpeed = 0;

/* ===================== 记录数据变量 ===================== */

int16_t recordIrLeftRaw = 0;
int16_t recordIrCenterRaw = 0;
int16_t recordIrRightRaw = 0;

bool recordIrLeftBlack = false;
bool recordIrCenterBlack = false;
bool recordIrRightBlack = false;

float recordTemperatureC = 0.0;
float recordPressureHpa = 0.0;
float recordHumidity = 0.0;

float recordAx = 0.0;
float recordAy = 0.0;
float recordAz = 0.0;

float recordGx = 0.0;
float recordGy = 0.0;
float recordGz = 0.0;

float recordProjectedG = 0.0;
float recordDiffG = 0.0;

bool recordMPUReadOK = false;

/* ===================== 时间控制变量 ===================== */

unsigned long lastRecordTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastMotionTime = 0;

/* ===================== 电机控制函数 ===================== */

void setSingleMotor(int enablePin, int dirPin1, int dirPin2, int speedValue, bool reverseMotor)
{
  speedValue = constrain(speedValue, -255, 255);

  if (reverseMotor) {
    speedValue = -speedValue;
  }

  if (speedValue > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    analogWrite(enablePin, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    analogWrite(enablePin, -speedValue);
  }
  else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    analogWrite(enablePin, 0);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  setSingleMotor(enablePinLeft,  in3Pin, in4Pin, leftSpeed,  reverseLeftMotor);
  setSingleMotor(enablePinRight, in1Pin, in2Pin, rightSpeed, reverseRightMotor);
}

void stopMotors()
{
  setMotorSpeed(0, 0);
}

void runForwardForExperiment()
{
  setMotorSpeed(testForwardSpeed, testForwardSpeed);
}

/* ===================== MPU 原始寄存器操作 ===================== */

bool writeMPUByte(byte reg, byte data)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  byte error = Wire.endTransmission();

  return error == 0;
}

byte readMPUByte(byte reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return 0xFF;
  }

  byte count = Wire.requestFrom(MPU_ADDR, 1);

  if (count == 1 && Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool readMPUWord(byte reg, int16_t &value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    value = 0;
    return false;
  }

  byte count = Wire.requestFrom(MPU_ADDR, 2);

  if (count == 2 && Wire.available() >= 2) {
    int16_t highByte = Wire.read();
    int16_t lowByte = Wire.read();
    value = (highByte << 8) | lowByte;
    return true;
  }

  value = 0;
  return false;
}

bool readMPURawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;

  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;

  bool ok = true;

  ok = ok && readMPUWord(MPU_REG_ACCEL_XOUT_H,     axRaw);
  ok = ok && readMPUWord(MPU_REG_ACCEL_XOUT_H + 2, ayRaw);
  ok = ok && readMPUWord(MPU_REG_ACCEL_XOUT_H + 4, azRaw);

  ok = ok && readMPUWord(MPU_REG_GYRO_XOUT_H,     gxRaw);
  ok = ok && readMPUWord(MPU_REG_GYRO_XOUT_H + 2, gyRaw);
  ok = ok && readMPUWord(MPU_REG_GYRO_XOUT_H + 4, gzRaw);

  if (!ok) {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
    return false;
  }

  if (axRaw == 0 && ayRaw == 0 && azRaw == 0 &&
      gxRaw == 0 && gyRaw == 0 && gzRaw == 0) {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
    return false;
  }

  ax = axRaw / MPU_ACCEL_SCALE;
  ay = ayRaw / MPU_ACCEL_SCALE;
  az = azRaw / MPU_ACCEL_SCALE;

  gx = gxRaw / MPU_GYRO_SCALE;
  gy = gyRaw / MPU_GYRO_SCALE;
  gz = gzRaw / MPU_GYRO_SCALE;

  return true;
}

bool initMPURaw()
{
  mpuWhoAmI = readMPUByte(MPU_REG_WHO_AM_I);

  Serial.print(F("MPU WHO_AM_I: 0x"));
  Serial.println(mpuWhoAmI, HEX);

  if (mpuWhoAmI == 0x00 || mpuWhoAmI == 0xFF) {
    return false;
  }

  writeMPUByte(MPU_REG_PWR_MGMT_1, 0x80);
  delay(100);

  writeMPUByte(MPU_REG_PWR_MGMT_1, 0x01);
  delay(100);

  writeMPUByte(MPU_REG_PWR_MGMT_2, 0x00);
  delay(20);

  writeMPUByte(MPU_REG_CONFIG, 0x03);
  writeMPUByte(MPU_REG_SMPLRT_DIV, 0x07);
  writeMPUByte(MPU_REG_GYRO_CONFIG, 0x00);
  writeMPUByte(MPU_REG_ACCEL_CONFIG, 0x00);
  writeMPUByte(MPU_REG_ACCEL_CONFIG2, 0x03);

  delay(300);

  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;
  float gx = 0.0;
  float gy = 0.0;
  float gz = 0.0;

  for (int i = 0; i < 10; i++) {
    if (readMPURawData(ax, ay, az, gx, gy, gz)) {
      return true;
    }

    delay(100);
  }

  return false;
}

/* ===================== I2C 扫描 ===================== */

void scanI2CDevices()
{
  Serial.println();
  Serial.println(F("I2C Bus Scan:"));

  byte count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("  Device found at 0x"));

      if (address < 16) {
        Serial.print(F("0"));
      }

      Serial.println(address, HEX);
      count++;
    }
  }

  Serial.print(F("  Total devices detected: "));
  Serial.println(count);
  Serial.println();
}

/* ===================== MPU 基准校准 ===================== */

void calibrateMPUBase()
{
  Serial.println(F("Calibrating MPU baseline..."));
  Serial.println(F("Keep the robot still."));

  stopMotors();

  float sumAx = 0.0;
  float sumAy = 0.0;
  float sumAz = 0.0;
  int validCount = 0;

  delay(500);

  for (int i = 0; i < 50; i++) {
    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;

    if (readMPURawData(ax, ay, az, gx, gy, gz)) {
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      validCount++;
    }

    delay(20);
  }

  if (validCount > 0) {
    baseAx = sumAx / validCount;
    baseAy = sumAy / validCount;
    baseAz = sumAz / validCount;

    baseNorm = sqrt(baseAx * baseAx + baseAy * baseAy + baseAz * baseAz);

    if (baseNorm < 0.2) {
      baseNorm = 1.0;
      unitGx = 0.0;
      unitGy = 0.0;
      unitGz = 1.0;
    }
    else {
      unitGx = baseAx / baseNorm;
      unitGy = baseAy / baseNorm;
      unitGz = baseAz / baseNorm;
    }

    Serial.print(F("Baseline Ax="));
    Serial.print(baseAx, 3);
    Serial.print(F(" Ay="));
    Serial.print(baseAy, 3);
    Serial.print(F(" Az="));
    Serial.print(baseAz, 3);
    Serial.print(F(" Norm="));
    Serial.print(baseNorm, 3);
    Serial.println(F(" g"));
  }
  else {
    Serial.println(F("MPU baseline calibration failed."));
    mpuOK = false;
  }

  Serial.println();
}

/* ===================== 数据记录函数 ===================== */

void recordDisplayData()
{
  if (adsOK) {
    recordIrLeftRaw = ads.readADC_SingleEnded(IR_LEFT_CH);
    recordIrCenterRaw = ads.readADC_SingleEnded(IR_CENTER_CH);
    recordIrRightRaw = ads.readADC_SingleEnded(IR_RIGHT_CH);

    recordIrLeftBlack = recordIrLeftRaw > irBlackThreshold;
    recordIrCenterBlack = recordIrCenterRaw > irBlackThreshold;
    recordIrRightBlack = recordIrRightRaw > irBlackThreshold;
  }

  if (bmeOK) {
    recordTemperatureC = bme.readTemperature();
    recordPressureHpa = bme.readPressure() / 100.0;
    recordHumidity = bme.readHumidity();
  }

  if (mpuOK) {
    recordMPUReadOK = readMPURawData(
      recordAx, recordAy, recordAz,
      recordGx, recordGy, recordGz
    );

    if (recordMPUReadOK) {
      recordProjectedG = recordAx * unitGx + recordAy * unitGy + recordAz * unitGz;
      recordDiffG = recordProjectedG - baseNorm;
    }
  }
  else {
    recordMPUReadOK = false;
  }
}

/* ===================== 快速动作检测函数 ===================== */

void updateMotionState()
{
  if (!mpuOK) {
    stopMotors();
    return;
  }

  unsigned long now = millis();

  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;

  float gx = 0.0;
  float gy = 0.0;
  float gz = 0.0;

  bool ok = readMPURawData(ax, ay, az, gx, gy, gz);

  if (!ok) {
    return;
  }

  float projectedG = ax * unitGx + ay * unitGy + az * unitGz;
  float diffG = projectedG - baseNorm;

  recordProjectedG = projectedG;
  recordDiffG = diffG;

  if (liftedRunningState) {
    if (now - liftStartTime < liftSettleIgnoreDownMs) {
      runForwardForExperiment();
      return;
    }

    if (diffG < downDiffG) {
      liftedRunningState = false;
      liftConfirmCount = 0;
      lowerConfirmCount = 0;

      liftBlockUntilTime = now + lowerToLiftLockoutMs;

      stopMotors();

      if (debugPrintEnabled) {
        Serial.println(F("EVENT: DOWN detected -> motors stopped"));
      }

      return;
    }

    runForwardForExperiment();
    return;
  }

  if (now < liftBlockUntilTime) {
    liftConfirmCount = 0;
    stopMotors();
    return;
  }

  if (diffG < downDiffG) {
    liftConfirmCount = 0;
    lowerConfirmCount = 0;

    liftedRunningState = false;
    liftBlockUntilTime = now + lowerToLiftLockoutMs;

    stopMotors();
    return;
  }

  if (diffG > liftDiffG) {
    liftConfirmCount++;

    if (liftConfirmCount >= liftConfirmNeed) {
      liftedRunningState = true;
      liftStartTime = now;
      liftConfirmCount = 0;
      lowerConfirmCount = 0;

      runForwardForExperiment();

      if (debugPrintEnabled) {
        Serial.println(F("EVENT: LIFT detected -> motors running"));
      }

      return;
    }
  }
  else {
    liftConfirmCount = 0;
  }

  stopMotors();
}

/* ===================== 输出辅助函数 ===================== */

void printBW(bool isBlack)
{
  if (isBlack) {
    Serial.print(F("B"));
  }
  else {
    Serial.print(F("W"));
  }
}

/* ===================== 正式调试输出 ===================== */

void printFormalData()
{
  if (!debugPrintEnabled) {
    return;
  }

  Serial.println(F("--------------------------------------------------"));

  Serial.print(F("Mode: "));
  if (manualMotorMode) {
    Serial.println(F("MANUAL MOTOR TEST"));
  }
  else {
    Serial.println(F("AUTO LIFT DETECTION"));
  }

  Serial.print(F("IR: L="));
  Serial.print(recordIrLeftRaw);
  Serial.print(F("("));
  printBW(recordIrLeftBlack);
  Serial.print(F("), C="));
  Serial.print(recordIrCenterRaw);
  Serial.print(F("("));
  printBW(recordIrCenterBlack);
  Serial.print(F("), R="));
  Serial.print(recordIrRightRaw);
  Serial.print(F("("));
  printBW(recordIrRightBlack);
  Serial.print(F("), Threshold="));
  Serial.println(irBlackThreshold);

  Serial.print(F("BME280: "));
  if (bmeOK) {
    Serial.print(recordTemperatureC, 2);
    Serial.print(F(" C, "));
    Serial.print(recordHumidity, 2);
    Serial.print(F(" %, "));
    Serial.print(recordPressureHpa, 2);
    Serial.println(F(" hPa"));
  }
  else {
    Serial.println(F("NOT FOUND"));
  }

  Serial.print(F("MPU: "));
  if (recordMPUReadOK) {
    Serial.print(F("Ax="));
    Serial.print(recordAx, 3);
    Serial.print(F(" Ay="));
    Serial.print(recordAy, 3);
    Serial.print(F(" Az="));
    Serial.print(recordAz, 3);
    Serial.print(F(" | Gx="));
    Serial.print(recordGx, 2);
    Serial.print(F(" Gy="));
    Serial.print(recordGy, 2);
    Serial.print(F(" Gz="));
    Serial.print(recordGz, 2);
    Serial.println();

    Serial.print(F("Motion: projectedG="));
    Serial.print(recordProjectedG, 3);
    Serial.print(F(" diffG="));
    Serial.print(recordDiffG, 3);
    Serial.print(F(" liftThr="));
    Serial.print(liftDiffG, 3);
    Serial.print(F(" downThr="));
    Serial.println(downDiffG, 3);
  }
  else {
    Serial.println(F("READ FAILED"));
  }

  Serial.print(F("Robot State: "));
  if (liftedRunningState) {
    Serial.println(F("LIFTED / WHEELS RUNNING"));
  }
  else {
    Serial.println(F("LOWERED / WHEELS STOPPED"));
  }

  Serial.print(F("Motor: speed="));
  Serial.print(testForwardSpeed);
  Serial.print(F(" reverseL="));
  Serial.print(reverseLeftMotor ? F("true") : F("false"));
  Serial.print(F(" reverseR="));
  Serial.println(reverseRightMotor ? F("true") : F("false"));

  Serial.println(F("--------------------------------------------------"));
  Serial.println();
}

/* ===================== 串口调试命令 ===================== */

void printHelp()
{
  Serial.println();
  Serial.println(F("================ Debug Commands ================"));
  Serial.println(F("help                 -> show command list"));
  Serial.println(F("debug 1              -> enable debug print"));
  Serial.println(F("debug 0              -> disable debug print"));
  Serial.println(F("speed 130            -> set auto forward speed"));
  Serial.println(F("ir 10000             -> set IR black threshold"));
  Serial.println(F("lift 0.20            -> set lift threshold"));
  Serial.println(F("down -0.15           -> set down threshold"));
  Serial.println(F("reverseL 1           -> reverse left motor"));
  Serial.println(F("reverseL 0           -> normal left motor"));
  Serial.println(F("reverseR 1           -> reverse right motor"));
  Serial.println(F("reverseR 0           -> normal right motor"));
  Serial.println(F("motor 120 120        -> manual motor test, left right"));
  Serial.println(F("run                  -> manual run forward"));
  Serial.println(F("stop                 -> stop motor and exit manual mode"));
  Serial.println(F("auto                 -> exit manual mode, use lift detection"));
  Serial.println(F("base                 -> recalibrate MPU baseline"));
  Serial.println(F("scan                 -> scan I2C bus"));
  Serial.println(F("================================================"));
  Serial.println();
}

void handleSerialCommand()
{
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "help" || cmd == "h") {
    printHelp();
    return;
  }

  if (cmd.startsWith("debug ")) {
    int value = cmd.substring(6).toInt();
    debugPrintEnabled = (value != 0);
    Serial.print(F("Debug print = "));
    Serial.println(debugPrintEnabled ? F("ON") : F("OFF"));
    return;
  }

  if (cmd.startsWith("speed ")) {
    int value = cmd.substring(6).toInt();
    testForwardSpeed = constrain(value, 0, 255);
    Serial.print(F("Auto forward speed = "));
    Serial.println(testForwardSpeed);
    return;
  }

  if (cmd.startsWith("ir ")) {
    int value = cmd.substring(3).toInt();
    irBlackThreshold = value;
    Serial.print(F("IR black threshold = "));
    Serial.println(irBlackThreshold);
    return;
  }

  if (cmd.startsWith("lift ")) {
    float value = cmd.substring(5).toFloat();
    liftDiffG = value;
    Serial.print(F("Lift threshold = "));
    Serial.println(liftDiffG, 3);
    return;
  }

  if (cmd.startsWith("down ")) {
    float value = cmd.substring(5).toFloat();
    downDiffG = value;
    Serial.print(F("Down threshold = "));
    Serial.println(downDiffG, 3);
    return;
  }

  if (cmd.startsWith("reverseL ")) {
    int value = cmd.substring(9).toInt();
    reverseLeftMotor = (value != 0);
    Serial.print(F("reverseLeftMotor = "));
    Serial.println(reverseLeftMotor ? F("true") : F("false"));
    return;
  }

  if (cmd.startsWith("reverseR ")) {
    int value = cmd.substring(9).toInt();
    reverseRightMotor = (value != 0);
    Serial.print(F("reverseRightMotor = "));
    Serial.println(reverseRightMotor ? F("true") : F("false"));
    return;
  }

  if (cmd.startsWith("motor ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    if (secondSpace > 0) {
      manualLeftSpeed = cmd.substring(firstSpace + 1, secondSpace).toInt();
      manualRightSpeed = cmd.substring(secondSpace + 1).toInt();

      manualLeftSpeed = constrain(manualLeftSpeed, -255, 255);
      manualRightSpeed = constrain(manualRightSpeed, -255, 255);

      manualMotorMode = true;
      liftedRunningState = false;

      setMotorSpeed(manualLeftSpeed, manualRightSpeed);

      Serial.print(F("Manual motor: left="));
      Serial.print(manualLeftSpeed);
      Serial.print(F(" right="));
      Serial.println(manualRightSpeed);
    }
    else {
      Serial.println(F("Wrong format. Use: motor 120 120"));
    }

    return;
  }

  if (cmd == "run") {
    manualMotorMode = true;
    liftedRunningState = false;

    manualLeftSpeed = testForwardSpeed;
    manualRightSpeed = testForwardSpeed;

    setMotorSpeed(manualLeftSpeed, manualRightSpeed);

    Serial.print(F("Manual run forward, speed = "));
    Serial.println(testForwardSpeed);
    return;
  }

  if (cmd == "stop") {
    manualMotorMode = false;
    liftedRunningState = false;
    liftConfirmCount = 0;
    lowerConfirmCount = 0;

    stopMotors();

    Serial.println(F("Motors stopped. Manual mode OFF."));
    return;
  }

  if (cmd == "auto") {
    manualMotorMode = false;
    liftedRunningState = false;
    liftConfirmCount = 0;
    lowerConfirmCount = 0;

    stopMotors();

    Serial.println(F("Auto lift detection mode ON."));
    return;
  }

  if (cmd == "base") {
    if (mpuOK) {
      calibrateMPUBase();
    }
    else {
      Serial.println(F("MPU not OK, cannot calibrate."));
    }
    return;
  }

  if (cmd == "scan") {
    scanI2CDevices();
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(cmd);
  Serial.println(F("Type help to show command list."));
}

/* ===================== 初始化 ===================== */

void setup()
{
  pinMode(enablePinRight, OUTPUT);
  pinMode(enablePinLeft, OUTPUT);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("Robot I2C Sensor Demonstration - No LED Version"));
  Serial.println(F("BME280 + MPU-9250 + ADS1115"));
  Serial.println(F("Lift robot upward: wheels run forward"));
  Serial.println(F("Hold robot in the air: wheels keep running"));
  Serial.println(F("Move robot downward: wheels stop"));
  Serial.println(F("Landing rebound will be ignored"));
  Serial.println(F("No LED pins are used"));
  Serial.println(F("Serial Monitor: 115200 baud"));
  Serial.println(F("Type help to show debug commands"));
  Serial.println(F("=============================================="));
  Serial.println();

  Wire.begin();
  Wire.setClock(100000);

  scanI2CDevices();

  ads.setGain(GAIN_TWOTHIRDS);

  if (ads.begin(ADS1115_ADDR)) {
    adsOK = true;
    Serial.println(F("ADS1115: OK"));
  }
  else {
    adsOK = false;
    Serial.println(F("ADS1115: NOT FOUND"));
  }

  if (bme.begin(BME280_ADDR1)) {
    bmeOK = true;
    Serial.println(F("BME280: OK at 0x76"));
  }
  else if (bme.begin(BME280_ADDR2)) {
    bmeOK = true;
    Serial.println(F("BME280: OK at 0x77"));
  }
  else {
    bmeOK = false;
    Serial.println(F("BME280: NOT FOUND"));
  }

  if (bmeOK) {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X2,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_X16,
      Adafruit_BME280::STANDBY_MS_500
    );
  }

  if (initMPURaw()) {
    mpuOK = true;
    Serial.println(F("MPU-9250 / MPU sensor: OK"));
  }
  else {
    mpuOK = false;
    Serial.println(F("MPU-9250 / MPU sensor: NOT READY"));
  }

  Serial.println();

  if (mpuOK) {
    calibrateMPUBase();
  }

  recordDisplayData();

  Serial.println(F("Setup finished."));
  printHelp();

  stopMotors();
}

/* ===================== 主循环 ===================== */

void loop()
{
  unsigned long now = millis();

  handleSerialCommand();

  if (manualMotorMode) {
    setMotorSpeed(manualLeftSpeed, manualRightSpeed);
  }
  else {
    if (now - lastMotionTime >= MOTION_INTERVAL_MS) {
      lastMotionTime = now;
      updateMotionState();
    }

    if (liftedRunningState) {
      runForwardForExperiment();
    }
    else {
      stopMotors();
    }
  }

  if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
    lastRecordTime = now;
    recordDisplayData();
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printFormalData();
  }

  delay(20);
}
