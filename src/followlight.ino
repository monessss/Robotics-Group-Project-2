/*
  Optimized PID Light / Line Following Car 最终版

  改进点：
  1. 增加误差死区，直线时不频繁修正
  2. 增加输出死区，小 correction 直接清零
  3. 增加 correction 变化速率限制，减少直线摆尾
  4. 增加动态输出上限，小误差慢修，大误差强转
  5. 增加左右轮基础速度补偿
*/

// ---------------- 电机引脚定义 ----------------
int in1 = 2, in2 = 3, ENA = 9;   // 左电机
int in3 = 4, in4 = 5, ENB = 10;  // 右电机

// ---------------- 光敏电阻供电引脚 ----------------
const int leftLdrPower = 7;
const int rightLdrPower = 8;

// ---------------- 光敏电阻信号引脚 ----------------
const int leftLdrPin = A1;
const int rightLdrPin = A2;

// ---------------- PID 参数 ----------------
// Kp 不要太大，否则直线会摆尾
// Kd 用来抑制摆动
float Kp = 0.20;
float Ki = 0.00;
float Kd = 0.055;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

float rawCorrection = 0;
float correction = 0;
float lastCorrection = 0;

// ---------------- 速度参数 ----------------
// 如果车直线还偏：优先调这两个
int baseLeftSpeed = 85;
int baseRightSpeed = 81;

// 基础速度不要太高，否则急弯容易丢
int minSpeed = 25;
int maxSpeed = 115;

// correction 最大输出
int smallMaxCorrection = 20;   // 小偏差时最大修正，防止直线摆尾
int largeMaxCorrection = 42;   // 大偏差时最大修正，增强转弯能力

// ---------------- 死区设置 ----------------
int deadBand = 45;             // 误差死区，小于这个认为基本居中
int outputDeadBand = 9;        // 输出死区，小于这个 correction 直接为 0
int straightBand = 56;         // 直线锁定范围，减少左右反复修正

// ---------------- 大误差判断 ----------------
int largeErrorThreshold = 180; // 超过这个认为进入弯道或偏离较大

// ---------------- 滤波参数 ----------------
// alpha 越大越平稳，但响应慢
// alpha 越小响应快，但容易抖
float alpha = 0.40;

float filteredLeft = 0;
float filteredRight = 0;

// ---------------- correction 限速 ----------------
// 每次 correction 最大变化量，防止突然左右甩
float maxCorrectionStep = 4.2;

// ---------------- 输出速度 ----------------
int leftSpeed = 0;
int rightSpeed = 0;

// ---------------- 时间控制 ----------------
unsigned long lastTime = 0;
int sampleTime = 12;

unsigned long lastPrintTime = 0;
int printInterval = 200;

// ---------------- 方向记忆 ----------------
// 1 表示上一次偏右边，需要左转
// -1 表示上一次偏左边，需要右转
// 0 表示没有明显方向
int lastTurnDirection = 0;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(leftLdrPower, OUTPUT);
  pinMode(rightLdrPower, OUTPUT);
  digitalWrite(leftLdrPower, HIGH);
  digitalWrite(rightLdrPower, HIGH);

  Serial.begin(9600);
  Serial.println("Optimized PID car started");

  delay(200);

  filteredLeft = analogRead(leftLdrPin);
  filteredRight = analogRead(rightLdrPin);
}

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= sampleTime) {
    lastTime = now;

    readSensors();
    calculatePID();
    driveMotors();
  }

  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;
    printDebug();
  }
}

void readSensors() {
  int leftVal = analogRead(leftLdrPin);
  int rightVal = analogRead(rightLdrPin);

  filteredLeft = alpha * filteredLeft + (1 - alpha) * leftVal;
  filteredRight = alpha * filteredRight + (1 - alpha) * rightVal;
}

void calculatePID() {
  /*
    error = right - left

    error > 0：
    右边数值更大，说明右边更暗 / 偏差在右边，需要左转
    左转 = 左轮慢，右轮快

    error < 0：
    左边数值更大，需要右转
    右转 = 左轮快，右轮慢
  */

  float currentError = filteredRight - filteredLeft;

  // 记录明显偏差方向
  if (currentError > deadBand) {
    lastTurnDirection = 1;
  } else if (currentError < -deadBand) {
    lastTurnDirection = -1;
  }

  // 误差死区
  if (abs(currentError) < deadBand) {
    currentError = 0;
  }

  // 直线锁定区
  // 作用：车已经基本直了，就不要频繁左右修正，解决摆尾
  if (abs(currentError) < straightBand) {
    error = 0;
    integral = 0;
    derivative = 0;
    rawCorrection = 0;
  } else {
    error = currentError;

    integral += error;
    integral = constrain(integral, -250, 250);

    derivative = error - lastError;

    rawCorrection = Kp * error + Ki * integral + Kd * derivative;
  }

  lastError = error;

  // 动态 correction 上限
  // 小偏差限制小一点，直线不摆
  // 大偏差限制大一点，弯道能转
  int currentMaxCorrection;

  if (abs(error) > largeErrorThreshold) {
    currentMaxCorrection = largeMaxCorrection;
  } else {
    currentMaxCorrection = smallMaxCorrection;
  }

  rawCorrection = constrain(rawCorrection, -currentMaxCorrection, currentMaxCorrection);

  // 输出死区
  if (abs(rawCorrection) < outputDeadBand) {
    rawCorrection = 0;
  }

  // correction 变化速率限制
  // 防止 correction 一下从正变负，导致摆尾
  float diff = rawCorrection - lastCorrection;

  if (diff > maxCorrectionStep) {
    correction = lastCorrection + maxCorrectionStep;
  } else if (diff < -maxCorrectionStep) {
    correction = lastCorrection - maxCorrectionStep;
  } else {
    correction = rawCorrection;
  }

  lastCorrection = correction;
}

void driveMotors() {
  /*
    correction > 0：左转
    左轮慢，右轮快

    correction < 0：右转
    左轮快，右轮慢
  */

  leftSpeed = baseLeftSpeed - correction;
  rightSpeed = baseRightSpeed + correction;

  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  runMotor(1, leftSpeed, false);
  runMotor(2, rightSpeed, false);
}

void runMotor(int motor, int speed, bool reverse) {
  speed = constrain(speed, 0, 255);

  if (motor == 1) {
    digitalWrite(in1, reverse);
    digitalWrite(in2, !reverse);
    analogWrite(ENA, speed);
  } else {
    digitalWrite(in3, !reverse);
    digitalWrite(in4, reverse);
    analogWrite(ENB, speed);
  }
}

void printDebug() {
  Serial.print("Left: ");
  Serial.print((int)filteredLeft);

  Serial.print("  Right: ");
  Serial.print((int)filteredRight);

  Serial.print("  Error(R-L): ");
  Serial.print(error);

  Serial.print("  RawCorrection: ");
  Serial.print(rawCorrection);

  Serial.print("  Correction: ");
  Serial.print(correction);

  Serial.print("  LSpeed: ");
  Serial.print(leftSpeed);

  Serial.print("  RSpeed: ");
  Serial.print(rightSpeed);

  Serial.print("  LastDir: ");
  Serial.println(lastTurnDirection);
}
