#include <Servo.h>

/*
  Obstacle avoidance car code with continuous 30-degree head sweep.

  New logic requested:
  1. During normal forward motion, the servo continuously sweeps in a small
     30-degree sector around the front direction.
  2. If the measured distance is smaller than the near trigger threshold,
     the car stops and performs a wide left/right scan.
  3. The wide scan records left and right distances separately.
  4. The side with the larger distance is treated as the safer rear-side
     backing direction.
  5. The car backs toward that side with an arc motion, then randomly pivots about 60 degrees before recovering forward.
  6. Left/right wheel speed trim is added to correct straight-line deviation.

  Motor direction logic is kept from the previous tested version.
*/

//*********************** Motor pins: 6-wire mode *************************

const int enablePin1 = 5;   // ENA, right motor enable / PWM
const int enablePin2 = 6;   // ENB, left motor enable / PWM

const int in1Pin = 4;       // Right motor IN1
const int in2Pin = 7;       // Right motor IN2
const int in3Pin = 9;       // Left motor IN3
const int in4Pin = 10;      // Left motor IN4

//*********************** Speed parameters *************************

const int maxSpeed = 255;

// Base values. The actual output will also pass through the trim values below.
const int baseSpeed = 130;          // normal forward speed
const int backSpeed = 175;          // normal backward speed
const int pivotSpeed = 160;         // kept for old turn functions
const int randomPivotSpeed = 165;   // in-place anti-stuck pivot speed after backing

// Straight-line correction.
// If the car turns LEFT while it should go straight, usually the RIGHT wheel is faster:
//   reduce rightForwardTrim or increase leftForwardTrim.
// If the car turns RIGHT while it should go straight, usually the LEFT wheel is faster:
//   reduce leftForwardTrim or increase rightForwardTrim.
const int rightForwardTrim = 0;
const int leftForwardTrim = 0;

const int rightBackwardTrim = 0;
const int leftBackwardTrim = 0;

// Arc-back parameters. The outer wheel moves faster than the inner wheel.
const int backArcOuterSpeed = 180;
const int backArcInnerSpeed = 115;

//*********************** Servo and ultrasonic pins *************************

const int servoPin = 8;

const int inputPin = 12;        // Echo
const int outputPin = 11;       // Trig

Servo myservo;

//*********************** Servo angle settings *************************

// Mechanical center. Change this if the ultrasonic sensor is not exactly forward at 90 degrees.
const int centerAngle = 90;

// Continuous search sector: total 30 degrees.
const int sweepLeftAngle = centerAngle + 20;    // 105 degrees
const int sweepRightAngle = centerAngle - 20;   // 75 degrees

// Wide scan angles. These are near side-looking angles.
// IMPORTANT:
// On some cars the servo is mounted in the opposite direction.
// In that case, a larger servo angle may physically look RIGHT instead of LEFT.
// Your current test symptom is: obstacle side and backing side look the same.
// That usually means the servo left/right labels are reversed.
// Therefore this version uses servoLeftRightReversed = true by default.
const bool servoLeftRightReversed = true;

const int wideHighAngle = 160;
const int wideLowAngle = 20;

const int wideLeftAngle = servoLeftRightReversed ? wideLowAngle : wideHighAngle;
const int wideRightAngle = servoLeftRightReversed ? wideHighAngle : wideLowAngle;

const int servoMinAngle = 20;
const int servoMaxAngle = 160;

// Use -1 to make sure the servo receives the first center command in setup().
int currentServoAngle = -1;

// Blocking movement for large scan only.
const int servoStepAngle = 3;
const int servoStepDelay = 6;

// Non-blocking continuous sweep parameters.
const unsigned long sweepServoMoveInterval = 15;  // slower head sweep, but detection below is faster
const int sweepMoveStepAngle = 2;          // smaller step for smoother sweep
int sweepAngle = centerAngle;
int sweepDirection = 1;
unsigned long lastSweepServoMoveTime = 0;

//*********************** Distance parameters *************************

const int frontDangerDistance = 38;       // kept name for compatibility
const int nearTriggerDistance = 38;       // current near threshold for sweep trigger
const int veryCloseDistance = 12;         // very close obstacle, cm
const int maxValidDistance = 400;         // cm

// Faster ultrasonic timeout. 12000 us is about 2 m, enough for this car.
// It prevents a no-echo case from blocking the loop for 30 ms.
const unsigned long ultrasonicTimeoutUS = 7000;  // shorter timeout: less waiting when there is no echo

// With continuous sweeping, one confirmed near hit is enough.
const int obstacleConfirmTimes = 1;
int obstacleCount = 0;

// Record nearest distance seen during the current forward sweep.
int sweepNearestDistance = maxValidDistance;
int sweepTriggerAngle = centerAngle;

//*********************** Time parameters *************************

const unsigned long sweepCheckInterval = 20;       // emergency front check interval; stop as soon as one near hit appears
const unsigned long stopBeforeWideScanTime = 20;   // only a very short brake pause before wide scan
const unsigned long wideServoSettleTime = 180;     // allow wide scan servo + ultrasonic to settle
const unsigned long backArcTime = 600;             // backward arc time after direction decision
const unsigned long backArcLongTime = 700;         // longer back arc if obstacle is very close

// Anti-stuck action after backing:
// The car does not have an encoder/gyro here, so "60 degrees" is time-based.
// Tune randomPivot60Time after real testing:
//   rotate too little  -> increase it by 30~50 ms
//   rotate too much    -> decrease it by 30~50 ms
const unsigned long randomPivot60Time = 320;       // approximate in-place 60-degree pivot time
const unsigned long recoverTime = 120;             // short forward recovery

// Serial printing can slow the car heavily at 9600 baud, so use 115200 and throttle prints.
const unsigned long debugPrintInterval = 450;      // print less often so Serial does not delay emergency stop
unsigned long lastDebugPrintTime = 0;

//*********************** Distance variables *************************

int Fspeedd = maxValidDistance;
int Lspeedd = maxValidDistance;
int Rspeedd = maxValidDistance;

//*********************** State machine *************************

enum CarState {
  STATE_FORWARD,
  STATE_STOP_BEFORE_BACK,
  STATE_BACKWARD_SMALL,       // kept for compatibility; no longer used by the new logic
  STATE_SCAN_LEFT,
  STATE_SCAN_RIGHT,
  STATE_BACK_ARC_LEFT,
  STATE_BACK_ARC_RIGHT,
  STATE_RANDOM_PIVOT_LEFT,
  STATE_RANDOM_PIVOT_RIGHT,
  STATE_TURN_LEFT,            // kept for compatibility; no longer used by the new logic
  STATE_TURN_RIGHT,           // kept for compatibility; no longer used by the new logic
  STATE_RECOVER
};

CarState carState = STATE_FORWARD;
CarState plannedBackState = STATE_BACK_ARC_LEFT;
CarState plannedTurnState = STATE_TURN_LEFT;   // kept for old helper compatibility

unsigned long stateStartTime = 0;
unsigned long lastFrontCheckTime = 0;

//*********************** Function declarations *************************

void safeServoWriteAngle(int targetAngle);
void directServoWriteAngle(int targetAngle);
void resetForwardSweep();
void updateForwardSweep(unsigned long now);
void enterState(CarState newState);

void handleForwardState(unsigned long now);
void handleStopBeforeBackState(unsigned long now);
void handleBackwardSmallState(unsigned long now);
void handleScanLeftState(unsigned long now);
void handleScanRightState(unsigned long now);
void handleBackArcLeftState(unsigned long now);
void handleBackArcRightState(unsigned long now);
void handleRandomPivotLeftState(unsigned long now);
void handleRandomPivotRightState(unsigned long now);
void startRandomPivotAfterBack();
void handleTurnLeftState(unsigned long now);
void handleTurnRightState(unsigned long now);
void handleRecoverState(unsigned long now);

void decideBackDirection();
void decideTurnDirection();

int readDistanceCM();
int readDistanceWideCM(byte samples);
bool isObstacleWithLimit(int distance, int dangerDistance);
bool isObstacle(int distance);

bool isPwmPin(int pin);
void writeEnable(int pin, int speedValue);
int trimmedSpeed(int baseValue, int trimValue);

void rightMotorForward(int speedValue);
void rightMotorBackward(int speedValue);
void rightMotorStop();
void leftMotorForward(int speedValue);
void leftMotorBackward(int speedValue);
void leftMotorStop();
void stopMotors();

void advance();
void back();
void backArcLeft();
void backArcRight();
void randomPivotLeft();
void randomPivotRight();
void turnL();
void turnR();

//********************************************************************
// SETUP
//********************************************************************

void setup()
{
  Serial.begin(115200);
  randomSeed(analogRead(A0) ^ micros());

  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  myservo.attach(servoPin);

  safeServoWriteAngle(centerAngle);
  delay(250);

  stopMotors();

  Serial.println("------------------------------");
  Serial.println("Obstacle avoidance code started.");
  Serial.println("Logic: 30-degree continuous sweep -> wide scan -> arc back -> random 60-degree pivot -> recover forward.");
  Serial.println("Servo pin: D8.");
  Serial.println("Forward sweep: 75 to 105 degrees.");
  Serial.print("Emergency check interval ms: ");
  Serial.println(sweepCheckInterval);
  Serial.print("Ultrasonic timeout us: ");
  Serial.println(ultrasonicTimeoutUS);
  Serial.print("Sweep move interval ms: ");
  Serial.println(sweepServoMoveInterval);
  Serial.print("Sweep move step deg: ");
  Serial.println(sweepMoveStepAngle);
  Serial.print("Random anti-stuck pivot time ms: ");
  Serial.println(randomPivot60Time);
  Serial.print("Random anti-stuck pivot speed: ");
  Serial.println(randomPivotSpeed);
  Serial.print("Servo LR reversed: ");
  Serial.println(servoLeftRightReversed ? "YES" : "NO");
  Serial.print("Physical LEFT scan angle: ");
  Serial.println(wideLeftAngle);
  Serial.print("Physical RIGHT scan angle: ");
  Serial.println(wideRightAngle);
  Serial.println("Serial baud: 115200.");
  Serial.println("Wheel trim enabled: tune rightForwardTrim / leftForwardTrim if the car cannot go straight.");
  Serial.println("------------------------------");

  enterState(STATE_FORWARD);
}

//********************************************************************
// LOOP
//********************************************************************

void loop()
{
  unsigned long now = millis();

  switch (carState) {

    case STATE_FORWARD:
      handleForwardState(now);
      break;

    case STATE_STOP_BEFORE_BACK:
      handleStopBeforeBackState(now);
      break;

    case STATE_BACKWARD_SMALL:
      handleBackwardSmallState(now);
      break;

    case STATE_SCAN_LEFT:
      handleScanLeftState(now);
      break;

    case STATE_SCAN_RIGHT:
      handleScanRightState(now);
      break;

    case STATE_BACK_ARC_LEFT:
      handleBackArcLeftState(now);
      break;

    case STATE_BACK_ARC_RIGHT:
      handleBackArcRightState(now);
      break;

    case STATE_RANDOM_PIVOT_LEFT:
      handleRandomPivotLeftState(now);
      break;

    case STATE_RANDOM_PIVOT_RIGHT:
      handleRandomPivotRightState(now);
      break;

    case STATE_TURN_LEFT:
      handleTurnLeftState(now);
      break;

    case STATE_TURN_RIGHT:
      handleTurnRightState(now);
      break;

    case STATE_RECOVER:
      handleRecoverState(now);
      break;
  }
}

//********************************************************************
// Safe servo angle control
//********************************************************************

void safeServoWriteAngle(int targetAngle)
{
  targetAngle = constrain(targetAngle, servoMinAngle, servoMaxAngle);

  if (targetAngle == currentServoAngle) {
    return;
  }

  if (currentServoAngle < 0) {
    myservo.write(targetAngle);
    currentServoAngle = targetAngle;

    Serial.print("Servo angle: ");
    Serial.println(currentServoAngle);

    return;
  }

  if (targetAngle > currentServoAngle) {
    for (int angle = currentServoAngle; angle <= targetAngle; angle += servoStepAngle) {
      myservo.write(angle);
      delay(servoStepDelay);
    }
  }
  else {
    for (int angle = currentServoAngle; angle >= targetAngle; angle -= servoStepAngle) {
      myservo.write(angle);
      delay(servoStepDelay);
    }
  }

  // Make sure the final target is written even when the step does not land exactly on it.
  myservo.write(targetAngle);
  currentServoAngle = targetAngle;

  Serial.print("Servo angle: ");
  Serial.println(currentServoAngle);
}

void directServoWriteAngle(int targetAngle)
{
  targetAngle = constrain(targetAngle, servoMinAngle, servoMaxAngle);
  myservo.write(targetAngle);
  currentServoAngle = targetAngle;
}

void resetForwardSweep()
{
  sweepAngle = centerAngle;
  sweepDirection = 1;
  lastSweepServoMoveTime = millis();
  directServoWriteAngle(sweepAngle);

  sweepNearestDistance = maxValidDistance;
  sweepTriggerAngle = centerAngle;
  obstacleCount = 0;

  // Force the first distance check to happen quickly.
  lastFrontCheckTime = millis() - sweepCheckInterval;
}

void updateForwardSweep(unsigned long now)
{
  if (now - lastSweepServoMoveTime < sweepServoMoveInterval) {
    return;
  }

  lastSweepServoMoveTime = now;

  sweepAngle += sweepDirection * sweepMoveStepAngle;

  if (sweepAngle >= sweepLeftAngle) {
    sweepAngle = sweepLeftAngle;
    sweepDirection = -1;
  }
  else if (sweepAngle <= sweepRightAngle) {
    sweepAngle = sweepRightAngle;
    sweepDirection = 1;
  }

  directServoWriteAngle(sweepAngle);
}

//********************************************************************
// State transition function
//********************************************************************

void enterState(CarState newState)
{
  carState = newState;
  stateStartTime = millis();

  switch (newState) {

    case STATE_FORWARD:
      Serial.println("STATE_FORWARD: move forward with continuous 30-degree sweep.");
      resetForwardSweep();
      advance();
      break;

    case STATE_STOP_BEFORE_BACK:
      stopMotors();
      Serial.println("STATE_STOP_BEFORE_BACK: stopped, prepare wide scan.");
      break;

    case STATE_BACKWARD_SMALL:
      // Compatibility state. The new logic uses back-arc left/right instead.
      Serial.println("STATE_BACKWARD_SMALL: compatibility straight back.");
      safeServoWriteAngle(centerAngle);
      back();
      break;

    case STATE_SCAN_LEFT:
      Serial.println("STATE_SCAN_LEFT: wide scan PHYSICAL LEFT.");
      stopMotors();
      safeServoWriteAngle(wideLeftAngle);
      break;

    case STATE_SCAN_RIGHT:
      Serial.println("STATE_SCAN_RIGHT: wide scan PHYSICAL RIGHT.");
      stopMotors();
      safeServoWriteAngle(wideRightAngle);
      break;

    case STATE_BACK_ARC_LEFT:
      Serial.println("STATE_BACK_ARC_LEFT: left side is safer, back toward left rear side.");
      safeServoWriteAngle(centerAngle);
      backArcLeft();
      break;

    case STATE_BACK_ARC_RIGHT:
      Serial.println("STATE_BACK_ARC_RIGHT: right side is safer, back toward right rear side.");
      safeServoWriteAngle(centerAngle);
      backArcRight();
      break;

    case STATE_RANDOM_PIVOT_LEFT:
      Serial.println("STATE_RANDOM_PIVOT_LEFT: anti-stuck random in-place pivot left, about 60 degrees.");
      safeServoWriteAngle(centerAngle);
      randomPivotLeft();
      break;

    case STATE_RANDOM_PIVOT_RIGHT:
      Serial.println("STATE_RANDOM_PIVOT_RIGHT: anti-stuck random in-place pivot right, about 60 degrees.");
      safeServoWriteAngle(centerAngle);
      randomPivotRight();
      break;

    case STATE_TURN_LEFT:
      // Kept for compatibility. New main logic does not use in-place turn.
      Serial.println("STATE_TURN_LEFT: compatibility in-place left turn.");
      safeServoWriteAngle(centerAngle);
      turnL();
      break;

    case STATE_TURN_RIGHT:
      // Kept for compatibility. New main logic does not use in-place turn.
      Serial.println("STATE_TURN_RIGHT: compatibility in-place right turn.");
      safeServoWriteAngle(centerAngle);
      turnR();
      break;

    case STATE_RECOVER:
      Serial.println("STATE_RECOVER: move forward shortly after backing.");
      safeServoWriteAngle(centerAngle);
      advance();
      break;
  }
}

//********************************************************************
// State handlers
//********************************************************************

void handleForwardState(unsigned long now)
{
  // Keep motors running from the previous command, but do not refresh advance()
  // before checking distance. This avoids giving the car another forward command
  // right before an emergency stop.

  updateForwardSweep(now);

  if (now - lastFrontCheckTime >= sweepCheckInterval) {
    lastFrontCheckTime = now;

    Fspeedd = readDistanceCM();

    if (Fspeedd < sweepNearestDistance) {
      sweepNearestDistance = Fspeedd;
      sweepTriggerAngle = currentServoAngle;
    }

    // Highest-priority emergency decision:
    // one valid reading below the threshold stops the car immediately.
    if (isObstacleWithLimit(Fspeedd, nearTriggerDistance)) {
      obstacleCount = 1;
      stopMotors();

      Serial.print("EMERGENCY STOP at angle ");
      Serial.print(currentServoAngle);
      Serial.print(", distance ");
      Serial.println(Fspeedd);

      enterState(STATE_STOP_BEFORE_BACK);
      return;
    }

    obstacleCount = 0;

    if (now - lastDebugPrintTime >= debugPrintInterval) {
      lastDebugPrintTime = now;
      Serial.print("Sweep angle: ");
      Serial.print(currentServoAngle);
      Serial.print("  Distance: ");
      Serial.print(Fspeedd);
      Serial.print("  Nearest: ");
      Serial.println(sweepNearestDistance);
    }
  }

  // Only keep moving forward after the emergency check has passed.
  advance();
}


void handleStopBeforeBackState(unsigned long now)
{
  if (now - stateStartTime >= stopBeforeWideScanTime) {
    enterState(STATE_SCAN_LEFT);
  }
}

void handleBackwardSmallState(unsigned long now)
{
  unsigned long backTime;

  if (Fspeedd <= veryCloseDistance) {
    backTime = backArcLongTime;
  }
  else {
    backTime = backArcTime;
  }

  if (now - stateStartTime >= backTime) {
    enterState(STATE_RECOVER);
  }
}

void handleScanLeftState(unsigned long now)
{
  if (now - stateStartTime >= wideServoSettleTime) {
    Lspeedd = readDistanceWideCM(2);

    Serial.print("Left wide distance: ");
    Serial.println(Lspeedd);

    enterState(STATE_SCAN_RIGHT);
  }
}

void handleScanRightState(unsigned long now)
{
  if (now - stateStartTime >= wideServoSettleTime) {
    Rspeedd = readDistanceWideCM(2);

    Serial.print("Right wide distance: ");
    Serial.println(Rspeedd);

    decideBackDirection();

    enterState(plannedBackState);
  }
}

void handleBackArcLeftState(unsigned long now)
{
  unsigned long activeBackArcTime;

  if (Fspeedd <= veryCloseDistance) {
    activeBackArcTime = backArcLongTime;
  }
  else {
    activeBackArcTime = backArcTime;
  }

  if (now - stateStartTime >= activeBackArcTime) {
    stopMotors();
    startRandomPivotAfterBack();
  }
}

void handleBackArcRightState(unsigned long now)
{
  unsigned long activeBackArcTime;

  if (Fspeedd <= veryCloseDistance) {
    activeBackArcTime = backArcLongTime;
  }
  else {
    activeBackArcTime = backArcTime;
  }

  if (now - stateStartTime >= activeBackArcTime) {
    stopMotors();
    startRandomPivotAfterBack();
  }
}

void startRandomPivotAfterBack()
{
  // Randomly choose left/right in-place rotation to avoid being trapped
  // repeatedly inside the same corner or V-shaped gap.
  if (random(0, 2) == 0) {
    enterState(STATE_RANDOM_PIVOT_LEFT);
  }
  else {
    enterState(STATE_RANDOM_PIVOT_RIGHT);
  }
}

void handleRandomPivotLeftState(unsigned long now)
{
  if (now - stateStartTime >= randomPivot60Time) {
    stopMotors();
    enterState(STATE_RECOVER);
  }
}

void handleRandomPivotRightState(unsigned long now)
{
  if (now - stateStartTime >= randomPivot60Time) {
    stopMotors();
    enterState(STATE_RECOVER);
  }
}

void handleTurnLeftState(unsigned long now)
{
  // Compatibility only. New logic normally does not enter this state.
  if (now - stateStartTime >= backArcTime) {
    enterState(STATE_RECOVER);
  }
}

void handleTurnRightState(unsigned long now)
{
  // Compatibility only. New logic normally does not enter this state.
  if (now - stateStartTime >= backArcTime) {
    enterState(STATE_RECOVER);
  }
}

void handleRecoverState(unsigned long now)
{
  if (now - stateStartTime >= recoverTime) {
    Fspeedd = maxValidDistance;
    Lspeedd = maxValidDistance;
    Rspeedd = maxValidDistance;
    obstacleCount = 0;

    enterState(STATE_FORWARD);
  }
}

//********************************************************************
// Decide backing direction
//********************************************************************

void decideBackDirection()
{
  Serial.println("Deciding rear backing direction...");

  Serial.print("L = ");
  Serial.print(Lspeedd);
  Serial.print(" cm, R = ");
  Serial.print(Rspeedd);
  Serial.println(" cm");

  if (Lspeedd >= Rspeedd) {
    plannedBackState = STATE_BACK_ARC_LEFT;
    Serial.println("Left side has larger distance. Backing toward left rear side.");
  }
  else {
    plannedBackState = STATE_BACK_ARC_RIGHT;
    Serial.println("Right side has larger distance. Backing toward right rear side.");
  }
}

//********************************************************************
// Old turn direction helper, kept for compatibility
//********************************************************************

void decideTurnDirection()
{
  Serial.println("Deciding turn direction... ");

  if (Lspeedd < frontDangerDistance && Rspeedd < frontDangerDistance) {
    if (Lspeedd >= Rspeedd) {
      plannedTurnState = STATE_TURN_LEFT;
      Serial.println("Both sides close. Left is slightly better.");
    }
    else {
      plannedTurnState = STATE_TURN_RIGHT;
      Serial.println("Both sides close. Right is slightly better.");
    }
  }
  else if (Lspeedd > Rspeedd) {
    plannedTurnState = STATE_TURN_LEFT;
    Serial.println("Left side is more open.");
  }
  else {
    plannedTurnState = STATE_TURN_RIGHT;
    Serial.println("Right side is more open.");
  }
}

//********************************************************************
// Ultrasonic distance functions
//********************************************************************

int readDistanceCM()
{
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);

  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);

  unsigned long duration = pulseIn(inputPin, HIGH, ultrasonicTimeoutUS);

  if (duration == 0) {
    return maxValidDistance;
  }

  int distance = duration / 58;

  if (distance <= 0 || distance > maxValidDistance) {
    distance = maxValidDistance;
  }

  return distance;
}

int readDistanceWideCM(byte samples)
{
  int bestDistance = 0;

  for (byte i = 0; i < samples; i++) {
    int d = readDistanceCM();
    if (d > bestDistance) {
      bestDistance = d;
    }
    delay(12);
  }

  if (bestDistance <= 0) {
    bestDistance = maxValidDistance;
  }

  return bestDistance;
}

bool isObstacleWithLimit(int distance, int dangerDistance)
{
  return distance > 2 && distance <= dangerDistance;
}

bool isObstacle(int distance)
{
  return isObstacleWithLimit(distance, frontDangerDistance);
}

//********************************************************************
// Enable pin output
//********************************************************************

bool isPwmPin(int pin)
{
  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
}

void writeEnable(int pin, int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  if (isPwmPin(pin)) {
    analogWrite(pin, speedValue);
  }
  else {
    if (speedValue > 0) {
      digitalWrite(pin, HIGH);
    }
    else {
      digitalWrite(pin, LOW);
    }
  }
}

int trimmedSpeed(int baseValue, int trimValue)
{
  return constrain(baseValue + trimValue, 0, 255);
}

//********************************************************************
// Motor control
//
// The forward/backward HIGH-LOW logic has been kept from your tested version.
//********************************************************************

void rightMotorForward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin1, speedValue);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
}

void rightMotorBackward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin1, speedValue);

  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
}

void rightMotorStop()
{
  writeEnable(enablePin1, 0);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}

void leftMotorForward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin2, speedValue);

  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
}

void leftMotorBackward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin2, speedValue);

  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
}

void leftMotorStop()
{
  writeEnable(enablePin2, 0);

  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
}

void stopMotors()
{
  rightMotorStop();
  leftMotorStop();
}

//********************************************************************
// Basic movement functions
//********************************************************************

void advance()
{
  int rightSpeed = trimmedSpeed(baseSpeed, rightForwardTrim);
  int leftSpeed = trimmedSpeed(baseSpeed, leftForwardTrim);

  rightMotorForward(rightSpeed);
  leftMotorForward(leftSpeed);
}

void back()
{
  int rightSpeed = trimmedSpeed(backSpeed, rightBackwardTrim);
  int leftSpeed = trimmedSpeed(backSpeed, leftBackwardTrim);

  rightMotorBackward(rightSpeed);
  leftMotorBackward(leftSpeed);
}

void backArcLeft()
{
  // Backing toward the left rear side:
  // right wheel backward faster, left wheel backward slower.
  int rightSpeed = trimmedSpeed(backArcOuterSpeed, rightBackwardTrim);
  int leftSpeed = trimmedSpeed(backArcInnerSpeed, leftBackwardTrim);

  rightMotorBackward(rightSpeed);
  leftMotorBackward(leftSpeed);
}

void backArcRight()
{
  // Backing toward the right rear side:
  // left wheel backward faster, right wheel backward slower.
  int rightSpeed = trimmedSpeed(backArcInnerSpeed, rightBackwardTrim);
  int leftSpeed = trimmedSpeed(backArcOuterSpeed, leftBackwardTrim);

  rightMotorBackward(rightSpeed);
  leftMotorBackward(leftSpeed);
}

void randomPivotLeft()
{
  // In-place left pivot:
  // right wheel forward, left wheel backward.
  rightMotorForward(randomPivotSpeed);
  leftMotorBackward(randomPivotSpeed);
}

void randomPivotRight()
{
  // In-place right pivot:
  // right wheel backward, left wheel forward.
  rightMotorBackward(randomPivotSpeed);
  leftMotorForward(randomPivotSpeed);
}

void turnL()
{
  // Compatibility function from the previous version.
  // Left turn in place: right wheel forward, left wheel backward.
  rightMotorForward(pivotSpeed);
  leftMotorBackward(pivotSpeed);
}

void turnR()
{
  // Compatibility function from the previous version.
  // Right turn in place: right wheel backward, left wheel forward.
  rightMotorBackward(pivotSpeed);
  leftMotorForward(pivotSpeed);
}


