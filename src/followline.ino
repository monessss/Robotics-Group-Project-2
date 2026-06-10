/*
 * ============================================================
 *  巡线小车 —— 模拟量加权质心 + PD 控制 (IR 经 ADS1115 读取)
 *
 *  设计思路（和旧版的本质区别）：
 *  1. 不再把传感器压成 0/1，而是用 16 位模拟量算出
 *     “线相对车体中心的连续位置 error (-100 ~ +100)”。
 *  2. 用标准 PD 控制（默认 Ki=0）输出平滑修正量，
 *     抑制左右甩动 / 过冲。
 *  3. 提高 I2C 速度 + ADS 数据率，环路频率更高，跑得更稳更快。
 *  4. 修正量允许超过基速 -> 内轮自动反转，可原地过急弯/直角。
 *  5. 丢线时按最后方向强力回找。
 *
 *  传感器约定：高读数 = 黑线（与旧代码一致）。
 *  通道：A0=左, A1=中, A2=右。
 * ============================================================
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

/* ---------- 调试开关 ----------
 * DEBUG_MODE = 1 -> 电机不转，串口打印原始值/归一化/error，用于标定
 * DEBUG_MODE = 0 -> 正常巡线
 */
#define DEBUG_MODE 0
const unsigned long DEBUG_INTERVAL_MS = 80;
unsigned long lastDebugPrint = 0;

/* ---------- 电机引脚 ---------- */
const int enablePin1 = 6;     // 右电机 PWM (ENA)
const int enablePin2 = 5;    // 左电机 PWM (ENB)
const int in1Pin = 7;         // 右电机方向 1
const int in2Pin = 4;         // 右电机方向 2
const int in3Pin = 9;         // 左电机方向 1
const int in4Pin = 10;         // 左电机方向 2

/* ---------- 预留引脚 ---------- */
const int SERVO_SIG_PIN = 3;
const int ULTRA_D0_PIN  = 7;

/* ============================================================
 *  速度参数（调车主要改这里）
 * ============================================================ */

// 直走基础速度。先调小（50~70）把巡线调稳，再慢慢加快。
const int baseSpeed = 110;

// PWM 上限，防止太快冲出弯道
const int maxPWM = 150;

// 最小启动 PWM：电机太小占空比只嗡嗡不转，给个下限（仅对“需要转的轮”生效）
const int minPWM = 35;

// Force a strong right turn immediately after line loss.
const int lostRightLeftSpeed = 70;
const int lostRightRightSpeed = -105;

// 左右轮机械差异补偿：左轮偏弱就把 leftTrim 调大一点点（±0~15）。
// 注意：直走跑偏优先用这个修，而不是去动 PID。
int leftTrim  = 0;
int rightTrim = 0;

/* ============================================================
 *  PID 参数（核心手感，按下面注释顺序调）
 * ============================================================
 * 调参顺序建议：
 *   1) Ki=0, Kd=0，只调 Kp：从小往大加，直到能跟线但开始左右轻微摆动。
 *   2) 加 Kd 抑制摆动，直到顺滑不甩。Kd 一般是 Kp 的 5~20 倍。
 *   3) 一般 Ki 保持 0；除非长期固定偏一侧才小心加一点点。
 */
float Kp = 0.75f;   // 比例：error 越大转得越急
float Ki = 0.0f;    // 积分：消除稳态偏差（默认关）
float Kd = 8.0f;    // 微分：抑制摆动/过冲

/* ============================================================
 *  传感器标定（务必用 DEBUG_MODE=1 重新标定！）
 * ============================================================
 * sensorWhite[i]：第 i 路在【白色地面】上的读数。
 * 只需要白底基线即可：算质心时分母会自动归一化，
 * 黑线的绝对幅度不用精确知道。
 *
 * 标定方法：
 *   - DEBUG_MODE 设为 1，上传，打开串口 115200。
 *   - 把每个传感器分别停在白地面上，记下 L_raw/M_raw/R_raw。
 *   - 把下面三个值填成“白底读数 + 一点余量(约 +1000~2000)”。
 */
int sensorWhite[3] = { 9600, 2200, 2600 };  // 左 / 中 / 右 白底基线

const int irAdsChannel[3] = { 2, 1, 0 };       // 左, 中, 右

// 三路传感器在车体上的横向位置（用于加权质心）。
// 左=-100, 中=0, 右=+100 -> 算出的 error 也是 -100~+100。
const int sensorPos[3] = { -100, 0, 100 };

// 丢线判定：三路“黑度”之和小于该值，认为脱线。
// 标定后按实际黑线时分母大小设置（通常黑线时分母 > 5000）。
const long LINE_LOST_SUM = 1500;

/* ---------- 运行时状态 ---------- */
int   rawValue[3]  = { 0, 0, 0 };
long  blackness[3] = { 0, 0, 0 };   // max(raw - white, 0)
float error      = 0.0f;            // 当前线位置误差 (-100~100)
float lastError  = 0.0f;
float integral   = 0.0f;
bool  lineLost   = false;

int leftSpeed  = 0;
int rightSpeed = 0;


void setup() {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    // D7 is used by the right motor direction input on this car.
    // pinMode(ULTRA_D0_PIN, INPUT);

    Wire.begin();
    Wire.setClock(400000);                 // I2C 提速到 400kHz，缩短读取耗时
    // 关键：给 I2C 设超时，避免电机噪声让总线死锁、程序卡死“跑一半不动”。
    // 25ms 内没应答就自动放弃本次通信并复位总线，而不是无限死等。
    Wire.setWireTimeout(25000 /*us*/, true /*reset_on_timeout*/);

    ads.setGain(GAIN_TWOTHIRDS);           // ±6.144V 量程（与旧代码一致）
    ads.setDataRate(RATE_ADS1115_860SPS);  // 提高采样率，环路更快
    ads.begin();

#if DEBUG_MODE
    Serial.begin(115200);
    Serial.println(F("=== Line Following DEBUG (analog centroid) ==="));
    Serial.println(F("L_raw\tM_raw\tR_raw\tnL\tnM\tnR\tsum\terror\tL_spd\tR_spd"));
#endif
}


void loop() {
    Scan();             // 读三路模拟量
    ComputeError();     // 算线位置误差（质心）
    PidControl();       // PD -> 左右轮速度

#if DEBUG_MODE
    setMotor(0, 0);     // 调试时不转电机
    DebugPrint();
#else
    Drive(leftSpeed, rightSpeed);
#endif
}


/* ============================================================
 *  Scan(): 读取 ADS1115 三路红外，转成“黑度”
 *  blackness = max(raw - white, 0)，白底约为 0，黑线为正值。
 * ============================================================ */
void Scan() {
    for (int i = 0; i < 3; i++) {
        Wire.clearWireTimeoutFlag();
        int16_t v16 = ads.readADC_SingleEnded(irAdsChannel[i]);

        // 若本次 I2C 通信超时（被电机噪声打断），保留上一帧读数、跳过本路，
        // 让控制环继续跑，而不是整车冻死。
        if (!Wire.getWireTimeoutFlag()) {
            rawValue[i] = v16;
        }
        long v = (long)rawValue[i] - sensorWhite[i];
        blackness[i] = (v > 0) ? v : 0;
    }
}


/* ============================================================
 *  ComputeError(): 加权质心，得到连续的线位置误差
 *  error = Σ(pos[i]*blackness[i]) / Σ(blackness[i])
 *  范围 -100(线在最左) ~ +100(线在最右)，0 为居中。
 * ============================================================ */
void ComputeError() {
    long sum = blackness[0] + blackness[1] + blackness[2];

    if (sum < LINE_LOST_SUM) {
        // 丢线：保持上一次方向，强力回找
        lineLost = true;
        if (lastError > 1.0f)       error =  100.0f;
        else if (lastError < -1.0f) error = -100.0f;
        else                        error =    0.0f;
        return;
    }

    lineLost = false;
    long num = (long)sensorPos[0] * blackness[0]
             + (long)sensorPos[1] * blackness[1]
             + (long)sensorPos[2] * blackness[2];
    error = (float)num / (float)sum;   // 连续位置
}


/* ============================================================
 *  PidControl(): PD(+I) 控制器，输出左右轮速度
 *  error>0 => 线在右 => 需要右转 => 左轮快、右轮慢
 * ============================================================ */
void PidControl() {
    // Highest priority: bypass normal PD while the line is lost.
    if (lineLost) {
        integral = 0.0f;
        lastError = 100.0f;
        leftSpeed = lostRightLeftSpeed;
        rightSpeed = lostRightRightSpeed;
        return;
    }

    float p = error;
    float d = error - lastError;

    // 积分（带抗饱和；丢线时清零防止乱积累）
    if (lineLost || Ki == 0.0f) {
        integral = 0.0f;
    } else {
        integral += error;
        integral = constrain(integral, -200.0f, 200.0f);
    }

    float correction = Kp * p + Ki * integral + Kd * d;

    lastError = error;

    // 修正量允许超过基速 -> 内轮反转，可过急弯
    leftSpeed  = baseSpeed + leftTrim  + (int)correction;
    rightSpeed = baseSpeed + rightTrim - (int)correction;
}


/* ============================================================
 *  Drive(): 限幅 + 方向处理后交给 setMotor
 *  支持负速度（反转），便于原地转向过急弯。
 * ============================================================ */
void Drive(int left, int right) {
    left  = clampSpeed(left);
    right = clampSpeed(right);
    setMotor(left, right);
}

// 限幅：[-maxPWM, maxPWM]，并对“需要转动”的轮施加最小 PWM
int clampSpeed(int v) {
    if (v >  maxPWM) v =  maxPWM;
    if (v < -maxPWM) v = -maxPWM;

    if (v > 0 && v < minPWM) v = minPWM;
    if (v < 0 && v > -minPWM) v = -minPWM;
    return v;
}


/* ============================================================
 *  setMotor(): 实际驱动 L298N（速度带符号，负=反转）
 * ============================================================ */
void setMotor(int leftVal, int rightVal) {
    bool revR = (rightVal < 0);
    bool revL = (leftVal  < 0);
    int rPwm = revR ? -rightVal : rightVal;
    int lPwm = revL ? -leftVal  : leftVal;

    // 右电机
    analogWrite(enablePin1, rPwm);
    digitalWrite(in1Pin, !revR);
    digitalWrite(in2Pin,  revR);

    // 左电机
    analogWrite(enablePin2, lPwm);
    digitalWrite(in3Pin, !revL);
    digitalWrite(in4Pin,  revL);
}


/* ============================================================
 *  DebugPrint(): 标定/调试用
 * ============================================================ */
#if DEBUG_MODE
void DebugPrint() {
    unsigned long now = millis();
    if (now - lastDebugPrint < DEBUG_INTERVAL_MS) return;
    lastDebugPrint = now;

    long sum = blackness[0] + blackness[1] + blackness[2];

    Serial.print(rawValue[0]);  Serial.print('\t');
    Serial.print(rawValue[1]);  Serial.print('\t');
    Serial.print(rawValue[2]);  Serial.print('\t');
    Serial.print(blackness[0]); Serial.print('\t');
    Serial.print(blackness[1]); Serial.print('\t');
    Serial.print(blackness[2]); Serial.print('\t');
    Serial.print(sum);          Serial.print('\t');
    Serial.print(error, 1);     Serial.print('\t');
    Serial.print(leftSpeed);    Serial.print('\t');
    Serial.println(rightSpeed);
}
#endif
