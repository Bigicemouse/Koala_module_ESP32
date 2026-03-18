/*
 * ESP32 + CNC Shield + EMM42_V5.0 闭环驱动测试代码
 *
 * 硬件：ESP32 D1 R32 + CNC Shield V3 + EMM42_V5.0
 * 接线（Y轴）：
 * - EN_PIN   -> GPIO 12
 * - STEP_PIN -> GPIO 25
 * - DIR_PIN  -> GPIO 27
 */

#include <Arduino.h>

// ========== 引脚定义 ==========
const int EN_PIN = 12;
const int STEP_PIN = 25;
const int DIR_PIN = 27;

// ========== 电机参数 ==========
const int PULSES_PER_REV = 3200;  // 1.8度电机 + 16细分
const int MIN_STEP_DELAY_US = 20;
const int MAX_STEP_DELAY_US = 2000;

// ========== 调试开关 ==========
const bool EN_ACTIVE_LOW = true;      // En=L 时为 true；En=H 时改为 false
const bool STARTUP_SELF_TEST = true;  // 上电自动跑一次测试

// 运行时速度参数（Sxx 可修改）
int currentStepDelayUs = 50;

// ========== 串口命令缓存 ==========
String inputString = "";
bool stringComplete = false;

// ========== 函数声明 ==========
void processCommand(String cmd);
void rotateMotor(bool direction, int pulses, int stepDelayUs);
void runTest();
void setMotorEnable(bool enabled);

void setup() {
  Serial.begin(115200);
  Serial.println("=================================");
  Serial.println("ESP32 + EMM42 闭环驱动控制器");
  Serial.println("=================================");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  setMotorEnable(true);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("电机已使能");
  Serial.println("可用命令:");
  Serial.println("  Fxxx - 正转xxx步 (例: F6400)");
  Serial.println("  Rxxx - 反转xxx步 (例: R6400)");
  Serial.println("  Sxx  - 设置脉冲延迟xxus (例: S50)");
  Serial.println("  E    - 使能电机");
  Serial.println("  D    - 禁用电机");
  Serial.println("  T    - 测试正反转");
  Serial.println("=================================");

  if (STARTUP_SELF_TEST) {
    Serial.println("上电自检: 2 秒后开始测试");
    delay(2000);
    runTest();
  }
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char action = cmd.charAt(0);
  int value = 0;
  if (cmd.length() > 1) {
    value = cmd.substring(1).toInt();
  }

  switch (action) {
    case 'F':
    case 'f':
      if (value > 0) {
        Serial.print("正转 ");
        Serial.print(value);
        Serial.println(" 步");
        rotateMotor(true, value, currentStepDelayUs);
        Serial.println("完成");
      } else {
        Serial.println("参数错误, 示例: F6400");
      }
      break;

    case 'R':
    case 'r':
      if (value > 0) {
        Serial.print("反转 ");
        Serial.print(value);
        Serial.println(" 步");
        rotateMotor(false, value, currentStepDelayUs);
        Serial.println("完成");
      } else {
        Serial.println("参数错误, 示例: R6400");
      }
      break;

    case 'S':
    case 's':
      if (value >= MIN_STEP_DELAY_US && value <= MAX_STEP_DELAY_US) {
        currentStepDelayUs = value;
        Serial.print("速度延迟设置为: ");
        Serial.print(currentStepDelayUs);
        Serial.println(" us");
      } else {
        Serial.print("S 命令范围: ");
        Serial.print(MIN_STEP_DELAY_US);
        Serial.print("~");
        Serial.print(MAX_STEP_DELAY_US);
        Serial.println(" us");
      }
      break;

    case 'E':
    case 'e':
      setMotorEnable(true);
      Serial.println("电机已使能");
      break;

    case 'D':
    case 'd':
      setMotorEnable(false);
      Serial.println("电机已禁用");
      break;

    case 'T':
    case 't':
      runTest();
      break;

    default:
      Serial.println("未知命令");
      break;
  }
}

void setMotorEnable(bool enabled) {
  bool level = EN_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(EN_PIN, level ? HIGH : LOW);
}

void rotateMotor(bool direction, int pulses, int stepDelayUs) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delayMicroseconds(10);  // 方向建立时间

  for (int i = 0; i < pulses; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

void runTest() {
  Serial.println("=== 开始测试 ===");

  Serial.println("正转 1 圈...");
  rotateMotor(true, PULSES_PER_REV, currentStepDelayUs);
  delay(500);

  Serial.println("反转 1 圈...");
  rotateMotor(false, PULSES_PER_REV, currentStepDelayUs);
  delay(500);

  Serial.println("正转 2 圈...");
  rotateMotor(true, PULSES_PER_REV * 2, currentStepDelayUs);
  delay(500);

  Serial.println("反转 2 圈...");
  rotateMotor(false, PULSES_PER_REV * 2, currentStepDelayUs);

  Serial.println("=== 测试完成 ===");
}
