/*
 * ESP32 + CNC Shield + EMM42_V5.0 闭环驱动代码
 * 
 * 配置：MStep = 16，P_Pul = PUL_FOC，En = L
 * 
 * 硬件接线（Y轴）：
 * - EN_PIN  -> GPIO 12 (CNC Shield EN)
 * - STEP_PIN -> GPIO 25 (CNC Shield Y_STEP)  
 * - DIR_PIN  -> GPIO 27 (CNC Shield Y_DIR)
 * 
 * 电机脉冲数配置（根据电机类型选择）：
 * - 1.8° 电机 + 16细分 = 3200 脉冲/圈
 * - 0.9° 电机 + 16细分 = 6400 脉冲/圈
 */
#include <Arduino.h>
// ========== 引脚定义 ==========
const int EN_PIN = 12;    // 使能引脚 - GPIO 12
const int STEP_PIN = 25;  // Y轴脉冲 - GPIO 25
const int DIR_PIN = 27;   // Y轴方向 - GPIO 27

// ========== 电机参数配置 ==========
// 【重要】根据你的电机类型修改这里：
// 1.8° 电机用 3200，0.9° 电机用 6400
const int PULSES_PER_REV = 3200;  // 1.8°电机 + 16细分
// const int PULSES_PER_REV = 6400;  // 0.9°电机 + 16细分

const int STEP_DELAY_US = 50;     // 步进延迟（微秒）- 控制速度

void setup() {
  Serial.begin(115200);
  Serial.println("=================================");
  Serial.println("ESP32 + EMM42 闭环驱动控制器");
  Serial.println("MStep = 16, P_Pul = PUL_FOC");
  Serial.println("=================================");
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  digitalWrite(EN_PIN, LOW);   // 使能电机（低电平有效）
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  Serial.print("脉冲数/圈: ");
  Serial.println(PULSES_PER_REV);
  Serial.println("电机已使能，输入命令开始测试\n");
  
  Serial.println("命令说明：");
  Serial.println("  F    - 正转1圈");
  Serial.println("  R    - 反转1圈");
  Serial.println("  F2   - 正转2圈");
  Serial.println("  R2   - 反转2圈");
  Serial.println("  T    - 自动测试（正反转各1圈）");
  Serial.println("  E    - 使能电机");
  Serial.println("  D    - 禁用电机");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    processCommand(cmd);
  }
}

void processCommand(String cmd) {
  if (cmd.length() == 0) return;
  
  char action = cmd.charAt(0);
  int circles = 1;
  
  // 解析圈数（如 F2 表示2圈）
  if (cmd.length() > 1) {
    circles = cmd.substring(1).toInt();
    if (circles < 1) circles = 1;
  }
  
  int pulses = PULSES_PER_REV * circles;
  
  switch (action) {
    case 'F':
    case 'f':
      Serial.print("正转 ");
      Serial.print(circles);
      Serial.println(" 圈");
      rotateMotor(true, pulses);
      Serial.println("完成\n");
      break;
      
    case 'R':
    case 'r':
      Serial.print("反转 ");
      Serial.print(circles);
      Serial.println(" 圈");
      rotateMotor(false, pulses);
      Serial.println("完成\n");
      break;
      
    case 'T':
    case 't':
      runTest();
      break;
      
    case 'E':
    case 'e':
      digitalWrite(EN_PIN, LOW);
      Serial.println("电机已使能\n");
      break;
      
    case 'D':
    case 'd':
      digitalWrite(EN_PIN, HIGH);
      Serial.println("电机已禁用\n");
      break;
      
    default:
      Serial.println("未知命令\n");
      break;
  }
}

// 电机旋转函数
void rotateMotor(bool direction, int pulses) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delayMicroseconds(10);  // 方向建立时间
  
  for (int i = 0; i < pulses; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// 测试函数
void runTest() {
  Serial.println("\n=== 自动测试开始 ===");
  
  Serial.println("正转 1 圈...");
  rotateMotor(true, PULSES_PER_REV);
  delay(500);
  
  Serial.println("反转 1 圈...");
  rotateMotor(false, PULSES_PER_REV);
  delay(500);
  
  Serial.println("=== 测试完成 ===\n");
}