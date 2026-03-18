#include <Arduino.h>

// D1 R32 + CNC Shield V3.0（Y轴位）
const int STEP_PIN = 25;   // Y_STEP
const int DIR_PIN  = 27;   // Y_DIR

// 0.9° 电机 + 16细分 = 6400 脉冲/圈
const int STEPS_PER_REV = 6400;

// 先保守一点，别太快
const int PULSE_DELAY_US = 800;

void stepMotor(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(PULSE_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(PULSE_DELAY_US);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("ESP32 + CNC Shield + EMM42 (Y轴) 启动");
  Serial.println("EN 由硬件短接到 GND，代码不控制 EN");
}

void loop() {
  Serial.println("Y轴 正转 1 圈...");
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(20);   // 方向建立时间
  stepMotor(STEPS_PER_REV);
  delay(1000);

  Serial.println("Y轴 反转 1 圈...");
  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(20);
  stepMotor(STEPS_PER_REV);
  delay(2000);
}