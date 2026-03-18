#include <Arduino.h>

const int STEP_PIN = 25;   // Y_STEP
const int DIR_PIN  = 27;   // Y_DIR

const int STEPS_PER_REV = 6400;   // 0.9° + 16细分
const int PULSE_DELAY_US = 1500;  // 先慢一点

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
  delay(1000);   // 给上电稳定一点时间

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("ESP32 + CNC Shield + EMM42 (Y轴) 启动");
}

void loop() {
  Serial.println("Y轴 正转...");
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(50);
  stepMotor(800);     // 先不要打一整圈，先小步测试
  delay(2000);

  Serial.println("Y轴 反转...");
  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(50);
  stepMotor(800);
  delay(3000);
}