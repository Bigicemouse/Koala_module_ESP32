#include <Arduino.h>

const int STEP_PIN = 17;   // Z_STEP
const int DIR_PIN  = 14;   // Z_DIR

const int PULSE_DELAY_US = 1500;   // 先慢一点
const int STEPS_TEST = 400;        // 先不要一上来打一整圈

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

  Serial.println("Z axis test start");
}

void loop() {
  Serial.println("Z forward...");
  digitalWrite(DIR_PIN, HIGH);
  delayMicroseconds(50);
  stepMotor(STEPS_TEST);
  delay(2000);

  Serial.println("Z reverse...");
  digitalWrite(DIR_PIN, LOW);
  delayMicroseconds(50);
  stepMotor(STEPS_TEST);
  delay(3000);
}