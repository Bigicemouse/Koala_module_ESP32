#include <Arduino.h>

const int EN_PIN = 12;
const int Y_STEP_PIN = 25;
const int Y_DIR_PIN = 27;

// Ultra-slow diagnostic parameters
const int STEP_HIGH_US = 3000;
const int STEP_LOW_US = 3000;
const int TEST_PULSES = 200;
const int BETWEEN_MOVE_MS = 1200;
const int BETWEEN_PHASE_MS = 2500;

void pulseY(bool dir, int pulses) {
  digitalWrite(Y_DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(20);

  for (int i = 0; i < pulses; i++) {
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(STEP_LOW_US);
  }
}

void testWithEnableLevel(bool enLevel) {
  digitalWrite(EN_PIN, enLevel ? HIGH : LOW);

  Serial.println();
  Serial.print("[PHASE] EN=");
  Serial.print(enLevel ? "HIGH" : "LOW");
  Serial.println(" (watch if motor has holding torque)");

  Serial.println("  Forward 200 pulses");
  pulseY(true, TEST_PULSES);
  delay(BETWEEN_MOVE_MS);

  Serial.println("  Reverse 200 pulses");
  pulseY(false, TEST_PULSES);
  delay(BETWEEN_MOVE_MS);

  Serial.println("  Hold test 2s...");
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(EN_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(EN_PIN, LOW);

  Serial.println();
  Serial.println("===== EMM42 Y-Axis Slow Diagnostic =====");
  Serial.println("Only Y axis is tested: STEP=25 DIR=27 EN=12");
  Serial.println("Program will test EN=LOW phase, then EN=HIGH phase.");
  Serial.println("Watch motor torque and movement in each phase.");
  Serial.println("=====================================");
}

void loop() {
  testWithEnableLevel(false);
  delay(BETWEEN_PHASE_MS);

  testWithEnableLevel(true);
  delay(BETWEEN_PHASE_MS);

  Serial.println("[LOOP] Repeat diagnostics...");
}
