#include <Arduino.h>

// Minimal diagnostic firmware
// - Fixed EN = LOW (active)
// - Y axis only
// - Constant 20 steps per second

const int EN_PIN = 12;
const int Y_STEP_PIN = 25;
const int Y_DIR_PIN = 27;

const unsigned long STEP_INTERVAL_MS = 50;  // 20 steps/s
const int PULSE_HIGH_US = 1000;
const unsigned long DIRECTION_HOLD_MS = 3000;

bool directionForward = true;
unsigned long lastStepMs = 0;
unsigned long lastDirectionToggleMs = 0;

void doOneStep() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(Y_STEP_PIN, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Fixed enable, no switching
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(Y_DIR_PIN, HIGH);  // Start forward

  Serial.println("=== Minimal Y-axis test ===");
  Serial.println("EN fixed LOW");
  Serial.println("Y STEP=25, DIR=27");
  Serial.println("Speed: 20 steps/s");
  Serial.println("Direction toggles every 3s");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastDirectionToggleMs >= DIRECTION_HOLD_MS) {
    directionForward = !directionForward;
    digitalWrite(Y_DIR_PIN, directionForward ? HIGH : LOW);
    lastDirectionToggleMs = now;

    Serial.print("DIR=");
    Serial.println(directionForward ? "FWD" : "REV");
  }

  if (now - lastStepMs >= STEP_INTERVAL_MS) {
    doOneStep();
    lastStepMs = now;
  }
}
