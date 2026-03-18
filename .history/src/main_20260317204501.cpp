#include <Arduino.h>

// STEP signal verifier firmware
// - Fixed EN = LOW (active)
// - Fixed DIR = HIGH
// - Continuous STEP pulses on Y axis

const int EN_PIN = 12;
const int Y_STEP_PIN = 25;
const int Y_DIR_PIN = 27;

const int STEP_HIGH_US = 1000;
const int STEP_LOW_US = 1000;  // 500 Hz square-wave equivalent on STEP pulses

unsigned long pulseCounter = 0;
unsigned long lastReportMs = 0;

void doOneStep() {
  digitalWrite(Y_STEP_PIN, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(Y_STEP_PIN, LOW);
  delayMicroseconds(STEP_LOW_US);
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);      // Fixed enable, no switching
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(Y_DIR_PIN, HIGH);  // Fixed direction

  Serial.println("=== Y STEP Signal Verifier ===");
  Serial.println("EN fixed LOW");
  Serial.println("DIR fixed HIGH");
  Serial.println("Y STEP=25, DIR=27");
  Serial.println("Output: continuous STEP pulses");
  Serial.println("Check STEP pin with meter/scope/logic-probe");
}

void loop() {
  doOneStep();
  pulseCounter++;

  const unsigned long now = millis();
  if (now - lastReportMs >= 1000) {
    Serial.print("Pulses/s ~= ");
    Serial.println(pulseCounter);
    pulseCounter = 0;
    lastReportMs = now;
  }
}
