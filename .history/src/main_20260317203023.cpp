#include <Arduino.h>

struct Axis {
  const char* name;
  int stepPin;
  int dirPin;
};

// CNC Shield on ESP32 D1 R32 (common mapping)
const int EN_PIN = 12;
Axis axes[] = {
  {"X", 26, 16},
  {"Y", 25, 27},
  {"Z", 17, 14},
};

const int AXIS_COUNT = sizeof(axes) / sizeof(axes[0]);

// Slow enough for visual/mechanical confirmation
const int STEP_DELAY_US = 800;
const int TEST_PULSES = 400;
const int GAP_MS = 600;

void pulseAxis(const Axis& axis, bool dir, int pulses, int stepDelayUs) {
  digitalWrite(axis.dirPin, dir ? HIGH : LOW);
  delayMicroseconds(20);

  for (int i = 0; i < pulses; i++) {
    digitalWrite(axis.stepPin, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(axis.stepPin, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

void testOneAxis(const Axis& axis) {
  Serial.print("[TEST] Axis ");
  Serial.print(axis.name);
  Serial.print(" pins STEP=");
  Serial.print(axis.stepPin);
  Serial.print(" DIR=");
  Serial.println(axis.dirPin);

  Serial.println("  Forward 400 pulses");
  pulseAxis(axis, true, TEST_PULSES, STEP_DELAY_US);
  delay(GAP_MS);

  Serial.println("  Reverse 400 pulses");
  pulseAxis(axis, false, TEST_PULSES, STEP_DELAY_US);
  delay(GAP_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // Active-low enable

  for (int i = 0; i < AXIS_COUNT; i++) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    digitalWrite(axes[i].stepPin, LOW);
    digitalWrite(axes[i].dirPin, LOW);
  }

  Serial.println();
  Serial.println("===== EMM42 Hardware Diagnostic =====");
  Serial.println("EN forced LOW (enabled)");
  Serial.println("Will test X/Y/Z axis pins in sequence forever.");
  Serial.println("If motor never moves on all 3 tests, issue is likely wiring/power/driver mode.");
  Serial.println("=====================================");
}

void loop() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    testOneAxis(axes[i]);
  }

  Serial.println("[LOOP] Full scan done. Repeat in 2s...");
  delay(2000);
}
