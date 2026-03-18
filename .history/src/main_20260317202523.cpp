/*
 * ESP32 + CNC Shield + EMM42_V5.0 test program (Y axis)
 *
 * Pins:
 * EN   -> GPIO12
 * STEP -> GPIO25
 * DIR  -> GPIO27
 */

#include <Arduino.h>

const int EN_PIN = 12;
const int STEP_PIN = 25;
const int DIR_PIN = 27;

// For MStep=16 with a 1.8deg motor => 3200 pulses/rev.
// If your motor is 0.9deg with MStep=16, change this to 6400.
const int PULSES_PER_REV = 3200;
const int MIN_STEP_DELAY_US = 20;
const int MAX_STEP_DELAY_US = 2000;
const bool RUN_STARTUP_TEST = true;

int currentStepDelayUs = 50;
String inputBuffer;

void processCommand(String cmd);
void rotateMotor(bool direction, int pulses, int stepDelayUs);
void runTest();

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);   // active-low enable
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("=== ESP32 EMM42 Controller Ready ===");
  Serial.println("Commands:");
  Serial.println("Fxxx: forward xxx pulses, e.g. F3200");
  Serial.println("Rxxx: reverse xxx pulses, e.g. R3200");
  Serial.println("Sxx : set pulse delay(us), e.g. S50");
  Serial.println("T   : run test");
  Serial.println("E   : enable motor");
  Serial.println("D   : disable motor");

  if (RUN_STARTUP_TEST) {
    delay(800);
    Serial.println("Startup test -> FWD 200 pulses");
    rotateMotor(true, 200, currentStepDelayUs);
    delay(300);
    Serial.println("Startup test -> REV 200 pulses");
    rotateMotor(false, 200, currentStepDelayUs);
  }
}

void loop() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (ch != '\r') {
      inputBuffer += ch;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  const char action = cmd.charAt(0);
  int value = 0;
  if (cmd.length() > 1) {
    value = cmd.substring(1).toInt();
  }

  switch (action) {
    case 'F':
    case 'f':
      if (value > 0) {
        rotateMotor(true, value, currentStepDelayUs);
        Serial.println("Done");
      } else {
        Serial.println("Invalid value. Example: F3200");
      }
      break;

    case 'R':
    case 'r':
      if (value > 0) {
        rotateMotor(false, value, currentStepDelayUs);
        Serial.println("Done");
      } else {
        Serial.println("Invalid value. Example: R3200");
      }
      break;

    case 'S':
    case 's':
      if (value >= MIN_STEP_DELAY_US && value <= MAX_STEP_DELAY_US) {
        currentStepDelayUs = value;
        Serial.print("Step delay set to: ");
        Serial.print(currentStepDelayUs);
        Serial.println(" us");
      } else {
        Serial.print("Range: ");
        Serial.print(MIN_STEP_DELAY_US);
        Serial.print("~");
        Serial.print(MAX_STEP_DELAY_US);
        Serial.println(" us");
      }
      break;

    case 'E':
    case 'e':
      digitalWrite(EN_PIN, LOW);
      Serial.println("Motor enabled");
      break;

    case 'D':
    case 'd':
      digitalWrite(EN_PIN, HIGH);
      Serial.println("Motor disabled");
      break;

    case 'T':
    case 't':
      runTest();
      break;

    default:
      Serial.println("Unknown command");
      break;
  }
}

void rotateMotor(bool direction, int pulses, int stepDelayUs) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delayMicroseconds(10);

  for (int i = 0; i < pulses; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelayUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
  }
}

void runTest() {
  Serial.println("Test: forward 1 rev");
  rotateMotor(true, PULSES_PER_REV, currentStepDelayUs);
  delay(500);

  Serial.println("Test: reverse 1 rev");
  rotateMotor(false, PULSES_PER_REV, currentStepDelayUs);
  delay(500);
}
