#include <Arduino.h>

// 你可能用到的引脚列表（全部试一遍）
int pins[] = {12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};

void setup() {
  Serial.begin(115200);
  Serial.println("STEP pin scan start");

  for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
    Serial.print("Testing pin: ");
    Serial.println(pins[i]);

    for (int j = 0; j < 200; j++) {
      digitalWrite(pins[i], HIGH);
      delayMicroseconds(2000);
      digitalWrite(pins[i], LOW);
      delayMicroseconds(2000);
    }

    delay(1000);
  }
}