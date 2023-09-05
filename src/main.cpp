#include <Arduino.h>
#include <MotorControl.h>

MotorControl MotorEsq(9, 10, 3);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(MotorEsq.MotorPosition);
}
