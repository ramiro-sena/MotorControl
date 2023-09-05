#include <Arduino.h>
#include <MotorControl.h>

MotorControl MotorEsq(5, 6, 2);
MotorControl MotorDir(9, 10, 3);

void setup() {
  Serial.begin(115200);
  MotorEsq.spin(50.000);
  MotorDir.spin(50.000);
}

float cm_to_pulse = 1.85;
int distance = 30 * cm_to_pulse;


void loop() {
  if(MotorEsq.MotorPosition >= distance) {
    MotorEsq.stop();
  }
  if(MotorDir.MotorPosition >= distance) {
    MotorDir.stop();
  }
  MotorEsq.handleControl();
  MotorDir.handleControl();
  Serial.println("0,150,"+ String(MotorEsq.MotorPosition)+ "," + String(MotorDir.MotorPosition));
}
