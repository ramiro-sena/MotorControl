#include <Arduino.h>
#include <MotorControl.h>

MotorControl MotorEsq(5, 6, 2);
MotorControl MotorDir(9, 10, 3);

void setup() {
  Serial.begin(115200);
  MotorEsq.move(100);
  MotorDir.move(100);
}

float cm_to_pulse = 1.85;
int distance = 30 * cm_to_pulse;


void loop() {

  MotorEsq.handleControl();
  MotorDir.handleControl();
  Serial.println("0,150,"+ String(MotorEsq.Setpoint)+ "," + String(MotorDir.Setpoint));
}
