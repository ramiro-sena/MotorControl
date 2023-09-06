// #include <Arduino.h>
// #include <MotorControl.h>

// MotorControl MotorEsq(5, 6, 2);
// MotorControl MotorDir(9, 10, 3);

// float cm_to_pulse = 1.85;
// int distance = 30 * cm_to_pulse;

// void printAll() {
//   Serial.print("[");
//   for (unsigned int i; i < 20; i++) {
//     Serial.print(String(MotorDir._queue[i]) + ",");
//   }
//   Serial.println("]"); 
// }

// void setup() {
//   Serial.begin(115200);
  
//   printAll();
//   MotorDir.move(21.5 * cm_to_pulse);
//   MotorDir.move(-21.5 * cm_to_pulse);
//   // MotorDir.spin(50);
//   printAll();
// }





// void loop() {

//   MotorEsq.handleControl();
//   MotorDir.handleControl();



//   // Serial.println("0,150,"+ String(MotorEsq.Setpoint)+ "," + String(MotorDir.Setpoint));
// }
