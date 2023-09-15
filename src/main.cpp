#include <Arduino.h>
#include <CarControl.h>

CarControl Car(5,6,2,9,10,3);

// void printIntArray(int* array) {
//   Serial.print("[");
//   for (int i; i < 50; i++) {
//     Serial.print(String(array[i]) + ",");
//   }
//   Serial.println("]"); 
// }

// void printFloatArray(float* array) {
//   Serial.print("[");
//   for (int i; i < 50; i++) {
//     Serial.print(String(array[i]) + ",");
//   }
//   Serial.println("]"); 
// }


void setup() {
  Serial.begin(115200);
  // Serial.println("start");
  Serial.println("Left_speed, Right_speed, B, T");
  Car.move(70);

  // Car.rotate(90);
  // Car.move(30);
  // Car.move(-30);
  // Car.rotate(-90);
  // Car.move(-30);
  
  // printFloatArray(Car.commandQueue);
  // printIntArray(Car.modeQueue);
}

float last_l_speed = 0;
float last_r_speed = 0;


void loop() {
  Car.loop();
  
  if (millis() == 3000) {
    Car.stop();
  }

  if (millis() == 4000) {
    Car.move(70);
  }

  if (millis() == 6000) {
    Car.stop();
  }

  if (Car.leftMotorSpeed != last_l_speed 
    || Car.rightMotorSpeed != last_r_speed )  {
    last_l_speed  = Car.leftMotorSpeed;
    last_r_speed  = Car.rightMotorSpeed;

    Serial.println(
      String(last_l_speed) 
      + ","
      + String(last_r_speed)
      + String(+ ",0,80")
    );
  }
}