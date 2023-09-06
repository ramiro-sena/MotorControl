#include <Arduino.h>
#include <CarControl.h>

CarControl Car(5,6,2,9,10,3);

void printIntArray(int* array) {
  Serial.print("[");
  for (int i; i < 50; i++) {
    Serial.print(String(array[i]) + ",");
  }
  Serial.println("]"); 
}

void printFloatArray(float* array) {
  Serial.print("[");
  for (int i; i < 50; i++) {
    Serial.print(String(array[i]) + ",");
  }
  Serial.println("]"); 
}


void setup() {
  // Serial.begin(115200);
  // Serial.println("start");

  // Car.move(30);
  // Car.move(-30);
  Car.move(30);
  Car.rotate(90);
  
  printFloatArray(Car.commandQueue);
  printIntArray(Car.modeQueue);
}


void loop() {
  Car.loop();
}