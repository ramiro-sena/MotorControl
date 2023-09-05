// MotorControl.cpp
#include <Arduino.h>
#include <MotorControl.h>
#include <PID_v1.h>

MotorControl* MotorControl::instance = 0;

MotorControl::MotorControl(
  int motor_pin_a,
  int motor_pin_b,
  int encoder_pin
)
{
  _pinA = motor_pin_a;
  _pinB = motor_pin_b;
  _encoderPin = encoder_pin;
  MotorPosition = 0;
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  pinMode(_encoderPin, INPUT_PULLUP);
  instance = this;
  attachInterrupt(digitalPinToInterrupt(_encoderPin), MotorControl::_static_increment, CHANGE);
}

void MotorControl::increment(){
  MotorPosition++;
}

void MotorControl::handleControl(){
  
}

void MotorControl::calculateSpeed(){

}

void MotorControl::spin(){
}

void MotorControl::_static_increment(){
  if (instance != 0) {
    instance->increment();
  }
}

