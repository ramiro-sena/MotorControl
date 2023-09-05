// MotorControl.cpp
#include <Arduino.h>
#include <MotorControl.h>
#include <PID_v1.h>

#define STOP_MODE 0
#define SPEED_MODE 1
#define POSITION_MODE 2

// MotorControl* MotorControl::instance0 = 0;
// MotorControl* MotorControl::instance1 = 0;


MotorControl::MotorControl(
  int motor_pin_a,
  int motor_pin_b,
  int encoder_pin
) : myPID(&Input, &Output, &Setpoint, 0.5, 5.5, 0.01, DIRECT)
{
  _pinA = motor_pin_a;
  _pinB = motor_pin_b;
  _encoderPin = encoder_pin;
  MotorPosition = 0;
  pinMode(_pinA, OUTPUT);
  pinMode(_pinB, OUTPUT);
  pinMode(_encoderPin, INPUT_PULLUP);
  // instance0 = this;

  lastCalc = 0;
  MotorSpeed = 0;
  _LastPulse = 0;
  _CurrentPulse = 0;
  _last_state = 0;
  Mode = STOP_MODE;

  Kp = 0.5; Ki = 5.5; Kd = 0.01;


  // spin(100);
  myPID.SetMode(AUTOMATIC);
}


void MotorControl::_handleSensor() {
  
  // int value = (PIND & (1 << PD2)) >> PD2;
  boolean value = digitalRead(_encoderPin);
  if (value != _last_state) {
    MotorPosition++;
    _last_state = value;
    _CurrentPulse = millis();
  }
}

void MotorControl::stop(){
  Mode = STOP_MODE;
}

void MotorControl::handleControl(){
  _handleSensor();
  calculateSpeed();
  if (Mode == STOP_MODE) {
    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 0);
  }
  if (Mode == SPEED_MODE) {
    Input = MotorSpeed;
    myPID.Compute();
    if (Setpoint > 0) {
      analogWrite(_pinA, abs(Output));
      analogWrite(_pinB, 0);
    } else {
      analogWrite(_pinA, 0);      
      analogWrite(_pinB, abs(Output));
    }
  }
}

void MotorControl::calculateSpeed(){
  long now = millis();
  
  if(now > lastCalc + 5) {
      long pulseInterval = _CurrentPulse - _LastPulse;
      if (pulseInterval >= 5000) { 
        MotorSpeed = 0.00;
      } else if(pulseInterval == 0.000) {
        if( MotorSpeed > 3 ) {
          MotorSpeed -= 1;
        } else {
          MotorSpeed = 0;
        }
      } else { 
        MotorSpeed = double(1000) * double(60.000 / 40.000) / double(pulseInterval); 
      }
      _LastPulse = _CurrentPulse;
      lastCalc = now;
  }
}

void MotorControl::spin(float speed){
  Mode = SPEED_MODE;
  Input = MotorSpeed;
  Setpoint = speed;
}


