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
) : myPID(&Input, &Output, &Setpoint, 1.6, 5, 0.03, DIRECT)
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
  _last_state = false;
  Mode = STOP_MODE;

  // int queue[10];
  // _queue[10] = queue;
  for (int i; i < _queueSize; i++) {
    _queue[i] = 0;
  }

  for (int i; i < sampleSize; i++) {
    smoothSpeed[i] = 0;
  }
  // spin(100);
  myPID.SetMode(AUTOMATIC);
}


void MotorControl::_handleSensor() {
  // int value = (PIND & (1 << PD2)) >> PD2;
  boolean value = digitalRead(_encoderPin);
  _CurrentPulse = millis();
  long pulseInterval = _CurrentPulse - _LastPulse;

  if (pulseInterval > 200) {
    MotorSpeed = 0.00;
    for (int i; i < sampleSize; i++) {
      smoothSpeed[i] = 0;
    }
    Speed = 0;
  }

  if (value != _last_state) {
    MotorPosition++;
    if ( pulseInterval != 0 ) {
      MotorSpeed = double(1000) * double(60.000 / 40.000) / double(pulseInterval); 
    }
    _LastPulse = _CurrentPulse;
    _last_state = value;

    smoothSpeed[sampleIndex] = MotorSpeed;
    sampleIndex ++;
    if (sampleIndex == sampleSize) {
      sampleIndex = 0;
    }
    double sum = 0;
    for (int i; i < sampleSize; i++){
      sum += smoothSpeed[i];
    }
    Speed = sum / sampleSize;
  }
}

void MotorControl::move(int dist){
  // Mode = POSITION_MODE;
  distance = dist;
  
  // for (int i; i < _queueSize; i++) {
  //   if (_queue[i] == 0) {
  //     _queue[i] = dist;
  //     break;
  //   }
  // }
}

void MotorControl::_runSpeed(float speed){
Setpoint = speed;
    Input = Speed;
    myPID.Compute();
    double output_value = min(255, max(0, Output));
    output_value = map(output_value, 0, 255, 20, 255);
    if (direction > 0) {
      analogWrite(_pinA, abs(output_value));
      analogWrite(_pinB, 0);
    } else {
      analogWrite(_pinA, 0);      
      analogWrite(_pinB, abs(output_value));
    }
}

void MotorControl::handleControl(){
  _handleSensor();
  // calculateSpeed();
  // handleQueue();

  if (Mode == STOP_MODE) {
    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 0);
  }

  if (Mode == SPEED_MODE) {
    _runSpeed(Setpoint);
  }

  if (Mode == POSITION_MODE) {
    distance = abs(distance);
    if (MotorPosition < 10 && distance - MotorPosition > 20) {
      _runSpeed(moveSpeed + steer);
    } else if (distance - MotorPosition > 20) {
      _runSpeed(moveSpeed + steer);
    } else if (distance - MotorPosition <= 20 && distance - MotorPosition > 0) {
      _runSpeed(moveSpeed + steer);
    } else {

      // Serial.println("Done");
      Mode = STOP_MODE;
      _isMoving = false;
      distance = 0;
      MotorPosition = 0;
      for (int i = 0; i < _queueSize - 1; i++) {
        _queue[i] = _queue[i + 1];
      }
    }
  }
}

void MotorControl::handleQueue(){
  if ( _queue[0] != 0 && !_isMoving) {
    _isMoving = true;
    Mode = POSITION_MODE;
    distance = _queue[0];

    if (distance > 0) {
      direction = 1;
    } else if (distance < 0) {
      direction = -1;
    }
    // Serial.println("starting " + String(distance));
  }
}

void MotorControl::spin(float speed){
  Mode = SPEED_MODE;
  // Input = MotorSpeed;
  if (speed > 0) {
    direction = 1;
  } else if (speed < 0) {
    direction = -1;
  }
  Setpoint = abs(speed);
}

void MotorControl::stop() {
  Mode = STOP_MODE;
  MotorPosition = 0;
  _isMoving = false;
  for (int i; i < _queueSize; i++) {
    _queue[i] = 0;
  }
}
