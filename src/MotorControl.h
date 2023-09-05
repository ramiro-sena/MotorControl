/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#ifndef MotorLib_h
#define MotorLib_h

#include "Arduino.h"

class MotorControl
{
  public:
    MotorControl(
      int motor_pin_a,
      int motor_pin_b,
      int encoder_pin_a
    );

    double Setpoint;
    double Input;
    double Output;

    int MotorPosition;
    double MotorSpeed;
    
    long LastPulse;
    long CurrentPulse;

    void move();
    void spin();
    void stop();
    void increment();
    void handleControl();
  private:

    void calculateSpeed();

    double Kp;
    double K1;
    double Kd;

    int _pinA;
    int _pinB;
    int _encoderPin;
    int _encoderPosition;
    static MotorControl* instance;
    static void _static_increment();
};

#endif