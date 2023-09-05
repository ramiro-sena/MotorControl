// MotorControl.h
#ifndef MotorLib_h
#define MotorLib_h

#include "Arduino.h"
#include <PID_v1.h>

class MotorControl
{
  public:
    MotorControl(
      int motor_pin_a,
      int motor_pin_b,
      int encoder_pin
    );

    double Setpoint;
    double Input;
    double Output;

    int MotorPosition;
    double MotorSpeed;
    
    int Mode;

    void move();
    void spin(float speed);
    void stop();
    void increment();
    void handleControl();
  private:
    PID myPID;
    void calculateSpeed();

    int _last_state;

    long lastCalc;
    long _CurrentPulse;
    long _LastPulse;

    double Kp;
    double Ki;
    double Kd;

    int _pinA;
    int _pinB;
    int _encoderPin;
    void _handleSensor();
    // static MotorControl* instance0;
    // static MotorControl* instance1;
};

#endif