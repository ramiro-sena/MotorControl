// CarControl.h
#ifndef CarControl_h
#define CarControl_h

#include "Arduino.h"
#include <MotorControl.h>
#include <QuickPID.h>

class CarControl
{
  public:
    CarControl(
      int LM_PIN_1,
      int LM_PIN_2,
      int LM_PIN_3,
      int RM_PIN_1,
      int RM_PIN_2,
      int RM_PIN_3
    );

    void run(float speed);
    void move(int distance);
    void rotate(int degrees);
    void stop();

    void startCommand(int mode, float value);

    void loop();
    float commandQueue[50];
    int modeQueue[50];
    float leftMotorSpeed = 0;
    float lastLeftMotorSpeed = 0;
    float rightMotorSpeed = 0;
    float lastRightMotorSpeed = 0;
    MotorControl LeftMotor;
    MotorControl RightMotor;
    QuickPID steerPID;

  private:
    float Setpoint;
    float Input;
    float Output;

    void queueCommand(int mode, float value);

    double cm_to_pulse = 1.8;

    const int queueSize = 50;

    boolean _isMoving = false;

    const int RUN_MODE = 0;
    const int MOVE_MODE = 1;
    const int ROTATE_MODE = 2;
    const int STOP = 3;

    int Mode = 3;
};

#endif
