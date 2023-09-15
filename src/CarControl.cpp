#include <Arduino.h>
#include <MotorControl.h>
#include <CarControl.h>
#include <QuickPID.h>

  CarControl::CarControl(
      int LM_PIN_1,
      int LM_PIN_2,
      int LM_PIN_3,
      int RM_PIN_1,
      int RM_PIN_2,
      int RM_PIN_3
  ) : 
    LeftMotor(LM_PIN_1, LM_PIN_2, LM_PIN_3),
    RightMotor(RM_PIN_1, RM_PIN_2, RM_PIN_3),
    steerPID(&Input, &Output, &Setpoint)
  {
    for (int i; i < queueSize; i++) {
      commandQueue[i] = 0;
      modeQueue[i] = 0;
    }
    steerPID.SetTunings(8, 8, 1);
    steerPID.SetControllerDirection(0);
    steerPID.SetMode(1);
  }


  void CarControl::queueCommand(int mode, float value) {
    for (int i; i < queueSize; i++) {
    if (commandQueue[i] == 0.00 && value != 0.00) {
      commandQueue[i] = value;
      modeQueue[i] = mode;
      break;
    }
  }
  }

  void CarControl::run(float speed_rpm) {
    LeftMotor.spin(speed_rpm);
    RightMotor.spin(speed_rpm);
  }

  void CarControl::move(int distance) {
    queueCommand(MOVE_MODE, distance);
  }
  
  void CarControl::rotate(int degrees) {
    queueCommand(ROTATE_MODE, degrees);
  }

  void CarControl::startCommand(int mode, float value)
  {

    for (int i = 0; i < queueSize - 1; i++) {
      commandQueue[i] = commandQueue[i + 1];
      modeQueue[i] = modeQueue[i + 1];
    }

    if (mode == MOVE_MODE) {
      // LeftMotor.move(double(value) * cm_to_pulse);
      // RightMotor.move(double(value) * cm_to_pulse);
      LeftMotor.move(double(value) * cm_to_pulse );
      RightMotor.move(double(value) * cm_to_pulse );
    }

    if (mode == ROTATE_MODE) {
        double rate = 0.182;
        LeftMotor.move(value * rate);
        RightMotor.move(-value * rate);
    }
  }

  void CarControl::stop() {
    LeftMotor.stop();
    RightMotor.stop();
  } 

  void CarControl::loop() {
    LeftMotor.handleControl();
    RightMotor.handleControl();

    //get speed
    leftMotorSpeed = LeftMotor.Speed;
    rightMotorSpeed = RightMotor.Speed;

    //handle steer
    int diff = LeftMotor.MotorPosition - RightMotor.MotorPosition;
    Input = diff;
    Setpoint = 0;
    steerPID.Compute();
    
    if (LeftMotor._isMoving && RightMotor._isMoving) {
      RightMotor.steer = -Output;
      LeftMotor.steer = Output;
    }

    if (!LeftMotor._isMoving
        && !RightMotor._isMoving) {
          if( commandQueue[0] != 0){
            LeftMotor.stop();
            RightMotor.stop();
            steerPID.Reset();
            float value = commandQueue[0];
            int mode = modeQueue[0];
            delay(300);
            startCommand(mode, value);
            // Serial.println("starting " + String(value));
          } else {
                for (int i = 0; i < queueSize - 1; i++) {
                  commandQueue[i] = commandQueue[i + 1];
                  modeQueue[i] = modeQueue[i + 1];
                }
          }
    }
  }
