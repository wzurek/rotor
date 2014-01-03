/*
 * motors.h
 *
 *  Created on: Aug 23, 2013
 *      Author: wzurek
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <Arduino.h>
#include <Servo.h>
#include "motors.h"

#define MOTOR_MAX 4

// Motor indexes
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define REAR_LEFT 2
#define REAR_RIGHT 3

#define PITCH_MAX 50
#define ROL_MAX 50
#define YAV_MAX 20

#define THROTTLE_MAX 1500

#define MOTOR_MODE_SAFE_START 0
#define MOTOR_MODE_OFF 1
#define MOTOR_MODE_STOPPED 2
#define MOTOR_MODE_OPERATION 3
#define MOTOR_MODE_DIRECT_COMMAND 4

class Motors {
private:
  // mode of operation
  uint32_t motor_mode;

  // use each motor as servo
  Servo motor[MOTOR_MAX];

  void updateThrottleForOperationMode();

public:
  Motors();

  // desired pitch, rol, yaw and throttle, only relevant in operation mode
  float pitch;
  float rol;
  float yaw;
  float throttle;

  // motor mode, see MOTOR_MODE_* constants.
  inline uint32_t get_mode() {
    return motor_mode;
  }

  // move to operation mode;
  inline void operation() {
    if (motor_mode <= MOTOR_MODE_OFF) {
      return;
    }
    motor_mode = MOTOR_MODE_OPERATION;
  }

  // move to operation direct;
  void direct();

  // stop the motors
  inline void stop() {
    if (motor_mode <= MOTOR_MODE_OFF) {
      return;
    }
    motor_mode = MOTOR_MODE_STOPPED;
  }

  // move the motor from SAFE_START to OFF, or no-op
  void init();

  // true, if the motors are armed
  inline bool isArmed() {
    return motor_mode > MOTOR_MODE_OFF;
  };


  // speed of each motor, can be set externally only in direct mode
  uint32_t motor_throttle[MOTOR_MAX];

  // send the motors status to ground station
  void print();

  // arm the motors
  void arm();

  // update the motors with the commands
  void updateMotors();

};

#endif /* MOTORS_H_ */
