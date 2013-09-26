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

#define PITCH_MAX 40
#define ROL_MAX 40
#define YAV_MAX 20

#define THROTTLE_MAX 1500


class Motors {
public:
  Motors();

  // desired pitch, rol, yaw and throttle
  float pitch;
  float rol;
  float yaw;
  float throttle;

  bool armed;
  bool motorOn;
  Servo motor[MOTOR_MAX];
  uint32_t motor_throttle[MOTOR_MAX];

  void print();

  void arm();
  void disarm();
  void updateMotors();

};

#endif /* MOTORS_H_ */
