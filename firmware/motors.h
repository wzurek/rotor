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

#define MOTOR_MAX 4

// Motor indexes
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define REAR_RIGHT 2
#define REAR_LEFT 3

#define PITCH_MAX 10
#define ROL_MAX 10
#define YAV_MAX 10


class Motors {
public:
  // desired pitch, rol, yaw and throttle
  float pitch;
  float rol;
  float yaw;
  float throttle;

  bool armed;
  Servo motor[MOTOR_MAX];
  uint32_t motor_throttle[MOTOR_MAX];

  void print();

  void arm();
  void updateMotors();

};

#endif /* MOTORS_H_ */
