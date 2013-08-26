/*
 * motors.cpp
 *
 *  Created on: Aug 23, 2013
 *      Author: wzurek
 */

#include "motors.h"
#include "receiver.h"

void Motors::arm() {
  // attache ESCs
  motor[0].attach(42);
  motor[1].attach(40);
  motor[2].attach(38);
  motor[3].attach(36);

  // write 'minimum'
  for (int i = 0; i < MOTOR_MAX; i++) {
    motor_throttle[i] = REC_MIN;
    motor[i].writeMicroseconds(motor_throttle[i]);
  }

  // wait for ESCs to boot
  Serial.println("booting speed controllers");
  for (int i = 0; i < 8; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done.");

  armed = true;
}

void Motors::updateMotors() {

  if (!armed) {
    return;
  }

  uint32_t base_throttle = (REC_MIN + REC_RANGE * throttle);

  motor_throttle[FRONT_LEFT] = base_throttle + (pitch * PITCH_MAX)
      + (rol * ROL_MAX);

  motor_throttle[FRONT_RIGHT] = base_throttle + (pitch * PITCH_MAX)
      - (rol * ROL_MAX);

  motor_throttle[REAR_RIGHT] = base_throttle - (pitch * PITCH_MAX)
      - (rol * ROL_MAX);

  motor_throttle[REAR_LEFT] = base_throttle - (pitch * PITCH_MAX)
      + (rol * ROL_MAX);

  Serial.print("!M");
  Serial.print(motor_throttle[0]);
  for (int i = 1; i < MOTOR_MAX; i++) {
    Serial.print(",");
    Serial.print(motor_throttle[i]);
  }
  Serial.print("|");

  // write to motors
  for (int i = 0; i < MOTOR_MAX; i++) {
    motor[i].writeMicroseconds(motor_throttle[i]);
  }
}

void Motors::print() {
  Serial.print("!T");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(rol);
  Serial.print(",");
  Serial.print(throttle);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print("|");
}

