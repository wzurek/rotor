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
  motor[FRONT_LEFT].attach(42);
  motor[FRONT_RIGHT].attach(40);
  motor[REAR_LEFT].attach(38);
  motor[REAR_RIGHT].attach(36);

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

void Motors::disarm() {
  armed = false;
}

void Motors::updateMotors() {

  if (!armed) {
    motor_throttle[FRONT_LEFT] = 0;
    motor_throttle[FRONT_RIGHT] = 0;
    motor_throttle[REAR_RIGHT] = 0;
    motor_throttle[REAR_LEFT] = 0;

  } else {

    uint32_t base_throttle = (REC_MIN + REC_RANGE * throttle);
    if (base_throttle > THROTTLE_MAX) {
      base_throttle = THROTTLE_MAX + 1;
    }

    float p = constrain(pitch, -1,1);
    float r = constrain(rol, -1,1);
    float y = constrain(yaw, -1,1);

    motor_throttle[FRONT_LEFT] = base_throttle + (p * PITCH_MAX)
        - (r * ROL_MAX);

    motor_throttle[FRONT_RIGHT] = base_throttle + (p * PITCH_MAX)
        + (r * ROL_MAX);

    motor_throttle[REAR_RIGHT] = base_throttle - (p * PITCH_MAX)
        + (r * ROL_MAX);

    motor_throttle[REAR_LEFT] = base_throttle - (p * PITCH_MAX) - (r * ROL_MAX);

    // check maxes
    for (int i = 0; i < MOTOR_MAX; i++) {
      if (motor_throttle[i] > THROTTLE_MAX) {
        motor_throttle[i] = THROTTLE_MAX + 1;
      }
    }
  }

  // write to motors
  for (int i = 0; i < MOTOR_MAX; i++) {
    motor[i].writeMicroseconds(motor_throttle[i]);
  }
}

void Motors::print() {
  Serial.print("!M");
  Serial.print(motor_throttle[0]);
  for (int i = 1; i < MOTOR_MAX; i++) {
    Serial.print(",");
    Serial.print(motor_throttle[i]);
  }
  Serial.print("|");
}

