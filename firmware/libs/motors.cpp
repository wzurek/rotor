/*
 * motors.cpp
 *
 *  Created on: Aug 23, 2013
 *      Author: wzurek
 */

#include "motors.h"
#include "receiver.h"
#include "global_objects.h"

void Motors::arm() {

  if (failSafeStart) {
    // don't arm during failsafe start
    return;
  }

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
  groundStation.textMessage("booting speed controllers");
  for (int i = 0; i < 10; i++) {
    groundStation.textMessage(".");
    delay(500);
  }
  groundStation.textMessage("booting done");

  armed = true;
}

void Motors::disarm() {
  armed = false;
}

Motors::Motors() {
  yaw = 0;
  pitch = 0;
  rol = 0;

  throttle = 0;
  armed = false;
  failSafeStart = true;
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

    motor_throttle[FRONT_LEFT] = base_throttle - (p * PITCH_MAX)
        + (r * ROL_MAX);

    motor_throttle[FRONT_RIGHT] = base_throttle - (p * PITCH_MAX)
        - (r * ROL_MAX);

    motor_throttle[REAR_RIGHT] = base_throttle + (p * PITCH_MAX)
        - (r * ROL_MAX);

    motor_throttle[REAR_LEFT] = base_throttle + (p * PITCH_MAX) + (r * ROL_MAX);

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

// print throttle and the instruction
void Motors::print() {

  groundStation.beginMessage(4); // motors M
  groundStation.writeVIntsField(1,motor_throttle,MOTOR_MAX);
  groundStation.writeFloatField(2, throttle);
  groundStation.writeFloatField(3, pitch);
  groundStation.writeFloatField(4, rol);
  groundStation.writeFloatField(5, yaw);
  groundStation.finishMessage();
}

