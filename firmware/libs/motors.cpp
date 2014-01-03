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

  if (motor_mode == MOTOR_MODE_SAFE_START) {
    // don't arm during failsafe start
    return;
  }

  if (motor_mode == MOTOR_MODE_OFF) {
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
    groundStation.textMessage("Booting speed controllers...");
    for (int i = 0; i < 8; i++) {
      groundStation.textMessage(".");
      delay(500);
    }
    groundStation.textMessage("booting done");
  }

  motor_mode = MOTOR_MODE_OPERATION;
}

Motors::Motors() {
  yaw = 0;
  pitch = 0;
  rol = 0;

  throttle = 0;

  motor_mode = MOTOR_MODE_SAFE_START;
}

void Motors::updateMotors() {
  switch (motor_mode) {

  case MOTOR_MODE_OPERATION:
    // normal operation, take throttle values from desired pitch/yaw/rol
    updateThrottleForOperationMode();
    break;

  case MOTOR_MODE_SAFE_START:
    // safe start, do nothing;
    return;

  case MOTOR_MODE_OFF:
    // not armed, send zeros
    motor_throttle[FRONT_LEFT] = 0;
    motor_throttle[FRONT_RIGHT] = 0;
    motor_throttle[REAR_RIGHT] = 0;
    motor_throttle[REAR_LEFT] = 0;
    break;

  case MOTOR_MODE_STOPPED:
    // armed but stopped, send minimum - 1000
    motor_throttle[FRONT_LEFT] = 1000;
    motor_throttle[FRONT_RIGHT] = 1000;
    motor_throttle[REAR_RIGHT] = 1000;
    motor_throttle[REAR_LEFT] = 1000;
    break;

  case MOTOR_MODE_DIRECT_COMMAND:
    // nothing done, use the throttle values as they are
    break;

  default:
    // something very wrong, try to set some safe values.
    groundStation.textMessage("Invalid motor mode!");
    motor_throttle[FRONT_LEFT] = 1000;
    motor_throttle[FRONT_RIGHT] = 1000;
    motor_throttle[REAR_RIGHT] = 1000;
    motor_throttle[REAR_LEFT] = 1000;
    return;
  }

  // write to motors
  for (int i = 0; i < MOTOR_MAX; i++) {
    motor[i].writeMicroseconds(motor_throttle[i]);
  }
}

void Motors::updateThrottleForOperationMode() {

  // Algorithm to change the throttle + pitch/rol/yaw into the actual motor pulse values
  uint32_t base_throttle = (REC_MIN + REC_RANGE * throttle);
  if (base_throttle > THROTTLE_MAX) {
    base_throttle = THROTTLE_MAX + 1;
  }

  float p = constrain(pitch, -1 , 1);
  float r = constrain(rol, -1 , 1);
  float y = constrain(yaw, -1 , 1);

  motor_throttle[FRONT_LEFT] = base_throttle - (p * PITCH_MAX) + (r * ROL_MAX);
  motor_throttle[FRONT_RIGHT] = base_throttle - (p * PITCH_MAX) - (r * ROL_MAX);
  motor_throttle[REAR_RIGHT] = base_throttle + (p * PITCH_MAX) - (r * ROL_MAX);
  motor_throttle[REAR_LEFT] = base_throttle + (p * PITCH_MAX) + (r * ROL_MAX);

  // check maxes
  for (int i = 0; i < MOTOR_MAX; i++) {
    if (motor_throttle[i] > THROTTLE_MAX) {
      motor_throttle[i] = THROTTLE_MAX + 1;
    }
  }
}

// print throttle and the instruction
void Motors::print() {

  groundStation.beginMessage(CMD_MOTORS); // motors M
  groundStation.writeVUIntsField(1, motor_throttle, MOTOR_MAX);
  float data[] = { throttle, pitch, rol, yaw };
  groundStation.writeFloatsField(2, data, 4);
  groundStation.writeVUInt32Field(3, motor_mode);
  groundStation.finishMessage();
}

void Motors::init() {
  if (motor_mode == MOTOR_MODE_SAFE_START) {
    motor_mode = MOTOR_MODE_OFF;
  }
}

void Motors::direct() {
  if (motor_mode <= MOTOR_MODE_OFF) {
    return;
  }
  // put to minimum
  for (int i = 0; i < MOTOR_MAX; i++) {
    motor_throttle[i] = REC_MIN;
  }
  motor_mode = MOTOR_MODE_DIRECT_COMMAND;
}
