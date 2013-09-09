/*
 * global_objects.h
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#ifndef GLOBAL_OBJECTS_H_
#define GLOBAL_OBJECTS_H_

#include "receiver.h"
#include "L3G.h"
#include "LSM303.h"
#include "motors.h"

// -------- RECEIVER

extern Receiver receiver;

// ---------- SENSORS

extern L3G gyro;
extern LSM303 compass;

// ----------- MOTORS

extern Motors motors;

// ------- navigation
// current angles
extern float kinematicsAngle[3];

// ground angles (as it was when calibrating)
extern float groundAngles[3];

// target angles
extern float targetAngles[3];

// PIDs

class PID;

extern PID pitchPID;
extern PID rolPID;
extern PID yawPID;

extern PID stabilizePitchPID;
extern PID stabilizeRolPID;
extern PID stabilizeYawPID;

extern PID *pids[];

extern uint32_t REPORTING;


#endif /* GLOBAL_OBJECTS_H_ */
