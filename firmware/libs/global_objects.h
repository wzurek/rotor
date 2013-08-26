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

#endif /* GLOBAL_OBJECTS_H_ */
