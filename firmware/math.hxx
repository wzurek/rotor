/*
 * math.hxx
 *
 *  Created on: Aug 22, 2013
 *      Author: wzurek
 */

#include "receiver.h"

// map integer stick value to -1.0:1.0 range
float mapStick(float value) {
  if (value < REC_MIN) {
    value = REC_MIN;
  } else if (value > REC_MAX) {
    value = REC_MAX;
  }
  float result = (value - REC_MIN - REC_RANGE / 2) * 2;
  return result / REC_RANGE;
}

// map integer throttle stick to 0.0:1.0 range
float mapThrottle(float value) {
  if (value < REC_MIN) {
    value = REC_MIN;
  } else if (value > REC_MAX) {
    value = REC_MAX;
  }
  float result = value - REC_MIN;
  return result / REC_RANGE;
}

union PID {
  float p, i, d;

  float previous_error;
  float integral;
  float lastTime;
};

float compoutePID(float value, float target, PID* pid, float currentTime) {
  float dt = currentTime - pid->lastTime;

  if (pid->lastTime == 0) {
    pid->lastTime = currentTime;
    return 0;
  }
  pid->lastTime = currentTime;

  float error = target - value;
  pid->integral += error * dt;
  float derivate = (error - pid->previous_error) / dt;
  pid->previous_error = error;

  return pid->p * error + pid->i * pid->integral + pid->d * derivate;
}

// print 3 values
void print3vf(char cmd, float v1, float v2, float v3) {
  Serial.print("!");
  Serial.print(cmd);
  Serial.print(v1);
  Serial.print(",");
  Serial.print(v2);
  Serial.print(",");
  Serial.print(v3);
  Serial.print("|");
}
// print vector
void print3vf(char cmd, Vector3f* v) {
  print3vf(cmd, v->data[XAXIS], v->data[YAXIS], v->data[ZAXIS]);
}
// print 3 values
void print3vi(char cmd, int32_t v1, int32_t v2, int32_t v3) {
  Serial.print("!");
  Serial.print(cmd);
  Serial.print(v1);
  Serial.print(",");
  Serial.print(v2);
  Serial.print(",");
  Serial.print(v3);
  Serial.print("|");
}

float length(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

