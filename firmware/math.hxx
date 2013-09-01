/*
 * math.hxx
 *
 *  Created on: Aug 22, 2013
 *      Author: wzurek
 */

#include "receiver.h"

float fixReading(float value) {
  if (value < REC_MIN) {
    // error corrections, sometimes it loose 1000 ns
    if (value < REC_AB_MIN && value > REC_AB_MIN - REC_AB_RANGE) {
      return value + REC_AB_RANGE;
    }
    value = REC_MIN;
  } else if (value > REC_MAX) {
    // error corrections, sometimes it loose 1000 ns
    if (value > REC_AB_MAX && value < REC_AB_MAX + REC_AB_RANGE) {
      return value - REC_AB_RANGE;
    }
    value = REC_MAX;
  }
  return value;
}

// map integer stick value to -1.0:1.0 range
float mapStick(float value) {
  value = fixReading(value);
  float result = (value - REC_MIN - REC_RANGE / 2) * 2;
  return result / REC_RANGE;
}

int mapSwitch(float value) {
  value = fixReading(value);
  return (value > REC_MID);
}

// map integer throttle stick to 0.0:1.0 range
float mapThrottle(float value) {
  value = fixReading(value);
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

// print vector
void printVector(float* v) {
  Serial.print(v[XAXIS]);
  Serial.print(",");
  Serial.print(v[YAXIS]);
  Serial.print(",");
  Serial.print(v[ZAXIS]);
}

float length(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

