/*
 * math.hxx
 *
 *  Created on: Aug 22, 2013
 *      Author: wzurek
 */

#include "receiver.h"

#define CMD_BEGIN '!'
#define CMD_END '|'

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

class PID {
public:
  PID(float p, float i, float d);

  float p, i, d;

  float previous_error;
  float integral;
  float lastTime;

  float computePID(float value, float target, float currentTime);
  void print();
};

float PID::computePID(float value, float target, float currentTime) {
  float dt = currentTime - lastTime;

  if (lastTime == 0) {
    lastTime = currentTime;
    return 0;
  }
  lastTime = currentTime;

  float error = target - value;
  if (i != 0) {
    integral += error * dt;
  }
  float derivate = (error - previous_error) / dt;
  previous_error = error;

  return p * error + i * integral + d * derivate;
}

void PID::print() {
  Serial.print(p);
  Serial.print(',');
  Serial.print(i);
  Serial.print(',');
  Serial.print(d);
  Serial.print(',');
  Serial.print(previous_error);
  Serial.print(',');
  Serial.print(integral);
  Serial.print(',');
  Serial.print(lastTime);
}

PID::PID(float p, float i, float d) {
  this->d = d;
  this->i = i;
  this->p = p;
  previous_error = 0;
  integral = 0;
  lastTime = 0;
}

// print 3 values
void print3vf(char cmd, float v1, float v2, float v3) {
  Serial.print(CMD_BEGIN);
  Serial.print(cmd);
  Serial.print(v1);
  Serial.print(',');
  Serial.print(v2);
  Serial.print(',');
  Serial.print(v3);
  Serial.print(CMD_END);
}

// print vector
void print3vf(char cmd, Vector3f* v) {
  print3vf(cmd, v->data[XAXIS], v->data[YAXIS], v->data[ZAXIS]);
}

// print 3 values
void print3vi(char cmd, int32_t v1, int32_t v2, int32_t v3) {
  Serial.print(CMD_BEGIN);
  Serial.print(cmd);
  Serial.print(v1);
  Serial.print(',');
  Serial.print(v2);
  Serial.print(',');
  Serial.print(v3);
  Serial.print(CMD_END);
}

// print vector
void printVectorData(float* v) {
  Serial.print(v[XAXIS]);
  Serial.print(',');
  Serial.print(v[YAXIS]);
  Serial.print(',');
  Serial.print(v[ZAXIS]);
}

float length(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

