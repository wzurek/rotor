/*
 * math.hxx
 *
 *  Created on: Aug 22, 2013
 *      Author: wzurek
 */

#include "receiver.h"

#include "global_objects.h"

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
  void print(uint32_t key);
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

void PID::print(uint32_t key) {
  float values[] = { p, i, d}; //, previous_error, integral, lastTime };
  groundStation.writeFloatsField(key, values, 3);
}

PID::PID(float p, float i, float d) {
  this->d = d;
  this->i = i;
  this->p = p;
  previous_error = 0;
  integral = 0;
  lastTime = 0;
}

void print3vf(uint32_t cmd, float* v) {
  groundStation.beginMessage(cmd);
  groundStation.writeFloatsField(1, v, 3);
  groundStation.finishMessage();
}

// print vector
void print3vf(uint32_t cmd, Vector3f* v) {
  print3vf(cmd, v->data);
}

// print 3 values
void print3vi(uint32_t cmd, int32_t v1, int32_t v2, int32_t v3) {

  // 3 vints, max 15 bytes.
  uint8_t buff[15];

  uint32_t size = 0;

  size += groundStation.appendVint32(v1, buff);
  size += groundStation.appendVint32(v2, buff + size);
  size += groundStation.appendVint32(v3, buff + size);

  groundStation.beginMessage(cmd);
  groundStation.writeFixedField(1, size, buff);
  groundStation.finishMessage();
}

// print vector
void printVectorData(float* v) {
//  Serial.print(v[XAXIS]);
//  Serial.print(',');
//  Serial.print(v[YAXIS]);
//  Serial.print(',');
//  Serial.print(v[ZAXIS]);
}

float length(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

uint32_t readVuint32(uint8_t *buff, int32_t &val) {
  val = 0;
  uint8_t *b = buff;
  while (*b & 0x80) {
    val <<= 7;
    val |= (*b & 0x7f);
    b++;
  }
  val <<= 7;
  val |= (*b & 0x7f);
  return b - buff + 1;
}
