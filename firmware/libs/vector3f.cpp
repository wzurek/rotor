/*
 * Vector3f.cpp
 *
 *  Created on: Aug 24, 2013
 *      Author: wzurek
 */

#include "Vector3f.h"

Vector3f::Vector3f() {
  data[XAXIS] = 0;
  data[YAXIS] = 0;
  data[ZAXIS] = 0;
}

Vector3f::Vector3f(float x, float y, float z) {
  data[XAXIS] = x;
  data[YAXIS] = y;
  data[ZAXIS] = z;
}

Vector3f::Vector3f(float* data, int index) {
  for (int i = 0; i < 3; i++) {
    this->data[i] = data[index + i];
  }
}

/** transform given vector through DCM matrix */
void Vector3f::transform(float* m) {
  float t[3];
  for (int i = 0; i < 3; i++) {
    t[i] = 0;

    for (int j = 0; j < 3; j++) {
      t[i] += data[j] * m[i + 3 * j];
    }
  }
  for (int i = 0; i < 3; i++) {
    data[i] = t[i];
  }
}

void Vector3f::copyFrom(float* src) {
  for (int i = 0; i < 3; i++) {
    data[i] = src[i];
  }
}

/** copy cross product of the vectors to out array */
void Vector3f::cross(float* b, float* out) {
  out[XAXIS] = data[YAXIS] * b[ZAXIS] - data[ZAXIS] * b[YAXIS];
  out[YAXIS] = data[ZAXIS] * b[XAXIS] - data[XAXIS] * b[ZAXIS];
  out[ZAXIS] = data[XAXIS] * b[YAXIS] - data[YAXIS] * b[XAXIS];
}

float Vector3f::dot(Vector3f b) {
  return data[XAXIS] * b.data[XAXIS] + data[YAXIS] * b.data[YAXIS]
      + data[ZAXIS] * b.data[ZAXIS];
}

void Vector3f::multiply(float dot) {
  data[XAXIS] *= dot;
  data[YAXIS] *= dot;
  data[ZAXIS] *= dot;
}

void Vector3f::substract(Vector3f* sub) {
  data[XAXIS] -= sub->data[XAXIS];
  data[YAXIS] -= sub->data[YAXIS];
  data[ZAXIS] -= sub->data[ZAXIS];
}

void Vector3f::normalize() {
  double length = sqrt(
      data[XAXIS] * data[XAXIS] + data[YAXIS] * data[YAXIS]
          + data[ZAXIS] * data[ZAXIS]);
  data[XAXIS] /= length;
  data[YAXIS] /= length;
  data[ZAXIS] /= length;
}

Matrix3f::Matrix3f() {
  for (int i = 0; i < 9; i++) {
    if (i / 3 == i % 3) {
      data[i] = 1;
    } else {
      data[i] = 0;
    }
  }
}

void Matrix3f::print() {
}

Matrix3f::Matrix3f(float v11, float v12, float v13, //
    float v21, float v22, float v23, //
    float v31, float v32, float v33) {
  data[0] = v11;
  data[1] = v12;
  data[2] = v13;
  data[3] = v21;
  data[4] = v22;
  data[5] = v23;
  data[6] = v31;
  data[7] = v32;
  data[8] = v33;
}

void Matrix3f::multiply(float* m) {
  float result[9];

  for (int i = 0; i < 9; i++) {
    int x = i % 3;
    int ri = i - x;

    result[i] = 0;
    for (int q = 0; q < 3; q++) {
      result[i] += data[q + ri] * m[q * 3 + x];
    }
  }
  for (int i = 0; i < 9; i++) {
    data[i] = result[i];
  }
}
void Matrix3f::applyRotation(float* g) {
  applyRotation(g[XAXIS], g[YAXIS], g[ZAXIS]);
}

void Matrix3f::applyRotation(float gx, float gy, float gz) {
  float m[9];
  m[0] = 1; //.. 11
  m[1] = -gz; // 12
  m[2] = gy; //. 13
  m[3] = gz; //. 21
  m[4] = 1; //.. 22
  m[5] = -gx; // 23
  m[6] = -gy; // 31
  m[7] = gx; //. 32
  m[8] = 1; //.. 33
  multiply(m);
}

void Matrix3f::fixError() {
  Vector3f a(data, 0);
  Vector3f b(data, 3);

  float err = a.dot(b) / 2;

  Vector3f aerr(a.data, 0);
  aerr.multiply(err);
  Vector3f berr(a.data, 0);
  berr.multiply(err);

  a.substract(&berr);
  a.normalize();

  b.substract(&berr);
  b.normalize();

  Vector3f c(0, 0, 0);
  a.cross(b.data, c.data);

  for (int j = 0; j < 3; j++) {
    data[j] = a.data[j];
  }
  for (int j = 0; j < 3; j++) {
    data[j + 3] = b.data[j];
  }
  for (int j = 0; j < 3; j++) {
    data[j + 6] = c.data[j];
  }
}

void Vector3f::add(Vector3f* add) {
  for (int i = 0; i < 3; i++) {
    data[i] += add->data[i];
  }
}
