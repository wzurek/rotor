/*
 * Vector3f.h
 *
 *  Created on: Aug 24, 2013
 *      Author: wzurek
 */

#ifndef VECTOR3F_H_
#define VECTOR3F_H_

#include <Arduino.h>

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

class Vector3f {

public:

  float data[3];

  Vector3f();
  Vector3f(float x, float y, float z);
  Vector3f(float* data, int index);

  void print();

  /** transform given vector through DCM matrix */
  void transform(float* m);
  void cross(float* b, float* out);
  float dot(Vector3f b);

  void multiply(float dot);
  float length();
  void substract(Vector3f* sub);
  void add(Vector3f* add);
  void normalize();
  void copyFrom(float* src);

};

class Matrix3f {

public:

  Matrix3f();
  Matrix3f(float v11, float v12, float v13, //
      float v21, float v22, float v23, //
      float v31, float v32, float v33);

  float data[9];

  void print();
  void multiply(float* m);
  void applyRotation(float* g);
  void applyRotation(float yaw, float pitch, float rol);
  void fixError();
};

#endif /* VECTOR3F_H_ */
