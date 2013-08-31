/*
 * receiver_test.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: wzurek
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include "L3G.h"
#include "LSM303.h"
#include "math.hxx"
#include "vector3f.h"
#include "receiver.h"
#include "motors.h"

#include "global_objects.h"

#define RECEIVER_PIN1 51
#define RECEIVER_PIN2 49
#define RECEIVER_PIN3 47
#define RECEIVER_PIN4 45
#define RECEIVER_PIN5 43
#define RECEIVER_PIN6 41
#define RECEIVER_PIN7 39
#define RECEIVER_PIN8 37
#define RECEIVER_PIN9 35
#define RECEIVER_PIN10 33

// --------------
// Global objects
Receiver receiver(RECEIVER_PIN1, RECEIVER_PIN2, RECEIVER_PIN3, RECEIVER_PIN4,
    RECEIVER_PIN5, RECEIVER_PIN6, RECEIVER_PIN7, RECEIVER_PIN8, RECEIVER_PIN9,
    RECEIVER_PIN10);
L3G gyro;
LSM303 compass;
Motors motors;

PID accelPidX;
PID accelPidY;

// -------------

#define REPORT_SENSORS 1
#define REPORT_RECEIVER 2

uint32_t REPORTING = REPORT_SENSORS & REPORT_RECEIVER;

uint32_t currentMicros;

// desired direction
Vector3f stabilise(1, 0, 0);

// actual direction
Vector3f base(1, 0, 0);

// rotation from desired to actual
Matrix3f dcmRotation;

float kinematicsAngle[3];

// loops constants
#define DELAY_50HZ 20
uint32_t next50Hz;

void initGyro() {
  if (!gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH)) {
    Serial.println("Failed to auto-detect gyroscope type!");
    while (1)
      ;
  }
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale

  gyro.calibrate();
}

void initCompass() {
  compass.init(LSM303DLHC_DEVICE);
  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
  compass.writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode

  compass.calibrateAccel();

  XAXIS;

  accelPidX.p = 10;
  accelPidX.i = 1;
  accelPidX.d = 0;

  accelPidX.integral = 0;
  accelPidX.lastTime = 0;
  accelPidX.previous_error = 0;

  accelPidY.p = 10;
  accelPidY.i = 1;
  accelPidY.d = 0;

  accelPidY.integral = 0;
  accelPidY.lastTime = 0;
  accelPidY.previous_error = 0;
}

void setup() {
  // open serial and delay a bit
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  Serial.println("S");
  delay(100);

  // start listening to the receiver
  receiver.start();
  delay(10);
  Serial.println("receiver started.");

  // init servos
  initGyro();
  initCompass();

  // arm motors
  motors.arm();

  // schedule next 50Hz refresh
  next50Hz = 0;
}

void readReceiver() {
  static uint32_t channels[MAX_CHANNEL];

  // get new data from receiver
  receiver.readAll(channels);
  if (REPORTING & REPORT_RECEIVER) {
    receiver.print();
  }

  motors.throttle = mapThrottle(channels[CH_THROTTLE]);
  motors.pitch = mapStick(channels[CH_PITCH]);
  motors.rol = mapStick(channels[CH_ROL]);
  motors.yaw = mapStick(channels[CH_YAW]);
}

void eulerAngles() {
  kinematicsAngle[XAXIS] = atan2(dcmRotation.data[7], dcmRotation.data[8]);
  kinematicsAngle[YAXIS] = -asin(dcmRotation.data[6]);
  kinematicsAngle[ZAXIS] = atan2(dcmRotation.data[3], dcmRotation.data[0]);

  print3vf('E', kinematicsAngle[0], kinematicsAngle[1], kinematicsAngle[2]);
}

#define GYRO_GAIN 5413
#define GRAVITY 240
void updateOrientation() {
  static uint32_t lastUpdate;

  float xps = gyro.g.x;
  float yps = gyro.g.y;
  float zps = gyro.g.z;

  xps = (xps / GYRO_GAIN) * PI * 2;
  yps = (yps / GYRO_GAIN) * PI * 2;
  zps = (zps / GYRO_GAIN) * PI * 2;

  float dt = (currentMicros - lastUpdate) / 1000000.0;
  lastUpdate = currentMicros;

  dcmRotation.applyRotation(xps * dt, yps * dt, zps * dt);
  dcmRotation.fixError();

  base.copyFrom(stabilise.data);
  base.transform(dcmRotation.data);

  // add PI from acceleration
  float ax = compass.a.x;
  float ay = compass.a.y;
  float az = compass.a.z;

  Vector3f pi;
  pi.data[XAXIS] = compoutePID(ax, 0, &accelPidX, currentMicros);
  pi.data[YAXIS] = compoutePID(ay, 0, &accelPidY, currentMicros);

  float force = length(ax, ay, az) / GRAVITY;

  // if the force is close to gravity, apply PID for drift correction
  if (force > 0.5 && force < 1.5) {
    print3vf('B', pi.data[XAXIS], pi.data[YAXIS], pi.data[ZAXIS]);
  }
  print3vf('F', force, 0, 0);

  print3vf('O', base.data[0], base.data[1], base.data[2]);

  eulerAngles();
}

void perform50HzActions() {
  // read from receiver
  readReceiver();

  // read sensors
  gyro.read();
  gyro.print();

  compass.readAcc();
  if (REPORTING & REPORT_SENSORS) {
    print3vi('A', compass.a.x, compass.a.y, compass.a.z);
  }

  compass.readMag();
  if (REPORTING & REPORT_SENSORS) {
    print3vi('M', compass.m.x, compass.m.y, compass.m.z);
  }
  // update orientation
  updateOrientation();

  // update motors with commands
  motors.updateMotors();
  motors.print();
}

void loop() {
  uint32_t currentMillisTime = millis();
  currentMicros = micros();

  if (currentMillisTime > next50Hz) {
    perform50HzActions();

    Serial.print("!T");
    Serial.print(millis() - currentMillisTime);
    Serial.print("|\n");

    next50Hz += DELAY_50HZ;
  }
}
