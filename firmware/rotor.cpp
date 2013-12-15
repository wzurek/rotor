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
#include "ground_station.h"

#include "global_objects.h"

#include "cmds.hxx"

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
Receiver receiver(RECEIVER_PIN1, RECEIVER_PIN2, RECEIVER_PIN3, RECEIVER_PIN4, RECEIVER_PIN5, RECEIVER_PIN6,
    RECEIVER_PIN7, RECEIVER_PIN8, RECEIVER_PIN9, RECEIVER_PIN10);
L3G gyro;
LSM303 compass;
Motors motors;

PID accelPID(4, 0.01, 0);

// the values baced on experiments
Vector3f accelCorrection(-7.8, -16.1, 0);

PID pitchPID(1, 0.0, 0.5);
PID rolPID(1, 0.0, 0.5);
PID yawPID(1, 0.0, 0.5);

// scale of the gyro component for PID
float gyroScale = 0.4;
float anglesScale = 0.4;

PID *pids[] = { &pitchPID, &rolPID, &yawPID, &stabilizePitchPID, &stabilizeRolPID, &stabilizeYawPID, &accelPID };

// -------------

//0
#define REPORT_SENSORS 1

//1
#define REPORT_RECEIVER 2

//2
#define REPORT_ANGLES 4

//3
#define REPORT_MOTORS 8

//4
#define REPORT_TIME 16

//5
#define REPORT_DCM 32

//6
#define REPORT_STABILIZATION 64

//7
#define REPORT_OMEGA 128

uint32_t REPORTING = REPORT_ANGLES | REPORT_TIME | REPORT_MOTORS
    | REPORT_SENSORS; // REPORT_RECEIVER; //|REPORT_SENSORS;// | REPORT_RECEIVER; // | REPORT_STABILIZATION | REPORT_RECEIVER

// current time
uint32_t currentMicros;

// desired direction
Vector3f stabilise(1, 0, 0);

// actual direction
Vector3f base(1, 0, 0);

// rotation from desired to actual
Matrix3f dcmRotation;

// accelerator drift correction
Vector3f omegaP;
// accelerator accumulated drift correction
Vector3f omegaI(0, 0, 0);

// gyro gain on last interval
Vector3f gyroGain(0, 0, 0);

// current angles
float kinematicsAngle[3];

// ground angles (as it was when calibrating)
float groundAngles[3];

// target angles
float targetAngles[3];

// max target angle
#define MAX_ANGLE (HALF_PI/8)

// loops constants
#define DELAY_50HZ 20
uint32_t next50Hz;

void initGyro() {
  if (!gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH)) {
    groundStation.textMessage("Failed to auto-detect gyroscope type!");
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
}

void setup() {
  // open serial and delay a bit
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  // send begin of transmission command
  uint8_t start[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, };
  groundStation.writeBytes(start, 5);
  delay(30);
  groundStation.writeBytes(start, 5);
  delay(100);

  // start listening to the receiver
  receiver.start();
  delay(10);
  groundStation.textMessage("Receiver started");

  // init servos
  initGyro();
  initCompass();

  // schedule next 50Hz refresh
  next50Hz = 0;

  // register ground station commands
//  groundStation.registerCommand('R', cmdUpdateReporting);
//  groundStation.registerCommand('M', cmdMotors);
  groundStation.registerCommand('P', cmdUpdatePid);
//  groundStation.registerCommand('A', cmdUpdateAccelError);
//  groundStation.registerCommand('G', cmdUpdateGyroScale);

  groundStation.textMessage("Finished setup");
}

void readReceiver() {
  static uint32_t channels[MAX_CHANNEL];

  // get new data from receiver
  receiver.readAll(channels);
  if (REPORTING & REPORT_RECEIVER) {
    receiver.print();
  }

  bool swOn = mapSwitch(channels[CH_5]);

  // continue failsafe mode until the stick is on low position at least once.
  if (motors.failSafeStart) {
    if (!swOn && receiver.connected) {
      motors.failSafeStart = false;
    }
  }

  if (!motors.failSafeStart) {
    if (swOn && !motors.armed) {
      motors.arm();
    } else if (swOn) {
      motors.throttle = mapThrottle(channels[CH_THROTTLE]);
      targetAngles[PITCH] = mapStick(channels[CH_PITCH]) * MAX_ANGLE;
      targetAngles[ROL] = -mapStick(channels[CH_ROL]) * MAX_ANGLE;
      targetAngles[YAW] = mapStick(channels[CH_YAW]) * MAX_ANGLE;
    } else {
      motors.throttle = 0;
      motors.pitch = 0;
      motors.rol = 0;
      motors.yaw = 0;
    }
  }
}

void eulerAngles() {
  kinematicsAngle[XAXIS] = atan2(dcmRotation.data[7], dcmRotation.data[8]);
  kinematicsAngle[YAXIS] = -asin(dcmRotation.data[6]);
  kinematicsAngle[ZAXIS] = atan2(dcmRotation.data[3], dcmRotation.data[0]);

  if (REPORTING & REPORT_ANGLES) {
    print3vf(CMD_ANGLES, kinematicsAngle);
  }
}

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define GRAVITY 245
void computeOmegas() {

  Vector3f acc(-compass.a.x, -compass.a.y, -compass.a.z);
  acc.substract(&accelCorrection);

  float weight = 1;
  float kp = accelPID.p;
  float ki = accelPID.i;

  float len = acc.length() / GRAVITY;
  acc.normalize();

  if (len < 0.5 || len > 1.5) {
    weight = 0;
  } else {
    if (len > 1) {
      len = 1.5 - len;
    } else {
      len -= 0.5;
    }
    weight = len * 2;
  }

  Vector3f err;
  acc.cross(&dcmRotation.data[6], err.data);

  omegaP.copyFrom(err.data);
  omegaP.multiply(kp * weight);

  Vector3f dOmegaI;
  dOmegaI.copyFrom(err.data);
  dOmegaI.multiply(ki * weight);

  omegaI.add(&dOmegaI);

  omegaP.data[ZAXIS] = 0;
  omegaI.data[ZAXIS] = 0;

  if (REPORTING & REPORT_OMEGA) {
//    Serial.print(CMD_BEGIN);
//    Serial.print(CMD_OMEGA);
//    Serial.print(kp);
//    Serial.print(',');
//    Serial.print(ki, 4);
//    Serial.print(',');
//    Serial.print(weight);
//    Serial.print(';');
//    accelCorrection.print();
//    Serial.print(';');
//    omegaP.print();
//    Serial.print(';');
//    omegaI.print();
//    Serial.print(CMD_END);
  }
}

#define GYRO_GAIN 5413
void updateOrientation() {

  static uint32_t lastUpdate;

  float xps = gyro.g.x;
  float yps = gyro.g.y;
  float zps = gyro.g.z;

  xps = (xps / GYRO_GAIN) * PI * 2;
  yps = (yps / GYRO_GAIN) * PI * 2;
  zps = (zps / GYRO_GAIN) * PI * 2;

  gyroGain.data[XAXIS] = xps;
  gyroGain.data[YAXIS] = yps;
  gyroGain.data[ZAXIS] = zps;

  computeOmegas();

  gyroGain.substract(&omegaP);
  gyroGain.substract(&omegaI);

  float dt = (currentMicros - lastUpdate) / 1000000.0;
  lastUpdate = currentMicros;

  Vector3f gt;
  gt.copyFrom(gyroGain.data);

  gt.multiply(dt);

  dcmRotation.applyRotation(gt.data);
  dcmRotation.fixError();

  if (REPORTING & REPORT_DCM) {
    dcmRotation.print(CMD_DCM);
  }

  base.copyFrom(stabilise.data);
  base.transform(dcmRotation.data);

  Vector3f accerDCM;
  // set as base vector
  accerDCM.copyFrom(compass.accel_base.data);
  // compute the expected value based on DCM matrix
  accerDCM.transform(dcmRotation.data);

  eulerAngles();
}

void perform50HzActions() {
  // read from receiver
  readReceiver();

  // read sensors
  gyro.read();

  compass.readAcc();
  if (REPORTING & REPORT_SENSORS) {
    groundStation.beginMessage(CMD_SENSORS);

    int32_t gints[] = { gyro.g.x, gyro.g.y, gyro.g.z };
    groundStation.writeVIntsField(1, gints, 3);

    int32_t aints[] = { compass.a.x, compass.a.y, compass.a.z };
    groundStation.writeVIntsField(2, aints, 3);

    groundStation.finishMessage();
  }

  // update orientation
  updateOrientation();

  // update motor
  if (motors.armed) {
//    motors.pitch = pitchPID.computePID(kinematicsAngle[PITCH] * anglesScale + gyroGain.data[PITCH] * gyroScale,
//        targetAngles[PITCH], currentMicros);
//    motors.rol = rolPID.computePID(kinematicsAngle[ROL] * anglesScale + gyroGain.data[ROL] * gyroScale,
//        targetAngles[ROL], currentMicros);
//    motors.yaw = yawPID.computePID(kinematicsAngle[YAW] * anglesScale + gyroGain.data[YAW] * gyroScale,
//        targetAngles[YAW], currentMicros);
//
    // based on gyro, targeting gyro gain == 0;
    motors.pitch = gyroScale * pitchPID.computePID(gyroGain.data[PITCH], 0, currentMicros);
    motors.rol = gyroScale * rolPID.computePID(gyroGain.data[ROL], 0, currentMicros);
    motors.yaw = gyroScale * yawPID.computePID(gyroGain.data[YAW], 0, currentMicros);

    // based on location
    motors.pitch -= (kinematicsAngle[PITCH] - targetAngles[PITCH]) * anglesScale;
    motors.rol -= (kinematicsAngle[ROL] - targetAngles[ROL]) * anglesScale;
    motors.yaw -= (kinematicsAngle[YAW] - targetAngles[YAW]) * anglesScale;

    if (REPORTING & REPORT_STABILIZATION) {

      groundStation.beginMessage(CMD_STABILIZATION);
      pitchPID.print(1);
      rolPID.print(2);
      yawPID.print(3);
      groundStation.writeFloatsField(4, gyroGain.data, 3);

      float motorsData[] = { motors.pitch, motors.rol, motors.yaw };
      groundStation.writeFloatsField(5, motorsData, 3);
      groundStation.writeFloatsField(6, targetAngles, 3);

      groundStation.writeFloatField(7, gyroScale);

      groundStation.finishMessage();
    }
  }

  // update motors with commands
  motors.updateMotors();
  if (REPORTING & REPORT_MOTORS) {
    motors.print();
  }
}

void sendConfiguratio() {
  groundStation.beginMessage(CMD_CONFIGURATION);
  groundStation.writeFloatField(1, anglesScale);
  groundStation.writeFloatField(2, gyroScale);
  groundStation.writeFloatsField(3, gyroGain.data, 3);
  groundStation.writeVUInt32Field(4, THROTTLE_MAX);
  groundStation.writeVUInt32Field(5, PITCH_MAX);
  groundStation.finishMessage();
}

int sec = 50;

void loop() {
  uint32_t currentMillisTime = millis();
  currentMicros = micros();

  if (currentMillisTime > next50Hz) {

    perform50HzActions();

    if (REPORTING & REPORT_TIME) {
      groundStation.beginMessage(CMD_TIME);
      groundStation.writeVUInt32Field(1, currentMillisTime);
      groundStation.writeVUInt32Field(2, millis() - currentMillisTime);
      groundStation.writeVUInt32Field(3, groundStation.bytesSent());
      groundStation.finishMessage();
    }

    groundStation.processCmds();

    next50Hz += DELAY_50HZ;
  }
}
