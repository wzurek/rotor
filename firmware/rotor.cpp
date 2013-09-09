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
Receiver receiver(RECEIVER_PIN1, RECEIVER_PIN2, RECEIVER_PIN3, RECEIVER_PIN4,
    RECEIVER_PIN5, RECEIVER_PIN6, RECEIVER_PIN7, RECEIVER_PIN8, RECEIVER_PIN9,
    RECEIVER_PIN10);
L3G gyro;
LSM303 compass;
Motors motors;

//PID accelPidX(10, 1, 0);
//PID accelPidY(10, 1, 0);

PID accelPID(0.1, 0.001, 0);
Vector3f accelCorrection(0, 0, 0);

PID pitchPID(1, 0.0, 200);
PID rolPID(1, 0.0, 200);
PID yawPID(1, 0.0, 200);

PID stabilizePitchPID(2, 0.00, 1);
PID stabilizeRolPID(2, 0.00, 1);
PID stabilizeYawPID(2, 0.00, 1);

PID *pids[] = { &pitchPID, &rolPID, &yawPID, &stabilizePitchPID,
    &stabilizeRolPID, &stabilizeYawPID, &accelPID };

// -------------

#define CMD_START 'S'
#define CMD_SENSORS_GYRO 'G'
#define CMD_SENSORS_ACC 'A'
#define CMD_RECEIVER 'R'
#define CMD_ANGLES 'E'
#define CMD_MOTORS 'M'
#define CMD_TIME 'T'
#define CMD_DCM 'D'
#define CMD_STABILIZATION 'Q'
#define CMD_OMEGA 'O'

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

uint32_t REPORTING = 0; // REPORT_RECEIVER; //|REPORT_SENSORS;// | REPORT_RECEIVER;

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
#define MAX_ANGLE (HALF_PI/2)

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
}

void setup() {
  // open serial and delay a bit
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  Serial.println(CMD_START);
  delay(100);

  // start listening to the receiver
  receiver.start();
  delay(10);
  Serial.println("receiver started.");

  // init servos
  initGyro();
  initCompass();

  // schedule next 50Hz refresh
  next50Hz = 0;

  // register ground station commands
  groundStation.registerCommand('R', cmdUpdateReporting);
  groundStation.registerCommand('M', cmdMotors);
  groundStation.registerCommand('P', cmdUpdatePid);
  groundStation.registerCommand('A', cmdUpdateAccelError);
}

void readReceiver() {
  static uint32_t channels[MAX_CHANNEL];

  // get new data from receiver
  receiver.readAll(channels);
  if (REPORTING & REPORT_RECEIVER) {
    receiver.print();
  }

  int motorsOn = mapSwitch(channels[CH_5]);
  if (motorsOn && !motors.armed) {
    motors.arm();
  } else if (!motorsOn) {
    motors.throttle = 0;
    motors.pitch = 0;
    motors.rol = 0;
    motors.yaw = 0;
  } else {
    motors.throttle = mapThrottle(channels[CH_THROTTLE]);
    targetAngles[PITCH] = mapStick(channels[CH_PITCH]) * MAX_ANGLE;
    targetAngles[ROL] = -mapStick(channels[CH_ROL]) * MAX_ANGLE;
    targetAngles[YAW] = mapStick(channels[CH_YAW]) * MAX_ANGLE;
  }
}

void eulerAngles() {
  kinematicsAngle[XAXIS] = atan2(dcmRotation.data[7], dcmRotation.data[8]);
  kinematicsAngle[YAXIS] = -asin(dcmRotation.data[6]);
  kinematicsAngle[ZAXIS] = atan2(dcmRotation.data[3], dcmRotation.data[0]);

  if (REPORTING & REPORT_ANGLES) {
    print3vf(CMD_ANGLES, kinematicsAngle[0], kinematicsAngle[1],
        kinematicsAngle[2]);
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
    Serial.print(CMD_BEGIN);
    Serial.print(CMD_OMEGA);
    Serial.print(kp);
    Serial.print(',');
    Serial.print(ki);
    Serial.print(',');
    Serial.print(weight);
    Serial.print(';');
    accelCorrection.print();
    Serial.print(';');
    omegaP.print();
    Serial.print(';');
    omegaI.print();
    Serial.print(CMD_END);
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
  if (REPORTING & REPORT_SENSORS) {
    print3vi(CMD_SENSORS_GYRO, gyro.g.x, gyro.g.y, gyro.g.z);
  }

  compass.readAcc();
  if (REPORTING & REPORT_SENSORS) {
    print3vi(CMD_SENSORS_ACC, compass.a.x, compass.a.y, compass.a.z);
  }

  // update orientation
  updateOrientation();

  // update motor
  if (motors.armed) {
    float pitchT = pitchPID.computePID(kinematicsAngle[PITCH],
        targetAngles[PITCH], currentMicros);
    float rolT = rolPID.computePID(kinematicsAngle[ROL], targetAngles[ROL],
        currentMicros);
    float yawT = yawPID.computePID(kinematicsAngle[YAW], targetAngles[YAW],
        currentMicros);

    motors.pitch = stabilizePitchPID.computePID(pitchT, gyroGain.data[PITCH],
        currentMicros);
    motors.rol = stabilizeRolPID.computePID(rolT, gyroGain.data[ROL],
        currentMicros);
    motors.yaw = stabilizeYawPID.computePID(yawT, gyroGain.data[YAW],
        currentMicros);

    if (REPORTING & REPORT_STABILIZATION) {
      Serial.print(CMD_BEGIN);
      Serial.print(CMD_STABILIZATION);
      pitchPID.print();
      Serial.print(';');
      rolPID.print();
      Serial.print(';');
      yawPID.print();
      Serial.print(';');
      stabilizePitchPID.print();
      Serial.print(';');
      stabilizeRolPID.print();
      Serial.print(';');
      stabilizeYawPID.print();
      Serial.print(';');

      printVectorData(gyroGain.data);
      Serial.print(';');

      Serial.print(pitchT);
      Serial.print(',');
      Serial.print(rolT);
      Serial.print(',');
      Serial.print(yawT);
      Serial.print(';');

      Serial.print(motors.pitch);
      Serial.print(',');
      Serial.print(motors.rol);
      Serial.print(',');
      Serial.print(motors.yaw);
      Serial.print(CMD_END);
    }
  }

  // update motors with commands
  motors.updateMotors();
  if (REPORTING & REPORT_MOTORS) {
    motors.print();
  }
}

void loop() {
  uint32_t currentMillisTime = millis();
  currentMicros = micros();

  if (currentMillisTime > next50Hz) {
    if (REPORTING) {
      Serial.print('\n');
    }
    perform50HzActions();

    if (REPORTING & REPORT_TIME) {
      Serial.print('!');
      Serial.print(CMD_TIME);
      Serial.print(millis() - currentMillisTime);
      Serial.print("|\n");
    }

    groundStation.processCmds();

    next50Hz += DELAY_50HZ;
  }
}
