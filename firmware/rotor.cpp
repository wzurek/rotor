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

#define MODEL_VERSION "Rotor 001"

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

PID accelPID(0.35f, 0.0f, 0.0f);

// the values based on experiments
Vector3f accelCorrection(-7.8, -16.1, 0);

PID pitchPID(1, 0.0, 0.5);
PID rolPID(1, 0.0, 0.5);
PID yawPID(1, 0.0, 0.5);

// scale of the gyro component for PID
float gyroScale = 0.4;
float anglesScale = 0.4;

// values for the mixin alghorithm type
#define MIXIN_TYPE_ORIENTATION 1
#define MIXIN_TYPE_ORIENTATION_GYRO 2
#define MIXIN_TYPE_GYRO 3

int32_t mixin_type = 1;

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

uint32_t REPORTING = REPORT_ANGLES | REPORT_TIME | REPORT_MOTORS | REPORT_SENSORS | REPORT_OMEGA; // REPORT_RECEIVER; //|REPORT_SENSORS;// | REPORT_RECEIVER; // | REPORT_STABILIZATION | REPORT_RECEIVER

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

void cmdSendConfiguration(const uint8_t* arg, size_t size) {
  groundStation.beginMessage(CMD_CONFIGURATION);
  groundStation.writeFloatField(1, anglesScale);
  groundStation.writeFloatField(2, gyroScale);
  float pid[] = { pitchPID.p, pitchPID.i, pitchPID.d };
  groundStation.writeFloatsField(3, pid, 3);
  groundStation.writeVUInt32Field(4, THROTTLE_MAX);
  groundStation.writeVUInt32Field(5, PITCH_MAX);
  groundStation.finishMessage();
}

#define TYPE_INT 1
#define TYPE_FLOAT 2
#define TYPE_PID 3
#define TYPE_SEPARATOR 4
#define TYPE_XYZ_FLOAT 5
#define TYPE_XYZ_INT 6
#define TYPE_32BIT 7

#define CONF_ID_GS_REPORTING 5

#define CONF_ID_MIXIN_GROUP 9
#define CONF_ID_MIXIN_TYPE 10
#define CONF_ID_MIXIN_ORIENTATION_SCALE 11
#define CONF_ID_MIXIN_GYRO_SCALE 12
#define CONF_ID_MIXIN_PID 13
#define CONF_ID_MIXIN_SPREAD 14

#define CONF_ID_RECEIVER_GROUP 19
#define CONF_ID_THROTTLE_MAX 20

#define CONF_ID_ORIENTATION_GROUP 29
#define CONF_ID_ACCEL_PID 30
#define CONF_ID_ACCEL_CORRECTION 31
#define CONF_ID_GYRO_CORRECTION 33

void writeConfLineDesc(uint32_t fieldId, uint8_t type, const char* desc) {
  char buff[100];
  buff[0] = type;
  size_t len = strlen(desc);

  strcpy(buff + 1, desc);
  groundStation.writeFixedField(fieldId, len + 1, (uint8_t*) buff);
}

void cmdSendConfigurationListDesc() {
  groundStation.beginMessage(CMD_CONFIGURATION_LIST);

  // write the model name and version on first field
  groundStation.writeFixedField(1, strlen(MODEL_VERSION), (uint8_t*) MODEL_VERSION);
  groundStation.writeVUInt32Field(2, 0);

  writeConfLineDesc(CONF_ID_GS_REPORTING, TYPE_32BIT,
      "Reporting|Sensors|Receiver|Orientation|Motors|Time|DCM|Stabilization|Omega");

  writeConfLineDesc(CONF_ID_MIXIN_GROUP, TYPE_SEPARATOR, "Mixing orientation to motors");
  writeConfLineDesc(CONF_ID_MIXIN_TYPE, TYPE_INT, "Mixin type|1:Orientation only|2:Orientation and Gyro|3:Gyro only");
  writeConfLineDesc(CONF_ID_MIXIN_ORIENTATION_SCALE, TYPE_FLOAT, "Mixin orientation scale");
  writeConfLineDesc(CONF_ID_MIXIN_GYRO_SCALE, TYPE_FLOAT, "Mixin gyro scale");
  writeConfLineDesc(CONF_ID_MIXIN_PID, TYPE_PID, "Mixin PID");
  writeConfLineDesc(CONF_ID_MIXIN_SPREAD, TYPE_INT, "Mixin spead spread MAX");

  writeConfLineDesc(CONF_ID_RECEIVER_GROUP, TYPE_SEPARATOR, "Receiver settings");
  writeConfLineDesc(CONF_ID_THROTTLE_MAX, TYPE_INT, "Throttle MAX");

  writeConfLineDesc(CONF_ID_ORIENTATION_GROUP, TYPE_SEPARATOR, "Orientation parameters");
  writeConfLineDesc(CONF_ID_ACCEL_PID, TYPE_PID, "Accel PID");
  writeConfLineDesc(CONF_ID_ACCEL_CORRECTION, TYPE_XYZ_FLOAT, "Accel base");
  writeConfLineDesc(CONF_ID_GYRO_CORRECTION, TYPE_XYZ_INT, "Gyro base");

  groundStation.finishMessage();
}

void cmdSendConfigurationListValues() {
  groundStation.beginMessage(CMD_CONFIGURATION_LIST);

  // write the model name and version on first field
  groundStation.writeFixedField(1, strlen(MODEL_VERSION), (uint8_t*) MODEL_VERSION);
  groundStation.writeVUInt32Field(2, 1);

  groundStation.writeVInt32Field(CONF_ID_GS_REPORTING, REPORTING);

  groundStation.writeVInt32Field(CONF_ID_MIXIN_TYPE, mixin_type);
  groundStation.writeFloatField(CONF_ID_MIXIN_ORIENTATION_SCALE, anglesScale);
  groundStation.writeFloatField(CONF_ID_MIXIN_GYRO_SCALE, gyroScale);
  pitchPID.print(CONF_ID_MIXIN_PID);
  groundStation.writeVUInt32Field(CONF_ID_MIXIN_SPREAD, PITCH_MAX);

  groundStation.writeVUInt32Field(CONF_ID_THROTTLE_MAX, THROTTLE_MAX);

  accelPID.print(CONF_ID_ACCEL_PID);
  groundStation.writeFloatsField(CONF_ID_ACCEL_CORRECTION, accelCorrection.data, 3);

  int32_t gyro_base[] = { gyro.base.x, gyro.base.y, gyro.base.z };
  groundStation.writeVIntsField(CONF_ID_GYRO_CORRECTION, gyro_base, 3);

  groundStation.finishMessage();
}

void cmdSetConfigurationListValues(const uint8_t* arg, size_t size) {

  size_t data_size = size - 1;
  uint32_t fieldId = arg[0];
  const uint8_t* data = arg + 1;
  bool correct = false;

  switch (fieldId) {
  case CONF_ID_MIXIN_TYPE: {
    if (data_size > 0 && data_size <= 5) {

      int32_t type;
      size_t len = groundStation.parseVSint32(data, data_size, &type);
      if (data_size == len) {
        mixin_type = type;
        correct = true;
      }

    }
    break;
  }
  case CONF_ID_GS_REPORTING: {
    if (data_size > 0 && data_size <= 5) {
      int32_t reporting;
      size_t len = groundStation.parseVSint32(data, data_size, &reporting);
      if (data_size == len) {
        REPORTING = reporting;
        correct = true;
      }

    }
    break;
  }
  case CONF_ID_MIXIN_ORIENTATION_SCALE: {
    if (data_size == 4) {
      correct = true;
      anglesScale = *((float*) data);
    }
    break;
  }
  case CONF_ID_MIXIN_GYRO_SCALE: {
    if (data_size == 4) {
      correct = true;
      gyroScale = *((float*) data);
    }
    break;
  }
  case CONF_ID_MIXIN_PID: {
    if (data_size == 12) {
      correct = true;
      const float* pid = (const float*) data;

      pitchPID.p = pid[0];
      pitchPID.i = pid[1];
      pitchPID.d = pid[2];

      rolPID.p = pid[0];
      rolPID.i = pid[1];
      rolPID.d = pid[2];

      yawPID.p = pid[0];
      yawPID.i = pid[1];
      yawPID.d = pid[2];
    }
    break;
  }
  case CONF_ID_MIXIN_SPREAD: {
    if (data_size > 0 && data_size <= 5) {
      correct = true;
    }
    break;
  }
  case CONF_ID_THROTTLE_MAX: {
    if (data_size > 0 && data_size <= 5) {
      correct = true;
    }
    break;
  }
  case CONF_ID_ACCEL_PID: {
    if (data_size == 12) {
      correct = true;
      const float* pid = (const float*) data;

      accelPID.p = pid[0];
      accelPID.i = pid[1];
      accelPID.d = pid[2];
    }
    break;
  }
  case CONF_ID_ACCEL_CORRECTION: {
    if (data_size == 12) {
      correct = true;
      const float* pid = (const float*) data;

      accelCorrection.data[0] = pid[0];
      accelCorrection.data[1] = pid[1];
      accelCorrection.data[2] = pid[2];
    }
    break;
  }
  case CONF_ID_GYRO_CORRECTION: {
    if (data_size > 0 && data_size < 15) {
      int32_t x, y, z;
      size_t len;

      len = groundStation.parseVSint32(data, data_size, &x);
      len += groundStation.parseVSint32(data + len, data_size - len, &y);
      len += groundStation.parseVSint32(data + len, data_size - len, &z);

      if (len == data_size) {
        correct = true;
        gyro.base.x = x;
        gyro.base.y = y;
        gyro.base.z = z;
      }
    }
    break;
  }
  }
  if (!correct) {
    char buff[100];
    sprintf(buff, "Unknown set field %d, size %d.", fieldId, size);
    groundStation.textMessage(buff);
  } else {
//    sprintf(buff, "Conf: %d (%d)", fieldId, size);
  }
}

void cmdSendConfigurationList(const uint8_t* arg, size_t size) {
  switch (*arg) {
  case 'D': // send descriptions
    cmdSendConfigurationListDesc();
    break;
  case 'V': // send values
    cmdSendConfigurationListValues();
    break;
  case 'S': // send values
    cmdSetConfigurationListValues(arg + 1, size - 1);
    break;
  default:
    groundStation.textMessage("Unknown configuration command.");
  }
}

void cmdSetConfiguration(const uint8_t* arg, size_t size) {

  if (size == 20) {
    anglesScale = *(float*) arg;
    gyroScale = *(float*) (arg + 4);
    pitchPID.p = *(float*) (arg + 8);
    pitchPID.i = *(float*) (arg + 12);
    pitchPID.d = *(float*) (arg + 16);

    rolPID.p = pitchPID.p;
    rolPID.i = pitchPID.i;
    rolPID.d = pitchPID.d;
    yawPID.p = pitchPID.p;
    yawPID.i = pitchPID.i;
    yawPID.d = pitchPID.d;
  }
  groundStation.textMessage("Received configuration");

  cmdSendConfiguration(NULL, 0);
}

void cmdRotorConfig(const uint8_t* arg, size_t size) {
  anglesScale = *(float*) arg;
  gyroScale = *(float*) (arg + 4);
  pitchPID.p = *(float*) (arg + 8);
  pitchPID.i = *(float*) (arg + 12);
  pitchPID.d = *(float*) (arg + 16);
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

  groundStation.textMessage(MODEL_VERSION);

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
  groundStation.registerCommand('R', cmdUpdateReporting);
  groundStation.registerCommand('E', cmdEcho);
  groundStation.registerCommand('C', cmdRotorConfig);
  groundStation.registerCommand('G', cmdSendConfiguration);
  groundStation.registerCommand('S', cmdSetConfiguration);
  groundStation.registerCommand('L', cmdSendConfigurationList);
  groundStation.registerCommand('M', cmdMotors);
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

  if (motors.get_mode() == MOTOR_MODE_SAFE_START) {
    // continue failsafe mode until the stick is on low position at least once.
    if (!swOn && receiver.connected) {
      motors.init();
    }
  } else {
    // motors are initialized, but maybe not armed
    if (swOn && !motors.isArmed()) {
      // not armed and switch is on - arm the motors
      motors.arm();
      motors.operation();

    } else if (swOn) {
      // armed and switch is on - normal operation
      motors.throttle = mapThrottle(channels[CH_THROTTLE]);
      targetAngles[PITCH] = mapStick(channels[CH_PITCH]) * MAX_ANGLE;
      targetAngles[ROL] = -mapStick(channels[CH_ROL]) * MAX_ANGLE;
      targetAngles[YAW] = mapStick(channels[CH_YAW]) * MAX_ANGLE;

    } else {
      // armed and switch off - desired angle to level and throttle to minimum value
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

#define GRAVITY 245.0f
void computeOmegas() {

  Vector3f acc(-compass.a.x, -compass.a.y, -compass.a.z);
  acc.substract(&accelCorrection);

  float weight = 1.0f;
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
    groundStation.beginMessage(CMD_OMEGA);
    groundStation.writeFloatsField(1, omegaP.data, 3);
    groundStation.writeFloatsField(2, omegaI.data, 3);
    groundStation.writeFloatsField(3, err.data, 2);
    groundStation.writeFloatField(4, weight);
    groundStation.finishMessage();
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
  if (motors.isArmed()) {

    switch (mixin_type) {
    case MIXIN_TYPE_ORIENTATION: {

      // ------- location only PID
      motors.pitch = pitchPID.computePID(kinematicsAngle[PITCH], targetAngles[PITCH], currentMicros);
      motors.rol = rolPID.computePID(kinematicsAngle[ROL], targetAngles[ROL], currentMicros);
      motors.yaw = yawPID.computePID(kinematicsAngle[YAW], targetAngles[YAW], currentMicros);

      break;
    }
    case MIXIN_TYPE_ORIENTATION_GYRO: {

      // mixin gyro and orientation
      motors.pitch = pitchPID.computePID(kinematicsAngle[PITCH] * anglesScale + gyroGain.data[PITCH] * gyroScale,
          targetAngles[PITCH], currentMicros);
      motors.rol = rolPID.computePID(kinematicsAngle[ROL] * anglesScale + gyroGain.data[ROL] * gyroScale,
          targetAngles[ROL], currentMicros);
      motors.yaw = yawPID.computePID(kinematicsAngle[YAW] * anglesScale + gyroGain.data[YAW] * gyroScale,
          targetAngles[YAW], currentMicros);
      break;
    }
    case MIXIN_TYPE_GYRO: {

      // Only gyro input
      motors.pitch = pitchPID.computePID(gyroGain.data[PITCH] * gyroScale, targetAngles[PITCH], currentMicros);
      motors.rol = rolPID.computePID(gyroGain.data[ROL] * gyroScale, targetAngles[ROL], currentMicros);
      motors.yaw = yawPID.computePID(gyroGain.data[YAW] * gyroScale, targetAngles[YAW], currentMicros);
      break;
    }
    default: {
      // full manual control
      motors.pitch = targetAngles[PITCH];
      motors.rol = targetAngles[ROL];
      motors.yaw = targetAngles[YAW];
      break;
    }
    }

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

int sec = 50;

void loop() {
  uint32_t currentMillisTime = millis();
  currentMicros = micros();

  if (currentMillisTime > next50Hz) {
    groundStation.processCmds();

    perform50HzActions();

    if (REPORTING & REPORT_TIME) {
      groundStation.beginMessage(CMD_TIME);
      groundStation.writeVUInt32Field(1, currentMillisTime);
      groundStation.writeVUInt32Field(2, millis() - currentMillisTime);
      groundStation.writeVUInt32Field(3, groundStation.bytesSent());
      groundStation.finishMessage();
    }

    next50Hz += DELAY_50HZ;
  }
}
