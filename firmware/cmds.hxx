#include "global_objects.h"

void cmdUpdateReporting(const uint8_t* arg, size_t size) {
  uint32_t flags;
  groundStation.parseVUint32(arg, size, &flags);
  REPORTING = flags;
}

void cmdMotors(const uint8_t* arg, size_t size) {

  switch (*arg) {
  case 'S': // stop the motors
    motors.stop();
    break;
  case 'O': // normal operation
    motors.operation();
    break;
  case 'D': { // directly set the speed of motors
    motors.direct();
    size_t shift = 0;

    shift  = groundStation.parseVUint32(arg + 1        , size - 1        , motors.motor_throttle    );
    shift += groundStation.parseVUint32(arg + 1 + shift, size - 1 - shift, motors.motor_throttle + 1);
    shift += groundStation.parseVUint32(arg + 1 + shift, size - 1 - shift, motors.motor_throttle + 2);
             groundStation.parseVUint32(arg + 1 + shift, size - 1 - shift, motors.motor_throttle + 3);

    char buff[100];
    snprintf(buff, 100, "Set direct motor: FL(%d) FR(%d) RL(%d) RR(%d)",
         motors.motor_throttle[FRONT_LEFT],
         motors.motor_throttle[FRONT_RIGHT],
         motors.motor_throttle[REAR_LEFT],
         motors.motor_throttle[REAR_RIGHT]);

    groundStation.textMessage(buff);
    break;
  }
  case 'A': // init/arm motor controllers
    motors.init();
    motors.arm();
    motors.direct();
    break;
  default:
    groundStation.textMessage("Unknown motors command.");
  }
}

void cmdUpdateGyroScale(const uint8_t* arg, size_t size) {
}

void cmdUpdatePid(const uint8_t* arg, size_t size) {
}

void cmdEcho(const uint8_t* arg, size_t size) {
  // send back the text
  groundStation.textMessage((char*) arg, size);
}
