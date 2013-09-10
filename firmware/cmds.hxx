#include "global_objects.h"

void cmdUpdateReporting(char* arg) {
  REPORTING = 0;
  int i = 0;

  while (*arg) {
    REPORTING <<= 1;
    if (*arg != '0') {
      REPORTING |= 1;
    }
    arg++;
  }

  Serial.print("!C");
  Serial.print(REPORTING);
  Serial.print("|");
}

void cmdMotors(char* arg) {
  switch (*arg) {
  case 'D': // dis-arm
    motors.disarm();
    break;
  case 'A': // arm
    motors.arm();
    break;
  }
}

void cmdUpdateAccelError(char* arg) {
  float x = strtof(arg, &arg);
  if (*arg != ',')
    return;
  arg++;

  float y = strtof(arg, &arg);
  if (*arg != ',')
    return;
  arg++;

  float z = strtof(arg, &arg);

  accelCorrection.data[XAXIS] = x;
  accelCorrection.data[YAXIS] = y;
  accelCorrection.data[ZAXIS] = z;
}

void cmdUpdatePid(char* arg) {
  int index = *arg - '0';
  if (index < 0 || index > 6) {
    Serial.print("!Cinvalid PID: ");
    Serial.print(index);
    Serial.print(CMD_END);
    return;
  }
  arg++;

  char* end;
  float p, i, d;
  bool reset;

  p = strtof(arg, &end);
  if (*end != ',')
    return;

  i = strtof(++end, &end);
  if (*end != ',')
    return;

  d = strtof(++end, &end);
  if (*end != ',')
    return;

  reset = strtol(end, NULL, 10);

  PID* pid = pids[index];
  if (reset) {
    pid->integral = 0;
  }
  pid->p = p;
  pid->i = i;
  pid->d = d;

  Serial.print("!Cupdated PID ");
  Serial.print(index);
  Serial.print(':');
  Serial.print(p);
  Serial.print(',');
  Serial.print(i);
  Serial.print(',');
  Serial.print(d);
  Serial.print(',');
  Serial.print(reset);
  Serial.print('|');
}
