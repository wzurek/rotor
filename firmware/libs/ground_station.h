/*
 * ground_station.h
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#ifndef GROUND_STATION_H_
#define GROUND_STATION_H_

#include <Arduino.h>
#include "global_objects.h"

#define CMD_BUFF_MAX 50

typedef void (*handlerPtr)(char[]);

struct Handler {
  char cmd;
  handlerPtr action;
};

class GroundStationComm {

  char cmd_buff[CMD_BUFF_MAX];
  char cmdArg[CMD_BUFF_MAX];
  char cmd;

  uint32_t buff_index;
  uint32_t cmd_index;
  uint32_t cmd_read_state;

  uint32_t handlersCount;
  Handler handlers[10];

  void invokeCommand();

public:
  GroundStationComm();

  void processCmds();

  void registerCommand(char cmd, handlerPtr handler);
};

// global object
extern GroundStationComm groundStation;

#endif /* GROUND_STATION_H_ */
