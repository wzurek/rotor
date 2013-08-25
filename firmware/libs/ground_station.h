/*
 * ground_station.h
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#ifndef GROUND_STATION_H_
#define GROUND_STATION_H_

#include <Arduino.h>

#define CMD_BUFF_MAX 50

class GroundStationComm {

  char cmd_buff[CMD_BUFF_MAX];
  uint32_t buff_index;
  uint32_t cmd_read_state;

  void invokeCommand();

public:
  GroundStationComm();
  void processCmds();

};

// global object
GroundStationComm groundStation;

#endif /* GROUND_STATION_H_ */
