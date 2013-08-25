/*
 * ground_station.cpp
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#include "ground_station.h"

#define CMD_WAIT_FOR_CMD_BEGIN 0
#define CMD_WAIT_FOR_CMD_END 1

#define CMD_BEGIN '!'
#define CMD_END '|'

GroundStationComm::GroundStationComm() {
  buff_index = 0;
  cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
}

void GroundStationComm::invokeCommand() {
}

void GroundStationComm::processCmds() {

  if (Serial.available()) {

    size_t size;
    switch (cmd_read_state) {

    case CMD_WAIT_FOR_CMD_BEGIN:
      size = Serial.readBytesUntil(CMD_BEGIN, cmd_buff, CMD_BUFF_MAX);
      if (size == 0) {
        return;
      } else if (cmd_buff[size - 1] == CMD_BEGIN) {
        cmd_read_state = CMD_WAIT_FOR_CMD_END;
      }
      break;

    case CMD_WAIT_FOR_CMD_END:
      size = Serial.readBytesUntil(CMD_BEGIN, cmd_buff + buff_index,
          CMD_BUFF_MAX - buff_index);
      buff_index += size;
      if (buff_index >= CMD_BUFF_MAX) {
        cmd_read_state = CMD_WAIT_FOR_CMD_END;
        buff_index = 0;
      } else if (cmd_buff[buff_index - 1] == CMD_END) {
        invokeCommand();
        cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
        buff_index = 0;
      }
      break;

    default:
      // should never happen
      cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
    }
  }
}
