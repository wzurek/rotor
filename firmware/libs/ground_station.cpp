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
  cmd_index = 0;
  cmd = '!';
  cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
}

void GroundStationComm::invokeCommand() {

  switch(cmd) {
  case 'K':
    kinematicsGroundCommand();
    break;
  default:
    Serial.print("!C'");
    Serial.print(cmd);
    Serial.print("','");
    Serial.print(cmdArg + 1);
    Serial.print("'|");
  }
}

void GroundStationComm::processCmds() {

  if (Serial.available()) {
    size_t size = Serial.readBytes(cmd_buff, CMD_BUFF_MAX);
    buff_index = 0;

    while (buff_index < size) {
      int c = cmd_buff[buff_index++];

      switch (cmd_read_state) {

      case CMD_WAIT_FOR_CMD_BEGIN:
        if (c == CMD_BEGIN) {
          cmd_read_state = CMD_WAIT_FOR_CMD_END;
          cmd_index = 0;
          cmdArg[0] = 0;
          cmd = ' ';
        }
        break;

      case CMD_WAIT_FOR_CMD_END:
        if (cmd_index == 0) {
          cmd = c;
          cmdArg[cmd_index++] = c;
        } else if (cmd_index >= CMD_BUFF_MAX) {
          cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
          cmd = '!';
          cmd_index = 0;
          cmdArg[0] = 0;
          break;
        } else {
          if (c == CMD_END) {
            cmdArg[cmd_index] = 0;
            invokeCommand();
            cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
            cmd = '!';
            cmd_index = 0;
            cmdArg[0] = 0;
          } else {
            cmdArg[cmd_index++] = c;
          }
        }
        break;

      default:
        // should never happen
        cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
        cmd = ' ';
        cmd_index = 0;
        cmdArg[0] = 0;
      }
    }
  }
}

// global object
GroundStationComm groundStation;

