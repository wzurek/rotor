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
  cmd = CMD_BEGIN;
  cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
  handlersCount = 0;
}

void GroundStationComm::invokeCommand() {

  for (int i = 0; i < handlersCount; i++) {
    if (handlers[i].cmd == cmd) {
      handlers[i].action(cmdArg + 1);
      return;
    }
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
          cmd = CMD_BEGIN;
          cmd_index = 0;
          cmdArg[0] = 0;
          break;
        } else {
          if (c == CMD_END) {
            cmdArg[cmd_index] = 0;
            invokeCommand();
            cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
            cmd = CMD_BEGIN;
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

// communication
uint32_t GroundStationComm::appendVuint32(uint32_t val, uint8_t *buff) {
  buff[0] = static_cast<uint8_t>(val | 0x80);
  if (val >= (1 << 7)) {
    buff[1] = static_cast<uint8_t>((val >> 7) | 0x80);
    if (val >= (1 << 14)) {
      buff[2] = static_cast<uint8_t>((val >> 14) | 0x80);
      if (val >= (1 << 21)) {
        buff[3] = static_cast<uint8_t>((val >> 21) | 0x80);
        if (val >= (1 << 28)) {
          buff[4] = static_cast<uint8_t>(val >> 28);
          return 5;
        } else {
          buff[3] &= 0x7F;
          return 4;
        }
      } else {
        buff[2] &= 0x7F;
        return 3;
      }
    } else {
      buff[1] &= 0x7F;
      return 2;
    }
  } else {
    buff[0] &= 0x7F;
    return 1;
  }
}

// communication
uint32_t GroundStationComm::appendVuint16(uint16_t val, uint8_t *buff) {
  buff[0] = static_cast<uint8_t>(val | 0x80);
  if (val >= (1 << 7)) {
    buff[1] = static_cast<uint8_t>((val >> 7) | 0x80);
    if (val >= (1 << 14)) {
      buff[2] = static_cast<uint8_t>((val >> 14) | 0x80);
      if (val >= (1 << 21)) {
        buff[3] = static_cast<uint8_t>((val >> 21) | 0x80);
        if (val >= (1 << 28)) {
          buff[4] = static_cast<uint8_t>(val >> 28);
          return 5;
        } else {
          buff[3] &= 0x7F;
          return 4;
        }
      } else {
        buff[2] &= 0x7F;
        return 3;
      }
    } else {
      buff[1] &= 0x7F;
      return 2;
    }
  } else {
    buff[0] &= 0x7F;
    return 1;
  }
}

uint32_t GroundStationComm::appendVint32(int32_t val, uint8_t *buff) {
  return appendVuint32(static_cast<uint32_t>((val << 1) ^ (val >> 31)), buff);
}

uint32_t GroundStationComm::appendVint16(int16_t val, uint8_t *buff) {
  return appendVuint16(static_cast<uint16_t>((val << 1) ^ (val >> 15)), buff);
}

void GroundStationComm::registerCommand(char cmd, handlerPtr handler) {
  handlers[handlersCount].cmd = cmd;
  handlers[handlersCount].action = handler;
  handlersCount++;
}

void GroundStationComm::beginMessage(uint32_t messageId) {
  writeVInt(messageId);
}

void GroundStationComm::writeVIntField(uint32_t id, uint32_t val) {
  beginField(id, 0);
  writeVInt(val);
}

void GroundStationComm::writeVIntField(uint32_t id, int32_t val) {
  beginField(id, 0);
  writeVInt(val);
}

void GroundStationComm::writeFixedField(uint32_t id, uint32_t size, uint8_t *buff) {
  beginField(id, 3);
  writeVInt(size);
  writeBytes(buff, size);
}

void GroundStationComm::beginField(uint32_t id, uint8_t type) {
  writeVInt((id << 3) | (type & 0x07));
}

void GroundStationComm::writeVInt(uint32_t val) {
  uint8_t buff[5];
  size_t size = appendVuint32(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeVInt(int32_t val) {
  uint8_t buff[5];
  size_t size = appendVint32(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeBytes(uint8_t* buff, size_t size) {
  // write to serial port
  // for some reason eclipse does not recognize this function.
  Serial.write(buff, size);
}

void GroundStationComm::finishMessage() {
  // this is in fact, end of the message.
  beginField(0, 7);
}

void GroundStationComm::textMessage(const char* text) {
  beginMessage(1);
  beginField(1, 3);
  size_t size = strlen(text);
  writeVInt((uint32_t)size);
  writeBytes((uint8_t*) text, size);
  finishMessage();
}

void GroundStationComm::writeFloatField(uint32_t id, float value) {
  beginField(id, 4);
  writeFloat(value);
}

void GroundStationComm::writeFloatsField(uint32_t id, float values[], size_t size) {
  beginField(id, 2);
  writeVInt((uint32_t)size * 4);
  for (int i = 0; i < size; i++) {
    writeFloat(values[i]);
  }
}

void GroundStationComm::writeVIntsField(uint32_t id, uint32_t vals[], size_t size) {
  uint8_t buff[size * 5];
  uint8_t *b = buff;
  for (int i = 0; i < size; i++) {
    b += appendVint32(vals[i], b);
  }
  uint32_t buff_size = b - buff;

  beginField(id, 2);
  writeVInt(buff_size);
  writeBytes(buff, buff_size);
}

void GroundStationComm::writeFloat(float val) {
  writeBytes((uint8_t*) (&val), 4);
}

void GroundStationComm::writeVIntField(uint32_t id, uint16_t val) {

}

void GroundStationComm::writeVIntField(uint32_t id, int16_t val) {

}
