/*
 * ground_station.cpp
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#include "ground_station.h"

#define CMD_WAIT_FOR_CMD_BEGIN 0
#define CMD_WAIT_FOR_CMD_END 1
#define CMD_WAIT_FOR_CMD_SIZE 2

#define RESET_CHAR 'A'

GroundStationComm::GroundStationComm() {
  buff_index = 0;
  cmd_index = 0;
  cmd = 0;
  cmd_size = 0;
  cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
  sentBytes = 0;
  handlersCount = 0;
  resetCharCount = 0;
}

void GroundStationComm::invokeCommand() {

//  char msg[3];
//  msg[0] = '-';
//  msg[1] = cmd;
//  msg[2] = 0;
//  textMessage(msg);

  for (int i = 0; i < handlersCount; i++) {
    if (handlers[i].cmd == cmd) {
      handlers[i].action(cmdArg, cmd_size);
      return;
    }
  }
  char buff[handlersCount + 1];
  for (int i = 0; i < handlersCount; i++) {
    buff[i] = handlers[i].cmd;
  }
  buff[handlersCount-1] = 0;
  textMessage("Unknown message. Known messages:");
  textMessage(buff);
}

void GroundStationComm::processCmds() {

//  size_t available = Serial.available();
//  if (available > 0) {

//    if (available > CMD_BUFF_MAX) {
//      available = CMD_BUFF_MAX;
//    }

//    size_t size = Serial.readBytes((char*)cmd_buff, available);
//    buff_index = 0;

    while (Serial.available()) {
      int c = Serial.read();

      if (c == RESET_CHAR) {
        resetCharCount++;
        char buff[10];
        sprintf(buff, "A:%d", resetCharCount);
        textMessage(buff);
      } else {
        if (resetCharCount > 10) {
          cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
          cmd = ' ';
          cmd_index = 0;
          cmdArg[0] = 0;

          textMessage("Message reset.");
        }
        resetCharCount = 0;
      }

      switch (cmd_read_state) {

      case CMD_WAIT_FOR_CMD_BEGIN:
        cmd = c;
        cmd_read_state = CMD_WAIT_FOR_CMD_SIZE;
        break;

      case CMD_WAIT_FOR_CMD_SIZE:
        cmd_size = c;
        cmd_index = 0;
        if (cmd_size > 0) {
          cmd_read_state = CMD_WAIT_FOR_CMD_END;
        } else {
          cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
          invokeCommand();
        }
        break;

      case CMD_WAIT_FOR_CMD_END:
        cmdArg[cmd_index++] = c;

        if (cmd_index >= cmd_size) {
          cmdArg[cmd_index] = 0;
          cmd_read_state = CMD_WAIT_FOR_CMD_BEGIN;
          invokeCommand();
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
//  }
}

// global object
GroundStationComm groundStation;

size_t GroundStationComm::parseVSint32(const uint8_t* buff, size_t size, int32_t* vint) {
  uint32_t resultVal;
  size_t result;
  result = parseVUint32(buff, size, &resultVal);
  *vint = (resultVal >> 1) | ((resultVal & 1) << 31);

  return result;
}

size_t GroundStationComm::parseVUint32(const uint8_t* buff, size_t size, uint32_t* vint) {
  if (buff[0] & 0x80) {
    *vint = buff[0] & 0x7f;
    if (buff[1] & 0x80) {
      *vint |= (buff[1] & 0x7f) << 7;
      if (buff[2] & 0x80) {
        *vint |= (buff[2] & 0x7f) << 14;
        if (buff[3] & 0x80) {
          *vint |= (buff[3] & 0x7f) << 21;
          *vint |= (buff[4] & 0x7f) << 28;
          return 5;
        }
        *vint |= buff[3] << 21;
        return 4;
      }
      *vint |= buff[2] << 14;
      return 3;
    }
    *vint |= buff[1] << 7;
    return 2;
  }
  *vint = buff[0];
  return 1;
}

// communication
size_t GroundStationComm::appendVuint32(uint32_t val, uint8_t *buff) {
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
size_t GroundStationComm::appendVuint16(uint16_t val, uint8_t *buff) {
  buff[0] = static_cast<uint8_t>(val | 0x80);
  if (val >= (1 << 7)) {
    buff[1] = static_cast<uint8_t>((val >> 7) | 0x80);
    if (val >= (1 << 14)) {
      buff[2] = static_cast<uint8_t>(val >> 14);
      return 3;
    } else {
      buff[1] &= 0x7F;
      return 2;
    }
  } else {
    buff[0] &= 0x7F;
    return 1;
  }
}

size_t GroundStationComm::appendVint32(int32_t val, uint8_t *buff) {
  return appendVuint32(static_cast<uint32_t>((val << 1) ^ (val >> 31)), buff);
}

size_t GroundStationComm::appendVint16(int16_t val, uint8_t *buff) {
  return appendVuint16(static_cast<uint16_t>((val << 1) ^ (val >> 15)), buff);
}

void GroundStationComm::registerCommand(char cmd, handlerPtr handler) {
  handlers[handlersCount].cmd = cmd;
  handlers[handlersCount].action = handler;
  handlersCount++;
}

void GroundStationComm::beginMessage(uint32_t messageId) {
  writeVUInt32(messageId);
}

void GroundStationComm::writeVUInt32Field(uint32_t id, uint32_t val) {
  beginField(id, FIELD_VINT);
  writeVUInt32(val);
}

void GroundStationComm::writeVInt32Field(uint32_t id, int32_t val) {
  beginField(id, FIELD_VINT);
  writeVInt32(val);
}

void GroundStationComm::writeFixedField(uint32_t id, uint32_t size, uint8_t *buff) {
  beginField(id, FIELD_FIXED_LEN);
  writeVUInt32(size);
  writeBytes(buff, size);
}

void GroundStationComm::beginField(uint32_t id, uint8_t type) {
  writeVUInt32((id << 3) | (type & 0x07));
}

void GroundStationComm::writeVUInt32(uint32_t val) {
  uint8_t buff[5];
  size_t size = appendVuint32(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeVUInt16(uint16_t val) {
  uint8_t buff[3];
  size_t size = appendVuint16(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeVInt32(int32_t val) {
  uint8_t buff[5];
  size_t size = appendVint32(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeVInt16(int16_t val) {
  uint8_t buff[5];
  size_t size = appendVint16(val, buff);
  writeBytes(buff, size);
}

void GroundStationComm::writeBytes(uint8_t* buff, size_t size) {
  // write to serial port
  // for some reason eclipse does not recognize this function.
  //  Serial.write(buff, size);

  for(size_t i = 0; i < size; i++) {
    Serial.write(buff[i]);
  }

  // for communication stats
  sentBytes += size;
}

void GroundStationComm::finishMessage() {
  // this is in fact, end of the message.
  beginField(0, FIELD_END);
  Serial.flush();
}

void GroundStationComm::textMessage(const char* text) {
  size_t size = strlen(text);
  textMessage(text, size);
}

void GroundStationComm::textMessage(const char* text, size_t size) {
  beginMessage(CMD_TEXT);
  beginField(1, FIELD_FIXED_LEN);
  writeVUInt32((uint32_t) size);
  writeBytes((uint8_t*) text, size);
  finishMessage();
}

void GroundStationComm::writeFloatField(uint32_t id, float value) {
  beginField(id, FIELD_32BIT);
  writeFloat(value);
}

void GroundStationComm::writeFloatsField(uint32_t id, float values[], size_t size) {
  beginField(id, FIELD_FIXED_LEN);
  writeVUInt32((uint32_t) size * 4);
  for (int i = 0; i < size; i++) {
    writeFloat(values[i]);
  }
}

void GroundStationComm::writeVUIntsField(uint32_t id, uint32_t vals[], size_t size) {
  uint8_t buff[size * 5];
  uint8_t *b = buff;
  for (int i = 0; i < size; i++) {
    b += appendVuint32(vals[i], b);
  }
  uint32_t buff_size = b - buff;

  beginField(id, FIELD_FIXED_LEN);
  writeVUInt32(buff_size);
  writeBytes(buff, buff_size);
}

void GroundStationComm::writeVIntsField(uint32_t id, int32_t vals[], size_t size) {
  uint8_t buff[size * 5];
  uint8_t *b = buff;
  for (int i = 0; i < size; i++) {
    b += appendVint32(vals[i], b);
  }
  uint32_t buff_size = b - buff;

  beginField(id, FIELD_FIXED_LEN);
  writeVUInt32(buff_size);
  writeBytes(buff, buff_size);
}

void GroundStationComm::writeFloat(float val) {
  writeBytes((uint8_t*) (&val), 4);
}

void GroundStationComm::writeVUInt16Field(uint32_t id, uint16_t val) {
  beginField(id, FIELD_VINT);
  writeVUInt16(val);
}

uint32_t GroundStationComm::bytesSent() {
  uint32_t result = sentBytes;
  sentBytes = 0;
  return result;
}

void GroundStationComm::writeVInt16Field(uint32_t id, int16_t val) {
  beginField(id, FIELD_VINT);
  writeVInt16(val);
}

