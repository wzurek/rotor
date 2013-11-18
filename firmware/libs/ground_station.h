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

#define FIELD_VINT 0
#define FIELD_FIXED_LEN 2
#define FIELD_32BIT 5
#define FIELD_END 7

// IDs os commands
#define CMD_TEXT 1
#define CMD_TIMER 2
#define CMD_GYRO 3
#define CMD_MOTOR 4
#define CMD_RECEIVER 5

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

  uint32_t appendVuint32(uint32_t val, uint8_t *buff);
  uint32_t appendVint32(int32_t val, uint8_t *buff);
  uint32_t appendVuint16(uint16_t val, uint8_t *buff);
  uint32_t appendVint16(int16_t val, uint8_t *buff);

public:
  GroundStationComm();

  void processCmds();

  void registerCommand(char cmd, handlerPtr handler);

  // messages

  void textMessage(const char* text);

  void beginMessage(uint32_t messageId);

  void writeVUInt32Field(uint32_t id, uint32_t val);
  void writeVInt32Field(uint32_t id, int32_t val);

  void writeVUInt16Field(uint32_t id, uint16_t val);
  void writeVInt16Field(uint32_t id, int16_t val);

  void writeVIntsField(uint32_t id, uint32_t vals[], size_t size);

  void writeFloatField(uint32_t id, float value);
  void writeFloatsField(uint32_t id, float value[], size_t size);
  void writeFixedField(uint32_t id, uint32_t size, uint8_t *buff);

  void beginField(uint32_t id, uint8_t type);

  void writeVUInt16(uint16_t val);
  void writeVUInt32(uint32_t val);

  void writeVInt16(int16_t val);
  void writeVInt32(int32_t val);

  void writeFloat(float val);
  void writeBytes(uint8_t *buff, size_t size);

  void finishMessage();
};

// global object
extern GroundStationComm groundStation;

#endif /* GROUND_STATION_H_ */
