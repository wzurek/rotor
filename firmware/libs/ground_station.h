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

  void writeVIntField(uint32_t id, uint32_t val);
  void writeVIntField(uint32_t id, int32_t val);
  void writeVIntField(uint32_t id, uint16_t val);
  void writeVIntsField(uint32_t id, uint32_t vals[], size_t size);
  void writeVIntField(uint32_t id, int16_t val);
  void writeFloatField(uint32_t id, float value);
  void writeFloatsField(uint32_t id, float value[], size_t size);
  void writeFixedField(uint32_t id, uint32_t size, uint8_t *buff);

  void beginField(uint32_t id, uint8_t type);
  void writeVInt(uint32_t val);
  void writeVInt(int32_t val);
  void writeFloat(float val);
  void writeBytes(uint8_t *buff, size_t size);

  void finishMessage();
};

// global object
extern GroundStationComm groundStation;

#endif /* GROUND_STATION_H_ */
