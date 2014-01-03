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

#define MAX_HANDLERS 10

// IDs os commands
#define CMD_TEXT 'P'
#define CMD_SENSORS 'S'
#define CMD_SENSORS_GYRO_ALL 'G'
#define CMD_RECEIVER 'R'
#define CMD_ANGLES 'E'
#define CMD_MOTORS 'M'
#define CMD_TIME 'T'
#define CMD_DCM 'D'
#define CMD_STABILIZATION 'Q'
#define CMD_OMEGA 'O'
#define CMD_CONFIGURATION 'C'


typedef void (*handlerPtr)(const uint8_t*, size_t size);

struct Handler {
  char cmd;
  handlerPtr action;
};

class GroundStationComm {

  uint8_t cmd_buff[CMD_BUFF_MAX];
  uint8_t cmdArg[CMD_BUFF_MAX];
  char cmd;

  uint32_t buff_index;
  uint32_t cmd_index;
  uint32_t cmd_size;
  uint32_t cmd_read_state;

  uint32_t sentBytes;

  uint32_t handlersCount;
  Handler handlers[MAX_HANDLERS];

  void invokeCommand();


public:
  /**
   * The constructor.
   */
  GroundStationComm();

  /**
   * Register handler for gice command type
   */
  void registerCommand(char cmd, handlerPtr handler);

  /**
   * Parse VUint from the buffer, returns size of the parsed VInt
   */
  size_t parseVUint32(const uint8_t* buff, size_t size, uint32_t* vint_result);

  // for constructing messages
  size_t appendVuint32(uint32_t val, uint8_t *buff);
  size_t appendVint32(int32_t val, uint8_t *buff);
  size_t appendVuint16(uint16_t val, uint8_t *buff);
  size_t appendVint16(int16_t val, uint8_t *buff);

  /**
   * Check if there are any commands from GroundSatation
   */
  void processCmds();

  /**
   * Send text message to ground station.
   */
  void textMessage(const char* text);
  void textMessage(const char* text, size_t size);

  /**
   * Begins the message with given ID
   */
  void beginMessage(uint32_t messageId);
  void finishMessage();

  // Write fields of given types.
  void writeVUInt32Field(uint32_t id, uint32_t val);
  void writeVInt32Field(uint32_t id, int32_t val);

  void writeVUInt16Field(uint32_t id, uint16_t val);
  void writeVInt16Field(uint32_t id, int16_t val);

  void writeVIntsField(uint32_t id, int32_t vals[], size_t size);
  void writeVUIntsField(uint32_t id, uint32_t vals[], size_t size);

  void writeFloatField(uint32_t id, float value);
  void writeFloatsField(uint32_t id, float value[], size_t size);
  void writeFixedField(uint32_t id, uint32_t size, uint8_t *buff);

  // ------ writing lower level stuff
  void beginField(uint32_t id, uint8_t type);

  // ------ writing bytes
  void writeVUInt16(uint16_t val);
  void writeVUInt32(uint32_t val);

  void writeVInt16(int16_t val);
  void writeVInt32(int32_t val);

  void writeFloat(float val);
  void writeBytes(uint8_t *buff, size_t size);

  /**
   * How many bytes has been sent since last 'bytesSent()' call.
   */
  uint32_t bytesSent();
};

// global object
extern GroundStationComm groundStation;

#endif /* GROUND_STATION_H_ */
