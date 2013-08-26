/*
 * Arduino.h
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#include <stdint.h>
#include <stdlib.h>

class SerialMock {

  char* input;
  size_t loc;

public:
  // mock
  void setInput(char* input);

  // real
  uint32_t available();
  size_t readBytesUntil(char terminator, char *buffer, size_t length);
};

extern SerialMock Serial;


#endif /* ARDUINO_H_ */
