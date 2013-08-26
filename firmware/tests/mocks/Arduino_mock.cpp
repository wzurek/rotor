/*
 * Arduino_mock.cpp
 *
 *  Created on: Aug 25, 2013
 *      Author: wzurek
 */

#include "Arduino.h"
#include <iostream>

using namespace std;

// global serial object
SerialMock Serial;

uint32_t SerialMock::available() {
  return input[loc];
}

void SerialMock::setInput(char* input) {
  loc = 0;
  this->input = input;
}

size_t SerialMock::readBytesUntil(char terminator, char* buffer,
    size_t length) {
  int i;
  for (i = 0; i < length; i++) {
    char c = input[loc + i];
    if (c) {
      buffer[i] = c;
    } else {
      break;
    }
  }
  loc += i;
  return i;
}
