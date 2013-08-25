/*
 * receiver.h
 *
 *  Created on: Aug 20, 2013
 *      Author: wzurek
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

#include <Arduino.h>

#define MAX_CHANNEL 10

#define REC_MIN 1100
#define REC_MAX 1900
#define REC_RANGE 800

class Receiver {

public:
  // define the receiver pins
  Receiver(uint32_t pin1, uint32_t pin2 = 0, uint32_t pin3 = 0, uint32_t pin4 =
      0, uint32_t pin5 = 0, uint32_t pin6 = 0, uint32_t pin7 = 0,
      uint32_t pin8 = 0, uint32_t pin9 = 0, uint32_t pin10 = 0);

  // start reading
  void start();

  // read single channel
  uint32_t read(uint32_t channel);

  // read all channels at once
  void readAll(uint32_t values[MAX_CHANNEL]);

  // return true if there is new data to read
  bool hasNewData();

private:

  static void interruptHandler(uint32_t pin);

  static void interruptHandler1();
  static void interruptHandler2();
  static void interruptHandler3();
  static void interruptHandler4();
  static void interruptHandler5();
  static void interruptHandler6();
  static void interruptHandler7();
  static void interruptHandler8();
  static void interruptHandler9();
  static void interruptHandler10();

protected:
  uint32_t locValues[MAX_CHANNEL];
  static uint32_t pins[MAX_CHANNEL];
  static volatile uint32_t values[MAX_CHANNEL];
  static volatile uint32_t bUpdateFlagsShared;
};

#endif /* RECEIVER_H_ */
