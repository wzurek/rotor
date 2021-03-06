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

// normal receiver ranges
#define REC_MIN 1100
#define REC_MAX 1900
#define REC_RANGE 800
#define REC_MID 1500

// absolute receiver ranges
#define REC_AB_MIN 1000
#define REC_AB_MAX 2000
#define REC_AB_RANGE 1000

#define CH_PITCH 0
#define CH_ROL 1
#define CH_THROTTLE 2
#define CH_YAW 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9

class Receiver {

public:

  // define the receiver pins
  Receiver(uint32_t pin1, uint32_t pin2 = 0, uint32_t pin3 = 0, uint32_t pin4 =
      0, uint32_t pin5 = 0, uint32_t pin6 = 0, uint32_t pin7 = 0,
      uint32_t pin8 = 0, uint32_t pin9 = 0, uint32_t pin10 = 0);

  // start reading
  void start();

  // read all channels at once
  void readAll(uint32_t values[MAX_CHANNEL]);

  // return true if there is new data to read
  bool hasNewData();

  // print status to serial port
  void print();

  // if the receiver is connected.
  // false from boot until the connection has been made, true afterwards (even if the connection is lost)
  bool connected;

  // true if the receiver was connected, but connection has been lost
  // should be used for fail-safe mode
  bool connectionLost;

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
