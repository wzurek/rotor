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

#define FLAG_PRINT 1

class Receiver {

public:

  uint32_t FLAGS;

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

  // print status to serial port
  void print();

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

#define RECEIVER_PIN1 51
#define RECEIVER_PIN2 49
#define RECEIVER_PIN3 47
#define RECEIVER_PIN4 45
#define RECEIVER_PIN5 43
#define RECEIVER_PIN6 41
#define RECEIVER_PIN7 39
#define RECEIVER_PIN8 37
#define RECEIVER_PIN9 35
#define RECEIVER_PIN10 33

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

// receiver object
Receiver receiver(RECEIVER_PIN1, RECEIVER_PIN2, RECEIVER_PIN3, RECEIVER_PIN4,
    RECEIVER_PIN5, RECEIVER_PIN6, RECEIVER_PIN7, RECEIVER_PIN8, RECEIVER_PIN9,
    RECEIVER_PIN10);

#endif /* RECEIVER_H_ */
