// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading and writing ten RC Channels using Arduino Due interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//
#include "receiver.h"
#include "global_objects.h"
#include <itoa.h>

// channel to pin mapping
uint32_t Receiver::pins[MAX_CHANNEL];

// values as read from the inputs
volatile uint32_t Receiver::values[MAX_CHANNEL];

// global flag to see if there is new data
volatile uint32_t Receiver::bUpdateFlagsShared;

void Receiver::interruptHandler1() {
  interruptHandler(0);
}
void Receiver::interruptHandler2() {
  interruptHandler(1);
}
void Receiver::interruptHandler3() {
  interruptHandler(2);
}
void Receiver::interruptHandler4() {
  interruptHandler(3);
}
void Receiver::interruptHandler5() {
  interruptHandler(4);
}
void Receiver::interruptHandler6() {
  interruptHandler(5);
}
void Receiver::interruptHandler7() {
  interruptHandler(6);
}
void Receiver::interruptHandler8() {
  interruptHandler(7);
}
void Receiver::interruptHandler9() {
  interruptHandler(8);
}
void Receiver::interruptHandler10() {
  interruptHandler(9);
}

void Receiver::interruptHandler(uint32_t channel) {
  static uint32_t ulStart[MAX_CHANNEL];
  if (digitalRead(pins[channel])) {
    ulStart[channel] = micros();
  } else {
    values[channel] = (uint32_t) (micros() - ulStart[channel]);
    bUpdateFlagsShared |= 1 << channel;
  }
}

Receiver::Receiver(uint32_t pin1, uint32_t pin2, uint32_t pin3, uint32_t pin4, uint32_t pin5, uint32_t pin6,
    uint32_t pin7, uint32_t pin8, uint32_t pin9, uint32_t pin10) {
  Receiver::pins[0] = pin1;
  Receiver::pins[1] = pin2;
  Receiver::pins[2] = pin3;
  Receiver::pins[3] = pin4;
  Receiver::pins[4] = pin5;
  Receiver::pins[5] = pin6;
  Receiver::pins[6] = pin7;
  Receiver::pins[7] = pin8;
  Receiver::pins[8] = pin9;
  Receiver::pins[9] = pin10;

  connected = false;
  connectionLost = false;

  for (int i = 0; i < MAX_CHANNEL; i++) {
    Receiver::values[i] = 0;
    locValues[i] = 0;
  }

}

bool Receiver::hasNewData() {
  return bUpdateFlagsShared;
}

void Receiver::start() {

  if (pins[0])
    attachInterrupt(pins[0], interruptHandler1, CHANGE);
  if (pins[1])
    attachInterrupt(pins[1], interruptHandler2, CHANGE);
  if (pins[2])
    attachInterrupt(pins[2], interruptHandler3, CHANGE);
  if (pins[3])
    attachInterrupt(pins[3], interruptHandler4, CHANGE);
  if (pins[4])
    attachInterrupt(pins[4], interruptHandler5, CHANGE);
  if (pins[5])
    attachInterrupt(pins[5], interruptHandler6, CHANGE);
  if (pins[6])
    attachInterrupt(pins[6], interruptHandler7, CHANGE);
  if (pins[7])
    attachInterrupt(pins[7], interruptHandler8, CHANGE);
  if (pins[8])
    attachInterrupt(pins[8], interruptHandler9, CHANGE);
  if (pins[9])
    attachInterrupt(pins[9], interruptHandler10, CHANGE);
}

void Receiver::readAll(uint32_t toReturn[MAX_CHANNEL]) {
  if (bUpdateFlagsShared) {
    // for some reason Eclipse does not recognize those function
    noInterrupts();
    for (int i = 0; i < MAX_CHANNEL; i++) {
      locValues[i] = values[i];
    }
    bUpdateFlagsShared = 0;
    // for some reason Eclipse does not recognize those function
    interrupts();
  }
  if (!connected && locValues[0] > 1000) {
    connected = true;
  }
  for (int i = 0; i < MAX_CHANNEL; i++) {
    toReturn[i] = locValues[i];
  }
}

void Receiver::print() {
  groundStation.beginMessage(CMD_RECEIVER); // message R
  groundStation.writeVUIntsField(1, locValues, MAX_CHANNEL);
  groundStation.finishMessage();
}
