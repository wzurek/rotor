#include <iostream>
#include "ground_station.h"
using namespace std;

int testGroundStation() {

  Serial.setInput("foo");
  groundStation.processCmds();
}

int main() {
  int result = 0;
  cout << "-- ground_station --\n" << endl;
  if (testGroundStation()) {
    result = 1;
  }
  return 0;
}
