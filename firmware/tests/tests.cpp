#include <iostream>
#include "ground_station.h"
using namespace std;

int testGroundStation() {

  Serial.setInput("some|rubbish!S1|other rubbish!S2|");
  groundStation.processCmds();
  cout<<"end"<<endl;
}

int main() {
  int result = 0;
  cout << "-- ground_station --\n" << endl;
  if (testGroundStation()) {
    result = 1;
  }
  return 0;
}
