#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;
//Simulator interface;
double t[]           {0,  0,  2,  5,  7,  9,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,150,140,200,150,150,150};
OpenLoopGuidance guide(interface,t,servoChannel,servoCommand);

int ouf;
char buf[128];

void setup() {

}

void loop() {
  guide.control();
  if(guide.time()>t[sizeof(t)/sizeof(t[0])-2]) guide.reset();
  usleep(2000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
