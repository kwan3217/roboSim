#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

//HardwarePiInterfaceArduino interface;
Simulator interface;
double t[]           {0,  0,  2,  5,  7,  9,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,150,140,200,150,150,150};
OpenLoopGuidance guide(interface,t,servoChannel,servoCommand);

void setup() {
  printf("t,wheelCount,dt\n");
}

uint32_t wheelT,dt;
int32_t wheelCount,oldWheelCount;

void loop() {
  interface.update(0.002);
  interface.readOdometer(wheelT,wheelCount,dt);
  if(wheelCount!=oldWheelCount) {
	oldWheelCount=wheelCount;
	printf("%d,%d,%d\n",wheelT,wheelCount,dt);
  }
  guide.control();
  usleep(2000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
