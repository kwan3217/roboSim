
#include "SimHardware.h"
#include "Simulator.h"
#include "LogCSV.h"
#include "Simulator.h"
#include "HardwarePi.h"
#include "OpenLoopGuidance.h"
#include <stdio.h> //print statements for to debug

LogCSV logC("readOut.csv");
SimHardware simHard(0,0,0);
double t[]           {0,  0,  2,  5,  6,  6,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,110,140,150,190,150,150};
OpenLoopGuidance controller(simHard,t,servoChannel,servoCommand);
void setup(){
}
void loop(){
  logC.start(0);
  simHard.showVector(logC);
  logC.end();
  controller.navigate();
  controller.guide();
  controller.control();
}

int main(){
  setup();
  while(1) loop();
}
