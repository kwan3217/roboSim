#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>
#include <wiringPi.h>
#include "LogCSV.h"

HardwarePiInterfaceArduino interface;
//Simulator interface;
double t[]           {0,  0,  2,  5,  6,  6,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,155,140,200,200,150,150};
OpenLoopGuidance controller(interface,t,servoChannel,servoCommand);
LogCSV logC("odometer.csv");

void setup() {
  printf("WC,DT,T0,T1,ID\n");
  pinMode(23, INPUT);
  pinMode(24, INPUT);
}

uint32_t wheelT,dt;
int32_t wheelCount,oldWheelCount;

int ouf;
char buf[128];

void loop() {
  int e0=digitalRead(23);
  int e1=digitalRead(24);
  printf("%d %d\n",e0,e1);
  logC.start(0);
  logC.write(controller.time(),"t");
  logC.write(e0,"e0");
  logC.write(e1,"e1");
  logC.end();
  controller.navigate();
  controller.guide();
  controller.control();
  usleep(2000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
