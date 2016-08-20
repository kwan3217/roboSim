#include "HardwarePi.h"
#include "Simulator.h"
#include "OpenLoopGuidance.h"
#include <iostream>

HardwarePiInterfaceArduino interface;
//Simulator interface;
double t[]           {0,  0,  2,  5,  7,  9,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,150,140,200,150,150,150};
OpenLoopGuidance controller(interface,t,servoChannel,servoCommand);

void setup() {
  printf("WC,DT,T0,T1,ID\n");
}

uint32_t wheelT,dt;
int32_t wheelCount,oldWheelCount;

int ouf;
char buf[128];

void loop() {
  readI2Creg(interface.bus,0x55,0x00,buf,0x12);
   int32_t WC;
  uint32_t DT;
  uint32_t T0;
  uint32_t T1;
  uint16_t ID;
  WC=readBuf_le<int32_t>(buf, 0);
  DT=readBuf_le<uint32_t>(buf, 4);
  T0=readBuf_le<uint32_t>(buf, 8);
  T1=readBuf_le<uint32_t>(buf,12);
  ID=readBuf_le<int16_t>(buf,16);
  //  interface.readOdometer(wheelT,wheelCount,dt);
  if(wheelCount!=oldWheelCount) {
    oldWheelCount=wheelCount;
    printf("%d,%d,%d,%d,%04x\n",WC,DT,T0,T1,ID);
  }
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
