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

void setup() {
  ouf=fileno(stdout);
//  int whoami=interface.mpu.whoami();
//  printf("whoami: %02x",whoami);
//  while(guide.time()<15) {
}

char buf[128];

void loop() {
//	interface.update(0.05);
//  guide.control();
//	std::cout << interface.time() << ',';
//	interface.showVector();
//	std::cout << std::endl;
  int16_t gx,gy,gz;
  interface.mpu.readGyro(gx,gy,gz);
  writeBuf_le(buf,0,uint32_t(interface.time()*1'000'000));
  writeBuf_le(buf,4,gx);
  writeBuf_le(buf,6,gy);
  writeBuf_le(buf,8,gz);
  write(ouf,buf,10);
  usleep(1000);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
