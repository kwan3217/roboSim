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

void setup() {
//  int whoami=interface.mpu.whoami();
//  printf("whoami: %02x",whoami);
//  while(guide.time()<15) {
  printf("t,ax,ay,az,gx,gy,gz,T\n");

}

void loop() {
//	interface.update(0.05);
//  guide.control();
//	std::cout << interface.time() << ',';
//	interface.showVector();
//	std::cout << std::endl;
  int16_t gx,gy,gz;
  interface.mpu.readGyro(gx,gy,gz);
  printf("%0.4f,%d,%d,%d\n",interface.time(),gx,gy,gz);
}

int main() {
  setup();
  for(;;) {
    loop();
  }
}
