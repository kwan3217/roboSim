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

}

void loop() {
  guide.control();
}

int main() {
  setup();
//  int whoami=interface.mpu.whoami();
//  printf("whoami: %02x",whoami);
//  while(guide.time()<15) {
  for(;;) {
//	interface.update(0.05);
	loop();
//	std::cout << interface.time() << ',';
//	interface.showVector();
//	std::cout << std::endl;
        if(guide.time()>10) guide.reset();
  }
}
